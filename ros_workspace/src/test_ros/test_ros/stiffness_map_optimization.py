import rclpy
from rclpy.node import Node
from sim_bridge.msg import SparseMatrix
from sim_bridge.srv import FactorGraphState, FocalLesionFactorGraphState
import numpy as np
import scipy.sparse as sp

from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Point, PoseStamped, Vector3Stamped

import trimesh
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation as R

import time


# K = sparse stiffness mat
# V = flattened list of vertices
# vs = indices of vertices in face
# barys = barycentric coords of point in face
def surface_point_compliance(K, V, vs, barys):
    N = K.shape[0]

    # applied force vector
    F = np.zeros((N,1))
    # compliance matrix - filled out column by column
    C = np.zeros((3,3))

    # the original position of the point on the face
    x_orig = np.zeros(3)
    for i in range(3):
        x_orig += V[3*vs[i] : 3*vs[i]+3]*barys[i]

    delta = 1e-6
    for dir in range(3):
        # set force
        for i in range(3):
            # vertex index
            v = vs[i]
            # distribute the force to the vertices in the face based on the bary coords
            F[3*v+dir] = barys[i]*delta
        
        # solve for vertex displacements
        u = sp.linalg.spsolve(K, F)
        V_new = V + u
        x_new = np.zeros(3)
        for i in range(3):
            x_new += V_new[3*vs[i] : 3*vs[i]+3]*barys[i]

        # put finite difference into compliance
        C[:,dir] = (x_new - x_orig) / delta

        F = np.zeros((N,1))

    
    C = 0.5 * (C + C.T)
    print(f"Compliance at point {vs} with barycentric coordinates {barys}:\n{C}")

    eigvals = np.linalg.eigvalsh(C)

    if np.all(eigvals > 0):
        print("SPD")
    else:
        print("Not SPD", eigvals)

    return C

# ---------------- geometry pipeline  ----------------
def plan_waypoints(V, F, lesion_vids,
                INNER_PCT=60, RADIUS=0.010, NU=10, NV=10):
    mesh = trimesh.Trimesh(vertices=V, faces=F, process=False)
    lesion_pts = V[lesion_vids]

    hull = mesh.convex_hull
    pq = trimesh.proximity.ProximityQuery(hull)
    dist_hull = np.abs(pq.signed_distance(mesh.triangles_center))
    inner = dist_hull > np.percentile(dist_hull, INNER_PCT)

    inner_fids = np.where(inner)[0]
    tree = cKDTree(lesion_pts)
    dist_les, _ = tree.query(mesh.triangles_center[inner_fids])
    region_fids = inner_fids[dist_les < RADIUS]

    patch = mesh.submesh([region_fids], append=True)
    fn, area = patch.face_normals, patch.area_faces
    m = (fn * area[:, None]).sum(0); m /= np.linalg.norm(m)
    c = patch.triangles_center.mean(0)
    ref = np.array([1., 0, 0]) if abs(m[0]) < 0.9 else np.array([0, 1., 0])
    e1 = np.cross(ref, m); e1 /= np.linalg.norm(e1)
    e2 = np.cross(m, e1)
    pv = patch.vertices
    a, b = (pv - c) @ e1, (pv - c) @ e2
    depth0 = ((pv - c) @ m).mean()

    origins, dirs, grid_ij = [], [], []
    for i, uu in enumerate(np.linspace(a.min(), a.max(), NU)):
        for j, vv in enumerate(np.linspace(b.min(), b.max(), NV)):
            base = c + depth0 * m + uu * e1 + vv * e2
            origins.append(base - 0.03 * m); dirs.append(m); grid_ij.append((i, j))
    origins, dirs = np.array(origins), np.array(dirs)
    grid_ij = np.array(grid_ij)

    loc, idx_ray, idx_tri = patch.ray.intersects_location(
        origins, dirs, multiple_hits=False)

    fid_global = region_fids[idx_tri]
    tri_verts = V[F[fid_global]]
    bary = trimesh.triangles.points_to_barycentric(tri_verts, loc)

    patch_oriented = patch.copy()
    trimesh.repair.fix_normals(patch_oriented)
    nrm = patch_oriented.face_normals[idx_tri].copy()
    k0 = int(np.argmin(np.linalg.norm(loc - loc.mean(0), axis=1)))
    if mesh.contains([loc[k0] + 0.001 * nrm[k0]])[0]:
        nrm = -nrm

    return fid_global, bary, loc, nrm, grid_ij[idx_ray]


def snake_order(grid_ij):
    order = np.lexsort((grid_ij[:, 1], grid_ij[:, 0]))
    result = []
    for row in np.unique(grid_ij[order, 0]):
        row_idx = order[grid_ij[order, 0] == row]
        if row % 2 == 1:
            row_idx = row_idx[::-1]
        result.extend(row_idx.tolist())
    return np.array(result, int)


class StiffnessMapOptimizer(Node):
    def __init__(self):
        super().__init__('matrix_subscriber')
        self.tip_sub = self.create_subscription(PoseStamped, '/sim/output/arm1_tip_frame', self.new_arm1_tip_frame, 10)
        self.force_pub = self.create_publisher(Vector3Stamped, '/sim/input/lesion_body_force', 10)

        self.client = self.create_client(FocalLesionFactorGraphState, '/sim/focal_lesion_factor_graph_state')

         # Wait for service to be available
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')
        self.get_logger().info("Service available!")

        self.req = FocalLesionFactorGraphState.Request()
        self.req.update_last_mesh = False
        self.req.compute_stiffness_matrix = True
        
        self.arm1_tip_pos = None
        
        self.timer = None

        # initial call to FG service to get the initial mesh
        self.init_timer = self.create_timer(1, self.send_initial_request)

    def new_arm1_tip_frame(self, msg):
        point = msg.pose.position
        self.arm1_tip_pos = np.array([point.x, point.y, point.z])

    def send_initial_request(self):
        self.init_timer.cancel()  # run once

        self.get_logger().info("Sending initial request...")
        future = self.client.call_async(self.req)
        future.add_done_callback(self.handle_initial_response)

    def handle_initial_response(self, future):
        try:
            response = future.result()
            self.req.update_last_mesh = False
            self.req.compute_stiffness_matrix = True

            self.get_logger().info("Response received!")
            self.get_logger().info(f'Initial mesh: {response.fg_state.sim_mesh.vertices.size} vertices (total size), {response.fg_state.sim_mesh.faces.size} faces (total size), and {response.fg_state.sim_mesh.elements.size} elements (total size)')

            # extract data from response
            m = response.fg_state.sim_mesh
            V_flat = np.asarray(m.vertices.data, float)
            V = np.asarray(m.vertices.data, float).reshape(-1, 3)
            F = np.asarray(m.faces.data, np.int32).reshape(-1, 3)
            lesion_vids = np.asarray(response.lesion_vertices, np.int32)
            self.get_logger().info(
                f'mesh: {len(V)} verts, {len(F)} faces, {len(lesion_vids)} lesion')

            # plan waypoints
            fid, bary, pos_w, nrm_w, grid_ij = plan_waypoints(V, F, lesion_vids)
            order = snake_order(grid_ij)
            fid, bary = fid[order], bary[order]
            pos_w, nrm_w = pos_w[order], nrm_w[order]

            stiffness_mat = response.fg_state.sim_stiffness_mat
            # use SciPy COO format to build sparse matrix
            sparse_mat_coo = sp.coo_matrix((stiffness_mat.values, (stiffness_mat.row_indices, stiffness_mat.col_indices)), shape=(stiffness_mat.rows,stiffness_mat.cols))
            # convert to CSR (compressed sparse row) - recommended for most matrix operations
            sparse_mat = sparse_mat_coo.tocsr()

            # print out the 10 smallest eigenvalues of the stiffness matrix - they should be nonzero
            # if 6 are zero, then rigid body modes are not being constrained ==> stiffness matrix not invertible
            vals = sp.linalg.eigsh(sparse_mat, k=10, which='SA', return_eigenvectors=False)
            print(np.sort(vals))

            # get apparent surface compliances at each point
            Cs = []
            for f, b in zip(fid, bary):
                print(f"Face ind: {f}")
                Cs.append(surface_point_compliance(sparse_mat, V_flat, F[f,:], b))

            

        except Exception as e:
            self.get_logger().error(f'Initial call failed: {e}')

    # optimization objective
    #  - Given lesion body force F, applies the force in the sim, waits ~3 sec, then finds the resulting stiffness map
    def optimization_objective(self, F):
        # Step 1: apply force in the lesion force sim
        msg = Vector3Stamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.vector.x = F[0]
        msg.vector.y = F[1]
        msg.vector.z = F[2]
        self.force_pub.publish(msg)

        # Step 2: wait for sim to converge to roughly steady-state
        time.sleep(3)

        # Step 3: get sim state
        response = self.client.call(self.req)
        fg_state = response.fg_state
        stiffness_mat = fg_state.sim_stiffness_mat

        # use SciPy COO format to build sparse matrix
        sparse_mat_coo = sp.coo_matrix((stiffness_mat.values, (stiffness_mat.row_indices, stiffness_mat.col_indices)), shape=(stiffness_mat.rows,stiffness_mat.cols))
        # convert to CSR (compressed sparse row) - recommended for most matrix operations
        sparse_mat = sparse_mat_coo.tocsr()

        fg_mesh = response.fg_state.sim_mesh
        fg_vertices = np.asarray(fg_mesh.vertices.data, float).reshape(-1, 3)
        fg_faces = np.asarray(fg_mesh.faces.data, np.int32).reshape(-1, 3)
        mesh = trimesh.Trimesh(vertices=fg_vertices, faces=fg_faces, process=False)

        # Step 4: get closest surface points to the original palpation points
        

        # Step 4: get apparent surface stiffness in palpation directions
        

def main(args=None):
    rclpy.init(args=args)
    node = StiffnessMapOptimizer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
