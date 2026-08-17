import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
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
import scipy.optimize as opt
import matplotlib.pyplot as plt

import time
import threading


# K = sparse stiffness mat
# V = flattened list of vertices
# vs = indices of vertices in face
# barys = barycentric coords of point in face
def surface_point_compliance(factorized_K, V, vs, barys):
    N = V.size

    # applied force vector
    F = np.zeros(N, dtype=V.dtype)
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
        u = factorized_K(F)
        V_new = V + u
        x_new = np.zeros(3)
        for i in range(3):
            x_new += V_new[3*vs[i] : 3*vs[i]+3]*barys[i]

        # put finite difference into compliance
        C[:,dir] = (x_new - x_orig) / delta

        F = np.zeros(N, dtype=V.dtype)

    
    C = 0.5 * (C + C.T)
    # print(f"Compliance at point {vs} with barycentric coordinates {barys}:\n{C}")

    # eigvals = np.linalg.eigvalsh(C)

    # if np.all(eigvals > 0):
    #     print("SPD")
    # else:
    #     print("Not SPD", eigvals)

    return C

# computes the tetrahedral volume of 4 vertices
def elementVolume(v1, v2, v3, v4):
    X = np.zeros((3,3))
    X[:,0] = (v1 - v4)
    X[:,1] = (v2 - v4)
    X[:,2] = (v3 - v4)

    return np.abs(np.linalg.det(X) / 6.0)

# evolves the initial tissue state given the stiffness matrix and an applied lesion body force
#  lesion_body_force - the distributed body force applied to the lesion (N/m^3)
#  factorized_K - the factorized stiffness matrix in the initial configuration (using scipy.sparse.linalg.factorized)
#  initial_V - flattened list of the initial vertices
#  lesion_elements - Nx4 matrix of vertex indices corresponding to the lesion elements
def getTissueStateGivenLesionForce(lesion_body_force, factorized_K, initial_V, lesion_elements):
    N = initial_V.size
    # compute the force vector given the lesion body force and the lesion elements
    F = np.zeros(N, dtype=initial_V.dtype)
    for elem in lesion_elements:
        # compute volume
        v1 = initial_V[3*elem[0]:3*elem[0]+3]
        v2 = initial_V[3*elem[1]:3*elem[1]+3]
        v3 = initial_V[3*elem[2]:3*elem[2]+3]
        v4 = initial_V[3*elem[3]:3*elem[3]+3]
        volume = elementVolume(v1, v2, v3, v4)
        # compute total force on element
        total_force = volume * lesion_body_force
        # distribute to each vertex in the element
        for v in elem:
            F[3*v:3*v+3] += 0.25*total_force

    # given the force vector and the factorized stiffness matrix, compute the vertex offsets (Ku = F)
    u = factorized_K(F)
    return initial_V + u

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

class PalpationMeasurement:
    def __init__(self, location, direction, stiffness):
        self.location = location
        self.direction = direction
        self.stiffness = stiffness


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

        # palpation "measurements" - simulated palpation data that we are comparing to
        # array of PalpationMeasurement
        self.palpation_measurements = []

        # initial call to FG service to get the initial mesh
        self.init_timer = self.create_timer(1, self.send_initial_request)

        # == for linear optimization ==
        # store the stiffness matrix in the initial configuration (as its LU decomposition - this is only what's available in scipy)
        # this is what is returned by scipy.sparse.linalg.factorized
        self.factorized_stiffness = None

        # store the initial vertices
        # using the stiffness matrix, we will repeatedly evolve the initial vertices the new approximate state using applied lesion body force
        self.initial_vertices = []
        self.initial_vertices_flat = []

        # store the initial faces
        self.initial_faces = []

        # store lesion elements
        # required for computing the applied body forces
        self.lesion_elements = []

    def new_arm1_tip_frame(self, msg):
        point = msg.pose.position
        self.arm1_tip_pos = np.array([point.x, point.y, point.z])

    def send_initial_request(self):
        self.init_timer.cancel()  # run once

        self.get_logger().info("Sending initial request...")
        future = self.client.call_async(self.req)
        future.add_done_callback(self.handle_initial_response)

    def handle_initial_response(self, future):
        # try:
            response = future.result()
            self.req.update_last_mesh = False
            self.req.compute_stiffness_matrix = True

            self.get_logger().info("Response received!")
            self.get_logger().info(f'Initial mesh: {response.fg_state.sim_mesh.vertices.size} vertices (total size), {response.fg_state.sim_mesh.faces.size} faces (total size), and {response.fg_state.sim_mesh.elements.size} elements (total size)')

            # extract data from response
            m = response.fg_state.sim_mesh
            V_flat = np.asarray(m.vertices.data, float)
            self.initial_vertices_flat = V_flat
            print(f"V_flat size:{V_flat.shape}")
            V = np.asarray(m.vertices.data, float).reshape(-1, 3)
            self.initial_vertices = V
            F = np.asarray(m.faces.data, np.int32).reshape(-1, 3)
            self.initial_faces = F
            E = np.asarray(m.elements.data, np.int32).reshape(-1, 4)
            lesion_vids = np.asarray(response.lesion_vertices, np.int32)
            self.get_logger().info(
                f'mesh: {len(V)} verts, {len(F)} faces, {len(lesion_vids)} lesion vertices')
            lesion_elements_inds = np.asarray(response.lesion_elements, np.int32)
            self.lesion_faces = np.asarray(response.lesion_faces.data, np.int32).reshape(-1, 3)
            self.lesion_elements = E[lesion_elements_inds]
            print(self.lesion_faces)

            # plan waypoints
            fid, bary, pos_w, nrm_w, grid_ij = plan_waypoints(V, F, lesion_vids)
            order = snake_order(grid_ij)
            fid, bary = fid[order], bary[order]
            pos_w, nrm_w = pos_w[order], nrm_w[order]

            stiffness_mat = response.fg_state.sim_stiffness_mat
            # use SciPy COO format to build sparse matrix
            sparse_mat_coo = sp.coo_matrix(( np.asarray(stiffness_mat.values, dtype=np.float64), (stiffness_mat.row_indices, stiffness_mat.col_indices)), shape=(stiffness_mat.rows,stiffness_mat.cols))
            # convert to CSR (compressed sparse row) - recommended for most matrix operations
            sparse_mat = sparse_mat_coo.tocsr()

            # factorize the stiffness matrix and store it
            self.factorized_stiffness = sp.linalg.factorized(sparse_mat)

            # print out the 10 smallest eigenvalues of the stiffness matrix - they should be nonzero
            # if 6 are zero, then rigid body modes are not being constrained ==> stiffness matrix not invertible
            vals = sp.linalg.eigsh(sparse_mat, k=10, which='SA', return_eigenvectors=False)
            print(np.sort(vals))

            # get apparent surface compliances at each point
            Cs = []
            for i, (f, b) in enumerate(zip(fid, bary)):
                C = surface_point_compliance(self.factorized_stiffness, V_flat, F[f,:], b)
                Cs.append(C)
                # F = C^-1 * u
                # ==> stiffness = (C^-1 * u).norm() (assuming that u is unit direction)
                stiffness = np.linalg.norm(np.linalg.inv(C) @ nrm_w[i]) / np.linalg.norm(nrm_w[i])
                self.palpation_measurements.append( PalpationMeasurement(pos_w[i], nrm_w[i], stiffness))

            # start the optimization in a new thread (so that the optimization objective can be synchronous)
            # threading.Thread(target=self.run_nonlinear_optimization, daemon=True).start()
            threading.Thread(target=self.run_linear_optimization, daemon=True).start()

        # except Exception as e:
        #     self.get_logger().error(f'Initial call failed: {e}')

    # runs the "linear" optimization
    # the stiffness matrix from the initial tissue state is always used
    def run_linear_optimization(self):
        x0 = np.array([5,5,5])
        result = opt.minimize(
            self.linear_optimization_objective,
            x0=x0,
            method="Powell",
            # bounds=[
            #     (0, 20),
            #     (0, 20),
            #     (0, 20),
            # ],
            options={
                "maxiter": 5,
                "xtol": 0.1,
                "ftol": 1e-3,
            }
        )
        print(f"Optimization result: {result}")

        self.visualizeStiffnessMap(result.x)
        self.visualizeStiffnessMap(x0)
        

    def linear_optimization_objective(self, F):
        # scale F by 1e6 for numerical sensitivity
        force = np.asarray(F) * 1e6

        # Step 1: get the new tissue state given the applied body force to the lesion
        print(f"  Updating tissue state with new lesion force {force}...")
        new_V = getTissueStateGivenLesionForce(force, self.factorized_stiffness, self.initial_vertices_flat, self.lesion_elements)

        # Step 2: get closest surface points to the original palpation points
        print(f"  Getting closest points on surface to palpation measurements...")
        palpation_locations = np.vstack(
            [meas.location for meas in self.palpation_measurements]
        )

        new_V_mat = new_V.reshape(-1, 3)
        mesh = trimesh.Trimesh(vertices=new_V_mat, faces=self.initial_faces, process=False)
        closest_points, distances, triangle_ids = trimesh.proximity.closest_point(mesh, palpation_locations)
        triangles = mesh.vertices[mesh.faces[triangle_ids]]
        barys = trimesh.triangles.points_to_barycentric(triangles, closest_points)

        # Step 3: get apparent surface stiffness in palpation directions
        # each entry in the array corresponds to the entry in the self.palpation_measurements array with the same index
        print(f"  Computing apparent stiffnesses at surface points...")
        new_palpation_measurements = []
        for i, (f, b) in enumerate(zip(triangle_ids, barys)):
            C = surface_point_compliance(self.factorized_stiffness, new_V, self.initial_faces[f,:], b)
            # F = C^-1 * u
            # ==> stiffness = (C^-1 * u).norm() (assuming that u is unit direction)
            dir = self.palpation_measurements[i].direction
            stiffness = np.linalg.norm(np.linalg.inv(C) @ dir) / np.linalg.norm(dir)
            new_palpation_measurements.append(PalpationMeasurement(closest_points[i], dir, stiffness))

        # Step 4: compute objective
        print(f"  Computing objective...")
        stiffness_weight = 1
        location_weight = 1e10
        stiffness_penalty = 0   # cost of stiffnesses being different
        location_penalty = 0    # cost of palpation locations being different
        for meas, sim in zip(self.palpation_measurements, new_palpation_measurements):
            # compute cost for location difference
            loc_diff = np.array(meas.location - sim.location)
            location_penalty += location_weight * (loc_diff.transpose() @ loc_diff)
            # compute cost for stiffness difference
            stiffness_diff = meas.stiffness - sim.stiffness
            stiffness_penalty += stiffness_weight * (stiffness_diff**2)

        # total objective
        # E = stiffness_penalty + location_penalty
        E = stiffness_penalty + location_penalty
        print(f"  Done. Stiffness cost: {stiffness_penalty},   Location penalty: {location_penalty},  Total objective: {E}")
        return E

    # runs the "nonlinear" optimization
    # given lesion body force, applies the force in the sim and waits for convergence before calculating the resulting stiffness map
    def run_nonlinear_optimization(self):
        x0 = np.array([2, 2, 2])
        result = opt.minimize(
            self.nonlinear_optimization_objective,
            x0=x0,
            method="Powell",
            # bounds=[
            #     (0, 20),
            #     (0, 20),
            #     (0, 20),
            # ],
            options={
                "maxiter": 50,
                "xtol": 0.1,
                "ftol": 1e-3,
            }
        )
        print(f"Optimization result: {result}")

    # optimization objective
    #  - Given lesion body force F (scaled up by 1e6), applies the force in the sim, waits ~3 sec, then finds the resulting stiffness map
    def nonlinear_optimization_objective(self, F):
        force = np.asarray(F) * 1e6
        print(f"=== Optimization objective start ===")
        print(f"  Applying body force {F} MN/m^3 to sim...")
        # Step 1: apply force in the lesion force sim
        msg = Vector3Stamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.vector.x = force[0]
        msg.vector.y = force[1]
        msg.vector.z = force[2]
        self.force_pub.publish(msg)

        # Step 2: wait for sim to converge to roughly steady-state
        wait_time = 5
        print(f"  Waiting {wait_time} seconds for convergence...")
        time.sleep(wait_time)

        # Step 3: get sim state
        print(f"  Getting new simulation state...")
        future = self.client.call_async(self.req)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        fg_state = response.fg_state
        stiffness_mat = fg_state.sim_stiffness_mat

        print(f"  Extracting data from service response...")
        # use SciPy COO format to build sparse matrix
        sparse_mat_coo = sp.coo_matrix((stiffness_mat.values, (stiffness_mat.row_indices, stiffness_mat.col_indices)), shape=(stiffness_mat.rows,stiffness_mat.cols))
        # convert to CSR (compressed sparse row) - recommended for most matrix operations
        sparse_mat = sparse_mat_coo.tocsr()
        factorized_stiffness = sp.linalg.factorized(sparse_mat)

        fg_mesh = response.fg_state.sim_mesh
        fg_vertices = np.asarray(fg_mesh.vertices.data, float).reshape(-1, 3)
        fg_vertices_flat = np.asarray(fg_mesh.vertices.data, float)
        fg_faces = np.asarray(fg_mesh.faces.data, np.int32).reshape(-1, 3)
        mesh = trimesh.Trimesh(vertices=fg_vertices, faces=fg_faces, process=False)

        # Step 4: get closest surface points to the original palpation points
        print(f"  Getting closest points on surface to palpation measurements...")
        palpation_locations = [meas.location for meas in self.palpation_measurements]
        palpation_locations = np.vstack(
            [meas.location for meas in self.palpation_measurements]
        )
        closest_points, distances, triangle_ids = trimesh.proximity.closest_point(mesh, palpation_locations)
        triangles = mesh.vertices[mesh.faces[triangle_ids]]
        barys = trimesh.triangles.points_to_barycentric(triangles, closest_points)

        # Step 5: get apparent surface stiffness in palpation directions
        # each entry in the array corresponds to the entry in the self.palpation_measurements array with the same index
        print(f"  Computing apparent stiffnesses at surface points...")
        new_palpation_measurements = []
        for i, (f, b) in enumerate(zip(triangle_ids, barys)):
            C = surface_point_compliance(factorized_stiffness, fg_vertices_flat, fg_faces[f,:], b)
            # F = C^-1 * u
            # ==> stiffness = (C^-1 * u).norm() (assuming that u is unit direction)
            dir = self.palpation_measurements[i].direction
            stiffness = np.linalg.norm(np.linalg.inv(C) @ dir) / np.linalg.norm(dir)
            new_palpation_measurements.append(PalpationMeasurement(closest_points[i], dir, stiffness))
        

        # Step 6: compute objective
        print(f"  Computing objective...")
        stiffness_weight = 1
        location_weight = 1e10
        stiffness_penalty = 0   # cost of stiffnesses being different
        location_penalty = 0    # cost of palpation locations being different
        for meas, sim in zip(self.palpation_measurements, new_palpation_measurements):
            # compute cost for location difference
            loc_diff = np.array(meas.location - sim.location)
            location_penalty += location_weight * (loc_diff.transpose() @ loc_diff)
            # compute cost for stiffness difference
            stiffness_diff = meas.stiffness - sim.stiffness
            stiffness_penalty += stiffness_weight * (stiffness_diff**2)

        # total objective
        # E = stiffness_penalty + location_penalty
        E = location_penalty

        print(f"  Done. Stiffness cost: {stiffness_penalty},   Location penalty: {location_penalty},  Total objective: {E}")

        return E
        
    def visualizeStiffnessMap(self, force):
        # visualize the result
        new_V = getTissueStateGivenLesionForce(force*1e6, self.factorized_stiffness, self.initial_vertices_flat, self.lesion_elements)
        new_V_mat = new_V.reshape(-1, 3)
        mesh = trimesh.Trimesh(vertices=new_V_mat, faces=self.initial_faces, process=False)

        palpation_locations = np.vstack(
            [meas.location for meas in self.palpation_measurements]
        )
        closest_points, distances, triangle_ids = trimesh.proximity.closest_point(mesh, palpation_locations)
        triangles = mesh.vertices[mesh.faces[triangle_ids]]
        barys = trimesh.triangles.points_to_barycentric(triangles, closest_points)

        new_palpation_measurements = []
        for i, (f, b) in enumerate(zip(triangle_ids, barys)):
            C = surface_point_compliance(self.factorized_stiffness, new_V, self.initial_faces[f,:], b)
            # F = C^-1 * u
            # ==> stiffness = (C^-1 * u).norm() (assuming that u is unit direction)
            dir = self.palpation_measurements[i].direction
            stiffness = np.linalg.norm(np.linalg.inv(C) @ dir) / np.linalg.norm(dir)
            new_palpation_measurements.append(PalpationMeasurement(closest_points[i], dir, stiffness))

        mesh.visual.face_colors[:, 3] = 100 # make mesh translucent

        lesion_mesh = trimesh.Trimesh(vertices=new_V_mat, faces=self.lesion_faces, process=False)
        lesion_mesh.fix_normals()
        palpation_locations = np.vstack(
            [meas.location for meas in new_palpation_measurements]
        )
        palpation_directions = [meas.direction for meas in new_palpation_measurements]
        stiffness = np.array([meas.stiffness for meas in new_palpation_measurements])
        norm = plt.Normalize(vmin=stiffness.min(), vmax=stiffness.max())
        cmap = plt.get_cmap("coolwarm")
        colors = (cmap(norm(stiffness)) * 255).astype(np.uint8)
        point_cloud = trimesh.points.PointCloud(palpation_locations, colors=colors)
        scene = trimesh.Scene()
        # scene.add_geometry(mesh)
        scene.add_geometry(lesion_mesh)
        # scene.add_geometry(point_cloud)
        
        # --------------------------------
        # Create arrows
        # --------------------------------
        # arrow_length = 5e-3
        shaft_radius = 2e-4
        head_radius = 4e-4
        head_length = 6e-4
        lengths = np.interp(
            stiffness,
            (stiffness.min(), stiffness.max()),
            (2e-3, 7e-3)
        )

        for point, direction, color, arrow_length in zip(palpation_locations, palpation_directions, colors, lengths):

            # Normalize direction
            direction = np.asarray(direction)
            direction = direction / np.linalg.norm(direction)

            # Arrow consists of shaft + cone
            shaft_length = arrow_length - head_length

            shaft = trimesh.creation.cylinder(
                radius=shaft_radius,
                height=shaft_length,
                sections=16
            )

            head = trimesh.creation.cone(
                radius=head_radius,
                height=head_length,
                sections=16
            )

            # Default cylinder/cone axis is Z.
            # Rotate Z axis onto the desired direction.
            rotation = trimesh.geometry.align_vectors(
                [0, 0, 1],
                -direction
            )

            shaft.apply_transform(rotation)
            head.apply_transform(rotation)

            # Position shaft and head along the direction
            shaft.apply_translation(
                point + head_length*direction + direction * (shaft_length / 2)
            )

            head.apply_translation(point + head_length*direction
                # point + direction * (
                #     -shaft_length
                # )
            )

            # Color them
            shaft.visual.face_colors = color
            head.visual.face_colors = color

            scene.add_geometry(shaft)
            scene.add_geometry(head)

        scene.show()

def main(args=None):
    import sys
    print("Executable:", sys.executable)
    print("Prefix:", sys.prefix)
    print("Path:")
    for p in sys.path:
        print(" ", p)

    rclpy.init(args=args)
    node = StiffnessMapOptimizer()

    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
