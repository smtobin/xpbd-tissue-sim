import rclpy
from rclpy.node import Node
from sim_bridge.msg import SparseMatrix
from sim_bridge.srv import FactorGraphState, FocalLesionFactorGraphState
import numpy as np
import scipy.sparse as sp

from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Point


class FocalLesionFGClient(Node):
    def __init__(self):
        super().__init__('matrix_subscriber')
        self.client = self.create_client(FocalLesionFactorGraphState, '/sim/focal_lesion_factor_graph_state')

        # publishers for visualization
        self.prostate_mesh_publisher = self.create_publisher(
            MarkerArray,
            'prostate_mesh_markers',
            10
        )

        self.lesion_mesh_publisher = self.create_publisher(
            MarkerArray,
            'lesion_mesh_markers',
            10
        )

        # colors for output meshes
        self.prostate_color = [1.0, 0.6, 0.5, 0.2]
        self.lesion_color = [1.0, 0.0, 1.0, 1.0]

         # Wait for service to be available
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')
        self.get_logger().info("Service available!")

        self.req = FocalLesionFactorGraphState.Request()
        self.req.update_last_mesh = False
        self.req.compute_stiffness_matrix = False
        
        
        self.timer = None

        # initial call to FG service to get the initial mesh
        self.init_timer = self.create_timer(1, self.send_initial_request)

    def send_initial_request(self):
        self.init_timer.cancel()  # run once

        self.get_logger().info("Sending initial request...")
        future = self.client.call_async(self.req)
        future.add_done_callback(self.handle_initial_response)

    def handle_initial_response(self, future):
        try:
            response = future.result()
            self.req.update_last_mesh = True
            self.req.last_mesh = response.fg_state.sim_mesh

            self.get_logger().info("Response received!")
            self.get_logger().info(f'Initial mesh: {response.fg_state.sim_mesh.vertices.size} vertices (total size), {response.fg_state.sim_mesh.faces.size} faces (total size), and {response.fg_state.sim_mesh.elements.size} elements (total size)')

            # now start periodic timer
            self.timer = self.create_timer(1.0, self.send_request)

        except Exception as e:
            self.get_logger().error(f'Initial call failed: {e}')

    def send_request(self):
        self.get_logger().info(f'Sending request for factor graph state...')

        future = self.client.call_async(self.req)
        future.add_done_callback(self.handle_response)

    def handle_response(self, future):
        try:
            response = future.result()

            fg_state = response.fg_state
            updated_last_mesh = response.updated_last_mesh
            stiffness_mat = fg_state.sim_stiffness_mat
            
            self.req.last_mesh = updated_last_mesh

            # use SciPy COO format to build sparse matrix
            sparse_mat_coo = sp.coo_matrix((stiffness_mat.values, (stiffness_mat.row_indices, stiffness_mat.col_indices)), shape=(stiffness_mat.rows,stiffness_mat.cols))
            # convert to CSR (compressed sparse row) - recommended for most matrix operations
            sparse_mat = sparse_mat_coo.tocsr()

            self.get_logger().info(f'Current mesh: {fg_state.sim_mesh.vertices.size} vertices (total), {fg_state.sim_mesh.faces.size} faces (total), and {fg_state.sim_mesh.elements.size} elements (total)')
            self.get_logger().info(f'Updated last mesh: {updated_last_mesh.vertices.size} vertices (total), {updated_last_mesh.faces.size} faces (total), and {updated_last_mesh.elements.size} elements (total)')
            self.get_logger().info(f'Received {stiffness_mat.rows}x{stiffness_mat.cols} sparse matrix with {sparse_mat.count_nonzero()} nonzero entries')

            self.publish_mesh(fg_state.sim_mesh.vertices, fg_state.sim_mesh.faces, self.prostate_mesh_publisher, self.prostate_color)
            self.publish_mesh(fg_state.sim_mesh.vertices, response.lesion_faces, self.lesion_mesh_publisher, self.lesion_color)
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')


    def make_point(self, xyz):
        p = Point()
        p.x = float(xyz[0])
        p.y = float(xyz[1])
        p.z = float(xyz[2])
        return p
    

    def publish_mesh(self, vertices_list, faces_list, publisher, color):

        marker = Marker()

        marker.header.frame_id = "sim/world"
        marker.header.stamp = self.get_clock().now().to_msg()

        marker.ns = "mesh"
        marker.id = 0

        marker.type = Marker.TRIANGLE_LIST
        marker.action = Marker.ADD

        # Pose
        marker.pose.orientation.w = 1.0

        # Scale must be 1 for TRIANGLE_LIST
        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0

        # Color
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = color[3]

        # Add triangles
        faces = np.reshape(faces_list.data, (-1,3))
        vertices = np.reshape(vertices_list.data, (-1,3))
        for face in faces:
            for vertex_index in face:

                vx, vy, vz = vertices[vertex_index]

                p = Point()
                p.x = vx
                p.y = vy
                p.z = vz

                marker.points.append(p)

        marker_array = MarkerArray()
        marker_array.markers.append(marker)

        line_marker = Marker()

        line_marker.header.frame_id = "sim/world"
        line_marker.header.stamp = self.get_clock().now().to_msg()

        line_marker.ns = "mesh_edges"
        line_marker.id = 1

        line_marker.type = Marker.LINE_LIST
        line_marker.action = Marker.ADD

        line_marker.pose.orientation.w = 1.0

        # LINE WIDTH
        line_marker.scale.x = 0.00007

        # Black edges
        line_marker.color.r = 0.0
        line_marker.color.g = 0.0
        line_marker.color.b = 0.0
        line_marker.color.a = 1.0

        #
        # Add triangle edges
        #
        for face in faces:

            edges = [
                (face[0], face[1]),
                (face[1], face[2]),
                (face[2], face[0]),
            ]

            for start_idx, end_idx in edges:

                line_marker.points.append(
                    self.make_point(vertices[start_idx])
                )

                line_marker.points.append(
                    self.make_point(vertices[end_idx])
                )

        marker_array.markers.append(line_marker)

        publisher.publish(marker_array)
        

def main(args=None):
    rclpy.init(args=args)
    node = FocalLesionFGClient()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
