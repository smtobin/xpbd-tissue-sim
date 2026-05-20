import rclpy
from rclpy.node import Node
from sim_bridge.msg import SparseMatrix
from sim_bridge.srv import FactorGraphState, FocalLesionFactorGraphState
import numpy as np
import scipy.sparse as sp


class FocalLesionFGClient(Node):
    def __init__(self):
        super().__init__('matrix_subscriber')
        self.client = self.create_client(FocalLesionFactorGraphState, '/sim/focal_lesion_factor_graph_state')

         # Wait for service to be available
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')
        self.get_logger().info("Service available!")

        self.req = FocalLesionFactorGraphState.Request()
        self.req.update_last_mesh = False
        self.req.compute_stiffness_matrix = True
        
        
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
            self.timer = self.create_timer(3.0, self.send_request)

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
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

        

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
