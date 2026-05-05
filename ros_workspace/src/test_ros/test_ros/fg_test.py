import rclpy
from rclpy.node import Node
from sim_bridge.msg import SparseMatrix
from sim_bridge.srv import FactorGraphState
import numpy as np
import scipy.sparse as sp


class FGClient(Node):
    def __init__(self):
        super().__init__('matrix_subscriber')
        self.client = self.create_client(FactorGraphState, '/sim/factor_graph_state')

         # Wait for service to be available
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')

        self.req = FactorGraphState.Request()

        # Call service once per second
        self.timer = self.create_timer(3.0, self.send_request)

    def send_request(self):
        self.get_logger().info(f'Sending request for factor graph state...')

        future = self.client.call_async(self.req)
        future.add_done_callback(self.handle_response)

    def handle_response(self, future):
        try:
            response = future.result()

            fg_state = response.fg_state
            stiffness_mat = fg_state.sim_stiffness_mat
            # use SciPy COO format to build sparse matrix
            sparse_mat_coo = sp.coo_matrix((stiffness_mat.values, (stiffness_mat.row_indices, stiffness_mat.col_indices)), shape=(stiffness_mat.rows,stiffness_mat.cols))
            # convert to CSR (compressed sparse row) - recommended for most matrix operations
            sparse_mat = sparse_mat_coo.tocsr()

            self.get_logger().info(f'Received {fg_state.sim_mesh.vertices.size} vertices, {fg_state.sim_mesh.faces.size} faces, and {fg_state.sim_mesh.elements.size}')
            self.get_logger().info(f'Received {stiffness_mat.rows}x{stiffness_mat.cols} sparse matrix with {sparse_mat.count_nonzero()} nonzero entries')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

        

def main(args=None):
    rclpy.init(args=args)
    node = FGClient()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
