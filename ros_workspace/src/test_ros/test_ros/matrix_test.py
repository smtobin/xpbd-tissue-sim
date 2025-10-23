import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Int32MultiArray
import numpy as np


class MatrixSubscriber(Node):
    def __init__(self):
        super().__init__('matrix_subscriber')
        self.vertices_subscription = self.create_subscription(
            Float64MultiArray,
            '/output/vertices_mat_0',
            self.matrix_callback,
            10)
        self.elements_subscription = self.create_subscription(
            Int32MultiArray,
            '/output/elements_mat_0',
            self.matrix_callback,
            10
        )

    def matrix_callback(self, msg):
        # Extract dimensions
        if len(msg.layout.dim) < 2:
            self.get_logger().warn('Invalid matrix dimensions')
            return
        
        rows = msg.layout.dim[0].size
        cols = msg.layout.dim[1].size
        
        # Convert flat data to numpy matrix
        matrix = np.array(msg.data).reshape(rows, cols)
        
        self.get_logger().info(f'Received {rows}x{cols} matrix:')
        self.get_logger().info(f'\n{matrix}')

def main(args=None):
    rclpy.init(args=args)
    node = MatrixSubscriber()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
