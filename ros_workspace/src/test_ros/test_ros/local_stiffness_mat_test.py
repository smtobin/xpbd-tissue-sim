import rclpy
from rclpy.node import Node
from sim_bridge.msg import PointOnFace, LocalStiffnessMatrix
import numpy as np
import scipy.sparse as sp
import time


class LocalStiffnessMatTester(Node):
    def __init__(self):
        super().__init__('stiffness_matrix_test')
        
        self.stiffness_mat_subscription = self.create_subscription(
            LocalStiffnessMatrix,
            'sim/output/local_stiffness_mats',
            self.stiffness_mat_callback,
            10)

        self.query_point_publisher = self.create_publisher(
            PointOnFace,
            'sim/input/stiffness_query_points',
            10
        )

        self.send_query_points()

    def send_query_points(self):
        time.sleep(1.0)

        for i in range(0,10):
            msg = PointOnFace()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.face_ind = i
            msg.face_barys.x = 1.0/3.0
            msg.face_barys.y = 1.0/3.0
            msg.face_barys.z = 1.0/3.0
            
            print(f"Sending query for face index: {msg.face_ind} and barycentric coords: ({msg.face_barys.x}, {msg.face_barys.y}, {msg.face_barys.z})")

            self.query_point_publisher.publish(msg)

            time.sleep(0.1)

            

    def stiffness_mat_callback(self, msg):
        print("Received message!")

        K = np.array(msg.stiffness_mat, dtype=np.float32).reshape((3, 3))
        print(f"Stiffness at Face {msg.point_on_face.face_ind}, Bary coords ({msg.point_on_face.face_barys.x}, {msg.point_on_face.face_barys.y}, {msg.point_on_face.face_barys.z}):\n{K}")

        

def main(args=None):
    rclpy.init(args=args)
    node = LocalStiffnessMatTester()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
