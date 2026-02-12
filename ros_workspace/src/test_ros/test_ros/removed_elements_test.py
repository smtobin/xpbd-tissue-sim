import rclpy
from rclpy.node import Node
from sim_bridge.msg import RemovedElementArray, RemovedElement
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
import math

class RemovedElementsVisualizer(Node):
    def __init__(self):
        super().__init__('removed_elements_subscriber')
        self.removed_elements_subscription = self.create_subscription(
            RemovedElementArray,
            '/sim/output/removed_elements_0',
            self.removed_elements_callback,
            10)
        
        self.timer = self.create_timer(0.1, self.publish_markers)

        self.marker_pub = self.create_publisher(
            MarkerArray,
            'removed_elements_markers',
            10
        )
        
        self.all_removed_elements = []
        self.latest_header = None

    def removed_elements_callback(self, msg):
        self.all_removed_elements.extend(msg.elements)
        self.latest_header = msg.header
        self.get_logger().info(f'{len(msg.elements)} removed elements received!\nTotal accumulated removed elements: {len(self.all_removed_elements)}')

    def publish_markers(self):
        if not self.all_removed_elements or self.latest_header is None:
            return
        
        marker_array = MarkerArray()
        for idx, element in enumerate(self.all_removed_elements):
            # Calculate sphere radius from rest volume
            # Volume of sphere = (4/3) * pi * r^3
            # So r = cbrt(3*V / (4*pi))
            radius = (3.0 * element.rest_volume / (4.0 * math.pi)) ** (1.0/3.0)
            
            # You might want to scale this for visibility
            radius *= 0.5  # Scale factor for visualization
            
            # Marker for CURRENT centroid
            current_marker = Marker()
            current_marker.header = self.latest_header
            current_marker.ns = "removed_elements_current"
            current_marker.id = idx
            current_marker.type = Marker.SPHERE
            current_marker.action = Marker.ADD
            
            current_marker.pose.position.x = element.centroid.x
            current_marker.pose.position.y = element.centroid.y
            current_marker.pose.position.z = element.centroid.z
            current_marker.pose.orientation.w = 1.0
            
            current_marker.scale.x = radius * 2.0  # Diameter
            current_marker.scale.y = radius * 2.0
            current_marker.scale.z = radius * 2.0
            
            # Red color for current position
            current_marker.color.r = 1.0
            current_marker.color.g = 0.0
            current_marker.color.b = 0.0
            current_marker.color.a = 0.7  # Semi-transparent
            
            marker_array.markers.append(current_marker)
            
            # Marker for INITIAL centroid
            initial_marker = Marker()
            initial_marker.header = self.latest_header
            initial_marker.ns = "removed_elements_initial"
            initial_marker.id = idx
            initial_marker.type = Marker.SPHERE
            initial_marker.action = Marker.ADD
            
            initial_marker.pose.position.x = element.initial_centroid.x
            initial_marker.pose.position.y = element.initial_centroid.y
            initial_marker.pose.position.z = element.initial_centroid.z
            initial_marker.pose.orientation.w = 1.0
            
            initial_marker.scale.x = radius * 2.0
            initial_marker.scale.y = radius * 2.0
            initial_marker.scale.z = radius * 2.0
            
            # Blue color for initial position
            initial_marker.color.r = 0.0
            initial_marker.color.g = 0.0
            initial_marker.color.b = 1.0
            initial_marker.color.a = 0.5  # More transparent
            
            marker_array.markers.append(initial_marker)

            # Add a line from initial to current centroid
            line_marker = Marker()
            line_marker.header = self.latest_header
            line_marker.ns = "removed_elements_trajectory"
            line_marker.id = idx
            line_marker.type = Marker.LINE_STRIP
            line_marker.action = Marker.ADD
            
            # Add two points: initial and current
            p1 = Point()
            p1.x, p1.y, p1.z = element.initial_centroid.x, element.initial_centroid.y, element.initial_centroid.z
            p2 = Point()
            p2.x, p2.y, p2.z = element.centroid.x, element.centroid.y, element.centroid.z
            
            line_marker.points = [p1, p2]
            
            line_marker.scale.x = 0.01  # Line width
            line_marker.color.r = 1.0
            line_marker.color.g = 1.0
            line_marker.color.b = 0.0
            line_marker.color.a = 0.5
            
            # self.all_removed_element_markers.markers.append(line_marker)
        
        self.marker_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = RemovedElementsVisualizer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()