#ifndef __VIRTUOSO_SIM_BRIDGE_HPP
#define __VIRTUOSO_SIM_BRIDGE_HPP

#include "sim_bridge/SimBridge.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "std_msgs/msg/int8.hpp"
#include <tf2_ros/transform_broadcaster.h>

#include "simulation/VirtuosoSimulation.hpp"

class VirtuosoSimBridge : public SimBridge<Sim::VirtuosoSimulation>
{
    public:
    VirtuosoSimBridge(Sim::VirtuosoSimulation* sim);

    private:
    /** Parses a ROS JointState message with 5 fields
     * "inner_rotation" - inner tube rotation
     * "outer_rotation" - outer tube rotation
     * "inner_translation" - inner tube translation
     * "outer_translation" - outer tube translation
     * "tool" - tool actuation
     * 
     * into a tuple: (outer_rot, outer_trans, inner_rot, inner_trans, tool)
     */
    std::tuple<double, double, double, double, int> _jointMsgToJointState(sensor_msgs::msg::JointState* msg) const;
    
    sensor_msgs::msg::JointState _createJointStateMsgForArm(const Sim::VirtuosoArm* arm) const;

    geometry_msgs::msg::Pose _poseFromTransformationMatrix(const Geometry::TransformationMatrix& transform) const;

    void _setupTransformBroadcaster();

    void _setupPublishers();

    void _setupArmJointStatePublisher(const Sim::VirtuosoArm* arm, rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher);

    void _setupArmFramesPublisher(const Sim::VirtuosoArm* arm, rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr publisher);

    void _setupArmTipFramePublisher(const Sim::VirtuosoArm* arm, rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher);

    void _setupArmCommandedTipFramePublisher(const Sim::VirtuosoArm* arm, rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher);

    void _setupArmToolTipFramePublisher(const Sim::VirtuosoArm* arm, rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher);

    void _setupArmNetForcePublisher(const Sim::VirtuosoArm* arm, rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr publisher);

    void _setupSubscribers();

    private:
    /** Publishers */
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr _arm1_joint_state_publisher;   // publishes the current joint state of arm1
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr _arm2_joint_state_publisher;   // published the current joint state of arm2

    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr _arm1_frames_publisher;     // publishes coordinate frames along backbone of arm1
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr _arm2_frames_publisher;     // publishes coordinate frames along backbone of arm2

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr _arm1_tip_frame_publisher;          // publishes the tip frame of arm1
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr _arm2_tip_frame_publisher;          // publishes the tip frame of arm2

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr _arm1_commanded_tip_frame_publisher;  // publishes the commanded tip position of arm1
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr _arm2_commanded_tip_frame_publisher;  // published the commanded tip position of arm2

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr _arm1_tool_tip_frame_publisher;          // publishes the tool tip frame of arm1
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr _arm2_tool_tip_frame_publisher;          // publishes the tool tip frame of arm2

    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr _arm1_net_force_publisher;         // publishes the net force on arm1
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr _arm2_net_force_publisher;         // publishes the net force on amr2

    std::unique_ptr<tf2_ros::TransformBroadcaster> _tf_broadcaster;

    /** Subscriptions */
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr _arm1_joint_state_subscriber;     // subscribes to joint state commands for arm1
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr _arm2_joint_state_subscriber;     // subscribes to joint state commands for arm2
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr _arm1_tip_position_subscriber;     // subscribes to tip position commands for arm1
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr _arm2_tip_position_subscriber;     // subscribes to tip position commands for arm2
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr _arm1_tool_state_subscriber;              // subscribes to tool state commands for arm1
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr _arm2_tool_state_subscriber;              // subscribes to tool state commands for arm2
};

#endif // __VIRTUOSO_SIM_BRIDGE_HPP