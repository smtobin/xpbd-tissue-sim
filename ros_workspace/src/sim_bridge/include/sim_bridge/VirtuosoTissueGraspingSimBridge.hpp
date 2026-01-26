#ifndef __VIRTUOSO_TISSUE_GRASPING_SIM_BRIDGE_HPP
#define __VIRTUOSO_TISSUE_GRASPING_SIM_BRIDGE_HPP

#include "sim_bridge/VirtuosoSimBridge.hpp"

#include "simulation/VirtuosoTissueGraspingSimulation.hpp"

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

class VirtuosoTissueGraspingSimBridge : public VirtuosoSimBridge
{
public:
    VirtuosoTissueGraspingSimBridge(Sim::VirtuosoTissueGraspingSimulation* sim);

private:

    void _setupPartialViewPointCloudPublishers();

    void _setupCTtoVBTransformListener();

    sensor_msgs::msg::PointCloud2 _trachea_partial_view_pc_message;
    sensor_msgs::msg::PointCloud2 _tumor_partial_view_pc_message;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _trachea_partial_view_pc_publisher;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _tumor_partial_view_pc_publisher;

    std::shared_ptr<tf2_ros::Buffer> _tf_buffer;
    std::shared_ptr<tf2_ros::TransformListener> _tf_listener;
};

#endif // __VIRTUOSO_TISSUE_GRASPING_SIM_BRIDGE_HPP