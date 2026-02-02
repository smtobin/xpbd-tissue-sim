#ifndef __VIRTUOSO_TISSUE_GRASPING_SIM_BRIDGE_HPP
#define __VIRTUOSO_TISSUE_GRASPING_SIM_BRIDGE_HPP

#include "sim_bridge/VirtuosoSimBridge.hpp"

#include "simulation/VirtuosoCTAnatomySimulation.hpp"

class VirtuosoCTAnatomySimBridge : public VirtuosoSimBridge
{
public:
    VirtuosoCTAnatomySimBridge(Sim::VirtuosoCTAnatomySimulation* sim);

private:

    void _setupPartialViewPointCloudPublishers();

    void _setupCTtoVBTransformListener();

    sensor_msgs::msg::PointCloud2 _trachea_partial_view_pc_message;
    sensor_msgs::msg::PointCloud2 _tumor_partial_view_pc_message;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _trachea_partial_view_pc_publisher;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _tumor_partial_view_pc_publisher;
};

#endif // __VIRTUOSO_TISSUE_GRASPING_SIM_BRIDGE_HPP