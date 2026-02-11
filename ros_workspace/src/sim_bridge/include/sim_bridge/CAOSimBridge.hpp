#ifndef __CAO_SIM_BRIDGE_HPP
#define __CAO_SIM_BRIDGE_HPP

#include "sim_bridge/VirtuosoCTAnatomySimBridge.hpp"

#include "simulation/VirtuosoCTAnatomySimulation.hpp"

class CAOSimBridge : public VirtuosoCTAnatomySimBridge
{
public:
    CAOSimBridge(Sim::VirtuosoCTAnatomySimulation* sim);

private:

    void _setupPartialViewPointCloudPublishers();

    sensor_msgs::msg::PointCloud2 _trachea_partial_view_pc_message;
    sensor_msgs::msg::PointCloud2 _tumor_partial_view_pc_message;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _trachea_partial_view_pc_publisher;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _tumor_partial_view_pc_publisher;
};

#endif // __CAO_SIM_BRIDGE_HPP