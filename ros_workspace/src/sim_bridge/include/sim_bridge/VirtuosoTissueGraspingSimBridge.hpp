#ifndef __VIRTUOSO_TISSUE_GRASPING_SIM_BRIDGE_HPP
#define __VIRTUOSO_TISSUE_GRASPING_SIM_BRIDGE_HPP

#include "sim_bridge/VirtuosoSimBridge.hpp"

#include "simulation/VirtuosoTissueGraspingSimulation.hpp"

class VirtuosoTissueGraspingSimBridge : public VirtuosoSimBridge
{
public:
    VirtuosoTissueGraspingSimBridge(Sim::VirtuosoTissueGraspingSimulation* sim);

private:

    void _setupPartialViewPointCloudPublishers();

    sensor_msgs::msg::PointCloud2 _trachea_partial_view_pc_message;
    sensor_msgs::msg::PointCloud2 _tumor_partial_view_pc_message;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _trachea_partial_view_pc_publisher;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _tumor_partial_view_pc_publisher;


};

#endif // __VIRTUOSO_TISSUE_GRASPING_SIM_BRIDGE_HPP