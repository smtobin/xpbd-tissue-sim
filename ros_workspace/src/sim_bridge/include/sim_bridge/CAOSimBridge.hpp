#ifndef __CAO_SIM_BRIDGE_HPP
#define __CAO_SIM_BRIDGE_HPP

#include "sim_bridge/VirtuosoCTAnatomySimBridge.hpp"

#include "simulation/VirtuosoCTAnatomySimulation.hpp"

#include "sim_bridge/msg/removed_element.hpp"
#include "sim_bridge/msg/removed_element_array.hpp"

class CAOSimBridge : public VirtuosoCTAnatomySimBridge
{
public:
    CAOSimBridge(Sim::VirtuosoCTAnatomySimulation* sim);

private:

    void _setupPartialViewPointCloudPublishers();
    void _setupRemovedElementsPublishers();

    template <typename XPBDMeshObject_BaseType>
    void _setupRemovedElementsPublisherForMesh(int index, XPBDMeshObject_BaseType* xpbd_obj);

    /** Reusable messages for the point clouds (so we don't reallocate memory every time we publish) */
    sensor_msgs::msg::PointCloud2 _trachea_partial_view_pc_message;
    sensor_msgs::msg::PointCloud2 _tumor_partial_view_pc_message;
    sensor_msgs::msg::PointCloud2 _tool_partial_view_pc_message;

    /** Publishers for the trachea and tumor point clouds */
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _trachea_partial_view_pc_publisher;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _tumor_partial_view_pc_publisher;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _tool_partial_view_pc_publisher;

    /** Publisher for removed element information */
    std::vector<rclcpp::Publisher<sim_bridge::msg::RemovedElementArray>::SharedPtr> _removed_elements_publishers;
};

#endif // __CAO_SIM_BRIDGE_HPP