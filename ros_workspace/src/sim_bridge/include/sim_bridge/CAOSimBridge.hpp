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
    void _setupTumorTracheaAttachmentForcePublisher();

    void _setupPartialViewPointCloudPublishers();
    void _setupRemovedElementsPublishers();

    void _setupSegmentationMaskPublishers();

    template <typename XPBDMeshObject_BaseType>
    void _setupRemovedElementsPublisherForMesh(int index, XPBDMeshObject_BaseType* xpbd_obj);

    void _setupToolTracheaCollisionPublisher();

    void _setupTumorDetachedPublisher();

    /** Publisher for the tumor-trachea attachment force */
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr _tumor_attachment_force_publisher;

    /** Publisher for tumor-trachea detachment */
    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr _tumor_detached_publisher;

    /** Reusable messages for the point clouds (so we don't reallocate memory every time we publish) */
    sensor_msgs::msg::PointCloud2 _trachea_partial_view_pc_message;
    sensor_msgs::msg::PointCloud2 _tumor_partial_view_pc_message;
    sensor_msgs::msg::PointCloud2 _tool_partial_view_pc_message;

    /** Publishers for the trachea and tumor point clouds */
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _trachea_partial_view_pc_publisher;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _tumor_partial_view_pc_publisher;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _tool_partial_view_pc_publisher;

    /** Publisher for tool-trachea collision */
    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr _arm1_trachea_collision_publisher;
    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr _arm2_trachea_collision_publisher;

    /** Publisher for removed element information */
    std::vector<rclcpp::Publisher<sim_bridge::msg::RemovedElementArray>::SharedPtr> _removed_elements_publishers;

    /** Publishers for segmentation mask images */
    std::vector<unsigned char> _flipped_image_buffer;
    sensor_msgs::msg::Image _seg_image_msg;
    sensor_msgs::msg::Image _arm1_seg_mask_msg;
    sensor_msgs::msg::Image _arm2_seg_mask_msg;
    sensor_msgs::msg::Image _tumor_seg_mask_msg;
    sensor_msgs::msg::Image _trachea_seg_mask_msg;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr _seg_image_publisher;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr _arm1_seg_mask_publisher;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr _arm2_seg_mask_publisher;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr _tumor_seg_mask_publisher;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr _trachea_seg_mask_publisher;
};

#endif // __CAO_SIM_BRIDGE_HPP