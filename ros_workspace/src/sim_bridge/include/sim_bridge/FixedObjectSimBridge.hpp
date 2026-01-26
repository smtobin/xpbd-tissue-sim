#ifndef __FIXED_CUBE_SIM_BRIDGE_HPP
#define __FIXED_CUBE_SIM_BRIDGE_HPP

#include "sim_bridge/SimBridge.hpp"

#include "simulation/FixedObjectSimulation.hpp"

class FixedObjectSimBridge : public SimBridge<Sim::FixedObjectSimulation>
{
public:
    FixedObjectSimBridge(Sim::FixedObjectSimulation* sim)
        : SimBridge<Sim::FixedObjectSimulation>(sim)
    {
        _setupPartialViewPointCloudPublisher();
    }

private:
    void _setupPartialViewPointCloudPublisher()
    {
        _partial_view_pc_publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>("/sim/output/partial_view_pc", 3);
        
        this->declare_parameter("partial_view_pc", true);
        this->declare_parameter("partial_view_pc_hfov", 80.0);
        this->declare_parameter("partial_view_pc_vfov", 30.0);
        this->declare_parameter("partial_view_pc_sample_density", 1.0);

        Real hfov_deg = this->get_parameter("partial_view_pc_hfov").as_double();
        Real vfov_deg = this->get_parameter("partial_view_pc_vfov").as_double();
        Real sample_density = this->get_parameter("partial_view_pc_sample_density").as_double();

        // lambda function to configure point cloud messages
        auto configure_pcl_message = [&](sensor_msgs::msg::PointCloud2& pcl_msg)
        {
            // set header
            pcl_msg.header.stamp = this->now();
            pcl_msg.header.frame_id = "world";

            // // add point fields
            pcl_msg.fields.resize(3);
            pcl_msg.fields[0].name = "x";
            pcl_msg.fields[0].offset = 0;
            pcl_msg.fields[0].datatype = (typeid(Real) == typeid(double)) ? sensor_msgs::msg::PointField::FLOAT64 : sensor_msgs::msg::PointField::FLOAT32;
            pcl_msg.fields[0].count = 1;

            pcl_msg.fields[1].name = "y";
            pcl_msg.fields[1].offset = sizeof(Real);
            pcl_msg.fields[1].datatype = (typeid(Real) == typeid(double)) ? sensor_msgs::msg::PointField::FLOAT64 : sensor_msgs::msg::PointField::FLOAT32;
            pcl_msg.fields[1].count = 1;

            pcl_msg.fields[2].name = "z";
            pcl_msg.fields[2].offset = 2*sizeof(Real);
            pcl_msg.fields[2].datatype = (typeid(Real) == typeid(double)) ? sensor_msgs::msg::PointField::FLOAT64 : sensor_msgs::msg::PointField::FLOAT32;
            pcl_msg.fields[2].count = 1;

            pcl_msg.height = 1;
            pcl_msg.width = hfov_deg * vfov_deg * sample_density * sample_density;
            pcl_msg.is_dense = true;
            pcl_msg.is_bigendian = false;

            pcl_msg.point_step = 3*sizeof(Real);
            pcl_msg.row_step = pcl_msg.point_step * pcl_msg.width;

            pcl_msg.data.resize(pcl_msg.row_step);
        };

        configure_pcl_message(_partial_view_pc_message);
        

        auto partial_view_pc_callback = 
            [this]() -> void {

                if (!this->get_parameter("partial_view_pc").as_bool())
                    return;

                const Geometry::CoordinateFrame& point_cloud_sample_frame = this->_sim->pointCloudSampleFrame();
                const Vec3r& sample_position = point_cloud_sample_frame.origin();
                const Vec3r& sample_view_dir = point_cloud_sample_frame.transform().rotMat().col(2);
                const Vec3r& sample_up_dir = point_cloud_sample_frame.transform().rotMat().col(1);

                Real hfov_deg = this->get_parameter("partial_view_pc_hfov").as_double();
                Real vfov_deg = this->get_parameter("partial_view_pc_vfov").as_double();
                Real sample_density = this->get_parameter("partial_view_pc_sample_density").as_double();

                this->_sim->updateEmbreeScene();
                std::vector<Vec3r> point_cloud = 
                    this->_sim->embreeScene()->partialViewPointCloud(sample_position, sample_view_dir, sample_up_dir, hfov_deg, vfov_deg, sample_density);

                this->_partial_view_pc_message.width = point_cloud.size();
                this->_partial_view_pc_message.row_step = this->_partial_view_pc_message.width * this->_partial_view_pc_message.point_step;
                this->_partial_view_pc_message.data.resize(this->_partial_view_pc_message.row_step);

                for (unsigned i = 0; i < point_cloud.size(); i++)
                {
                    memcpy((Real*)this->_partial_view_pc_message.data.data() + 3*i, point_cloud[i].data(), sizeof(Real)*3);
                }
                    

                this->_partial_view_pc_message.header.stamp = this->now();

                // publish the message
                this->_partial_view_pc_publisher->publish(this->_partial_view_pc_message);
            };
        
        _sim->addCallback(1.0/this->get_parameter("publish_rate_hz").as_double(), partial_view_pc_callback);
    }

private:

    sensor_msgs::msg::PointCloud2 _partial_view_pc_message;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _partial_view_pc_publisher;
};

#endif // __FIXED_CUBE_SIM_BRIDGE_HPP