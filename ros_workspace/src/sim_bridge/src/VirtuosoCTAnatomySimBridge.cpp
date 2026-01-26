#include "sim_bridge/VirtuosoCTAnatomySimBridge.hpp"

VirtuosoCTAnatomySimBridge::VirtuosoCTAnatomySimBridge(Sim::VirtuosoCTAnatomySimulation* sim)
    : VirtuosoSimBridge(sim)
{
    _setupPartialViewPointCloudPublishers();
    _setupCTtoVBTransformListener();

    std::cout << "VirtuosoCTAnatomySimBridge constructor" << std::endl;
}


void VirtuosoCTAnatomySimBridge::_setupPartialViewPointCloudPublishers()
{
    _trachea_partial_view_pc_publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>("/sim/output/trachea_partial_view_pc", 10);
    _tumor_partial_view_pc_publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>("/sim/output/tumor_partial_view_pc", 10);
    
    this->declare_parameter("partial_view_pc", true);
    this->declare_parameter("partial_view_pc_hfov", 80.0);
    this->declare_parameter("partial_view_pc_vfov", 30.0);
    this->declare_parameter("partial_view_pc_sample_density", 1.0);

    this->declare_parameter("trachea_label", "Trachea");
    this->declare_parameter("tumor_label", "Tumor");

    Real hfov_deg = this->get_parameter("partial_view_pc_hfov").as_double();
    Real vfov_deg = this->get_parameter("partial_view_pc_vfov").as_double();
    Real sample_density = this->get_parameter("partial_view_pc_sample_density").as_double();

    // lambda function to configure point cloud messages
    auto configure_pcl_message = [&](sensor_msgs::msg::PointCloud2& pcl_msg)
    {
        // set header
        pcl_msg.header.stamp = this->now();
        pcl_msg.header.frame_id = "ves/left/base";

        // // add point fields
        pcl_msg.fields.resize(3);
        pcl_msg.fields[0].name = "x";
        pcl_msg.fields[0].offset = 0;
        // pcl_msg.fields[0].datatype = (typeid(Real) == typeid(double)) ? sensor_msgs::msg::PointField::FLOAT64 : sensor_msgs::msg::PointField::FLOAT32;
        pcl_msg.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32; // always use floats
        pcl_msg.fields[0].count = 1;

        pcl_msg.fields[1].name = "y";
        pcl_msg.fields[1].offset = sizeof(float);
        // pcl_msg.fields[1].datatype = (typeid(Real) == typeid(double)) ? sensor_msgs::msg::PointField::FLOAT64 : sensor_msgs::msg::PointField::FLOAT32;
        pcl_msg.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32; // always use floats
        pcl_msg.fields[1].count = 1;

        pcl_msg.fields[2].name = "z";
        pcl_msg.fields[2].offset = 2*sizeof(float);
        // pcl_msg.fields[2].datatype = (typeid(Real) == typeid(double)) ? sensor_msgs::msg::PointField::FLOAT64 : sensor_msgs::msg::PointField::FLOAT32;
        pcl_msg.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32; // always use floats
        pcl_msg.fields[2].count = 1;

        pcl_msg.height = 1;
        pcl_msg.width = hfov_deg * vfov_deg * sample_density * sample_density;
        pcl_msg.is_dense = true;
        pcl_msg.is_bigendian = false;

        pcl_msg.point_step = 3*sizeof(float);
        pcl_msg.row_step = pcl_msg.point_step * pcl_msg.width;

        pcl_msg.data.resize(pcl_msg.row_step);
    };

    configure_pcl_message(_trachea_partial_view_pc_message);
    configure_pcl_message(_tumor_partial_view_pc_message);
    

    auto partial_view_pc_callback = 
        [this]() -> void {

            if (!this->get_parameter("partial_view_pc").as_bool())
                return;

            const Vec3r& cam_position = this->_sim->graphicsScene()->cameraPosition();
            const Vec3r& cam_view_dir = this->_sim->graphicsScene()->cameraViewDirection();
            const Vec3r& cam_up_dir = this->_sim->graphicsScene()->cameraUpDirection();

            Real hfov_deg = this->get_parameter("partial_view_pc_hfov").as_double();
            Real vfov_deg = this->get_parameter("partial_view_pc_vfov").as_double();
            Real sample_density = this->get_parameter("partial_view_pc_sample_density").as_double();

            const std::string trachea_label = this->get_parameter("trachea_label").as_string();
            const std::string tumor_label = this->get_parameter("tumor_label").as_string();

            this->_sim->updateEmbreeRayScene();
            std::vector<Geometry::PointsWithClass> point_clouds = 
                this->_sim->embreeScene()->partialViewPointCloudsWithClass(cam_position, cam_view_dir, cam_up_dir, hfov_deg, vfov_deg, sample_density);
            // go through returned point clouds and find the ones that match the trachea and tumor classes
            for (auto& pc : point_clouds)
            {
                // transform points to VB frame
                const Geometry::CoordinateFrame& vb_frame = this->_sim->virtuosoRobot()->VBFrame();
                const Geometry::TransformationMatrix vb_transform_inv = vb_frame.transform().inverse();
                for (unsigned i = 0; i < pc.points.size(); i++)
                {
                    pc.points[i] = vb_transform_inv.rotMat()*pc.points[i] + vb_transform_inv.translation();
                }

                if (pc.classification == trachea_label)
                {
                    this->_trachea_partial_view_pc_message.header.stamp = this->now();
                    this->_trachea_partial_view_pc_message.width = pc.points.size();
                    this->_trachea_partial_view_pc_message.row_step = this->_trachea_partial_view_pc_message.width * this->_trachea_partial_view_pc_message.point_step;
                    this->_trachea_partial_view_pc_message.data.resize(this->_trachea_partial_view_pc_message.row_step);

                    float* pc_data = (float*)this->_trachea_partial_view_pc_message.data.data();
                    for (unsigned i = 0; i < pc.points.size(); i++)
                    {
                        *(pc_data + 3*i) = static_cast<float>(pc.points[i][0]);
                        *(pc_data + 3*i+1) = static_cast<float>(pc.points[i][1]);
                        *(pc_data + 3*i+2) = static_cast<float>(pc.points[i][2]);
                    }
                }

                else if (pc.classification == tumor_label)
                {
                    this->_tumor_partial_view_pc_message.header.stamp = this->now();
                    this->_tumor_partial_view_pc_message.width = pc.points.size();
                    this->_tumor_partial_view_pc_message.row_step = this->_tumor_partial_view_pc_message.width * this->_tumor_partial_view_pc_message.point_step;
                    this->_tumor_partial_view_pc_message.data.resize(this->_tumor_partial_view_pc_message.row_step);

                    float* pc_data = (float*)this->_tumor_partial_view_pc_message.data.data();
                    for (unsigned i = 0; i < pc.points.size(); i++)
                    {
                        *(pc_data + 3*i) = static_cast<float>(pc.points[i][0]);
                        *(pc_data + 3*i+1) = static_cast<float>(pc.points[i][1]);
                        *(pc_data + 3*i+2) = static_cast<float>(pc.points[i][2]);
                    }
                }
            }

            this->_trachea_partial_view_pc_message.header.stamp = this->now();
            this->_tumor_partial_view_pc_message.header.stamp = this->now();

            // publish the messages
            this->_trachea_partial_view_pc_publisher->publish(this->_trachea_partial_view_pc_message);
            this->_tumor_partial_view_pc_publisher->publish(this->_tumor_partial_view_pc_message);
        };
    
    _sim->addCallback(1.0/this->get_parameter("publish_rate_hz").as_double(), partial_view_pc_callback);
}


void VirtuosoCTAnatomySimBridge::_setupCTtoVBTransformListener()
{
    _tf_buffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    _tf_listener = std::make_shared<tf2_ros::TransformListener>(*_tf_buffer);

    this->declare_parameter("CT_frame_name", "CT/kuka");

    auto ct_to_vb_callback = [this] () -> void {
        std::string source_frame = this->get_parameter("CT_frame_name").as_string();
        std::string target_frame = "ves/left/base";

        // get the latest transform (if it exists)
        if (this->_tf_buffer->canTransform(target_frame, source_frame, tf2::TimePointZero))
        {
            try
            {
                // try getting the transform
                auto transform_msg = this->_tf_buffer->lookupTransform(
                    target_frame, source_frame, tf2::TimePointZero);
                
                // create TransformationMatrix object from transform
                Vec3r origin(transform_msg.transform.translation.x, transform_msg.transform.translation.y, transform_msg.transform.translation.z);
                Vec4r quat(transform_msg.transform.rotation.x, transform_msg.transform.rotation.y, transform_msg.transform.rotation.z, transform_msg.transform.rotation.w);
                Mat3r rot_mat = GeometryUtils::quatToMat(quat);
                Geometry::TransformationMatrix ct_to_vb(rot_mat, origin);
                
                Sim::VirtuosoCTAnatomySimulation* vtg_sim = dynamic_cast<Sim::VirtuosoCTAnatomySimulation*>(this->_sim);
                assert(vtg_sim);

                vtg_sim->setCTtoVBTransform(ct_to_vb);
                    
            }
            catch (tf2::TransformException& ex)
            {
                RCLCPP_WARN(this->get_logger(), "Error getting transform: %s", ex.what());
            }
        }
        else
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "Transform from %s to %s does not exist yet", 
                source_frame.c_str(), target_frame.c_str());
        }

    };

    _sim->addCallback(0.5, ct_to_vb_callback);
}