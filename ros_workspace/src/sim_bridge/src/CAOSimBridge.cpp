#include "sim_bridge/CAOSimBridge.hpp"

CAOSimBridge::CAOSimBridge(Sim::VirtuosoCTAnatomySimulation* sim)
    : VirtuosoCTAnatomySimBridge(sim)
{
    _setupPartialViewPointCloudPublishers();
    _setupRemovedElementsPublishers();
    _setupToolTracheaCollisionPublisher();
}

void CAOSimBridge::_setupPartialViewPointCloudPublishers()
{
    _trachea_partial_view_pc_publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>("/sim/output/trachea_partial_view_pc", 10);
    _tumor_partial_view_pc_publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>("/sim/output/tumor_partial_view_pc", 10);
    _tool_partial_view_pc_publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>("/sin/output/tool_partial_view_pc", 10);
    
    this->declare_parameter("partial_view_pc", true);
    this->declare_parameter("partial_view_pc_hfov", 80.0);
    this->declare_parameter("partial_view_pc_vfov", 30.0);
    this->declare_parameter("partial_view_pc_sample_density", 1.0);

    this->declare_parameter("trachea_label", "Trachea");
    this->declare_parameter("tumor_label", "Tumor");
    this->declare_parameter("tool_label", "Tool");

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
    configure_pcl_message(_tool_partial_view_pc_message);
    

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
            const std::string tool_label = this->get_parameter("tool_label").as_string();

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

                else if (pc.classification == tool_label)
                {
                    this->_tool_partial_view_pc_message.header.stamp = this->now();
                    this->_tool_partial_view_pc_message.width = pc.points.size();
                    this->_tool_partial_view_pc_message.row_step = this->_tool_partial_view_pc_message.width * this->_tool_partial_view_pc_message.point_step;
                    this->_tool_partial_view_pc_message.data.resize(this->_tool_partial_view_pc_message.row_step);

                    float* pc_data = (float*)this->_tool_partial_view_pc_message.data.data();
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
            this->_tool_partial_view_pc_message.header.stamp = this->now();

            // publish the messages
            this->_trachea_partial_view_pc_publisher->publish(this->_trachea_partial_view_pc_message);
            this->_tumor_partial_view_pc_publisher->publish(this->_tumor_partial_view_pc_message);
            this->_tool_partial_view_pc_publisher->publish(this->_tool_partial_view_pc_message);
        };
    
    _sim->addCallback(1.0/this->get_parameter("publish_rate_hz").as_double(), partial_view_pc_callback);
}

void CAOSimBridge::_setupRemovedElementsPublishers()
{
    // assume that setup() has already been called on the Simulation object
    // then we can probe how many deformable objects are in the Sim
    const typename Sim::VirtuosoCTAnatomySimulation::ObjectVectorType& sim_objects = _sim->objects();
    const std::vector<std::unique_ptr<Sim::XPBDMeshObject_Base>>& xpbd_mesh_objs = sim_objects.template get<std::unique_ptr<Sim::XPBDMeshObject_Base>>();
    const std::vector<std::unique_ptr<Sim::FirstOrderXPBDMeshObject_Base>>& fo_xpbd_mesh_objs = 
        sim_objects.template get<std::unique_ptr<Sim::FirstOrderXPBDMeshObject_Base>>();
    
    unsigned num_xpbd_objs = xpbd_mesh_objs.size() + fo_xpbd_mesh_objs.size();
    _removed_elements_publishers.resize(num_xpbd_objs);

    int index = 0;
    sim_objects.template for_each_element<std::unique_ptr<Sim::XPBDMeshObject_Base>, std::unique_ptr<Sim::FirstOrderXPBDMeshObject_Base>>([&index, this](auto& obj){
        _setupRemovedElementsPublisherForMesh(index, obj.get());

        index++;
    });
}

template <typename XPBDMeshObject_BaseType>
void CAOSimBridge::_setupRemovedElementsPublisherForMesh(int index, XPBDMeshObject_BaseType* xpbd_obj)
{
    std::string topic_name = "/sim/output/removed_elements_" + std::to_string(index);
    _removed_elements_publishers[index] = this->create_publisher<sim_bridge::msg::RemovedElementArray>(topic_name, 3);
    
    auto callback = 
        [this, index, xpbd_obj]() -> void {
            std::vector<Geometry::TetMesh::RemovedElement> recently_removed_elements = xpbd_obj->tetMesh()->recentlyRemovedElements(true);
            if (recently_removed_elements.empty())
                return;
            

            sim_bridge::msg::RemovedElementArray array_msg;
            array_msg.header.stamp = this->now();
            array_msg.header.frame_id = "sim/world";
            array_msg.elements.reserve(recently_removed_elements.size());
            for (const auto& removed_element : recently_removed_elements)
            {
                sim_bridge::msg::RemovedElement msg;
                msg.header = array_msg.header;
                msg.centroid.x = removed_element.current_centroid[0];
                msg.centroid.y = removed_element.current_centroid[1];
                msg.centroid.z = removed_element.current_centroid[2];
                msg.initial_centroid.x = removed_element.rest_centroid[0];
                msg.initial_centroid.y = removed_element.rest_centroid[1];
                msg.initial_centroid.z = removed_element.rest_centroid[2];
                msg.rest_volume = static_cast<float>(removed_element.rest_volume);

                array_msg.elements.push_back(msg);
            }

            std::cout << "Publishing " << recently_removed_elements.size() << " removed elements..." << std::endl;
            this->_removed_elements_publishers[index]->publish(array_msg);
        };

    // add the callback, but specify to use the internal simulation time to determine when to publish, rather than wall clock time
    // i.e. publish every 10 time steps
    _sim->addCallback(1.0/this->get_parameter("publish_rate_hz").as_double(), callback, this->get_parameter("use_wall_time_for_publishing").as_bool());
}

void CAOSimBridge::_setupToolTracheaCollisionPublisher()
{
    _arm1_trachea_collision_publisher = this->create_publisher<std_msgs::msg::Int8>("/sim/output/arm1_trachea_collision", 10);
    _arm2_trachea_collision_publisher = this->create_publisher<std_msgs::msg::Int8>("/sim/output/arm2_trachea_collision", 10);

    auto arm1_callback =
        [this]() -> void {
            std_msgs::msg::Int8 msg;
            if (this->_sim->virtuosoRobot()->hasArm1() && !this->_sim->virtuosoRobot()->arm1()->rigidCollisions().empty())
                msg.data = 1;
            else
                msg.data = 0;
            
            this->_arm1_trachea_collision_publisher->publish(msg);
        };
    auto arm2_callback =
        [this]() -> void {
            std_msgs::msg::Int8 msg;
            if (this->_sim->virtuosoRobot()->hasArm2() && !this->_sim->virtuosoRobot()->arm2()->rigidCollisions().empty())
                msg.data = 1;
            else
                msg.data = 0;
            
            this->_arm2_trachea_collision_publisher->publish(msg);
        };
    _sim->addCallback(1.0/this->get_parameter("publish_rate_hz").as_double(), arm1_callback, this->get_parameter("use_wall_time_for_publishing").as_bool());
    _sim->addCallback(1.0/this->get_parameter("publish_rate_hz").as_double(), arm2_callback, this->get_parameter("use_wall_time_for_publishing").as_bool());
}