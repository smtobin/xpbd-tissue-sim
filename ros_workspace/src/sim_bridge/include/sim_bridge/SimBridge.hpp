#ifndef __SIM_BRIDGE_HPP
#define __SIM_BRIDGE_HPP

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "geometry/Mesh.hpp"
#include "simobject/XPBDMeshObjectBase.hpp"
#include "simobject/XPBDMeshObjectBaseWrapper.hpp"

#include <chrono>
#include <thread>

template <typename SimulationType>
class SimBridge : public rclcpp::Node
{
    public:
    SimBridge(SimulationType* sim)
        : rclcpp::Node("sim_bridge"), _sim(sim)
    {
        this->declare_parameter("use_wall_time_for_publishing", true);
        this->declare_parameter("publish_rate_hz", 30.0);
        this->declare_parameter("publish_matrices", false);

        // assume that setup() has already been called on the Simulation object
        // then we can probe how many deformable objects are in the Sim
        const typename SimulationType::ObjectVectorType& sim_objects = sim->objects();
        const std::vector<std::unique_ptr<Sim::XPBDMeshObject_Base>>& xpbd_mesh_objs = sim_objects.template get<std::unique_ptr<Sim::XPBDMeshObject_Base>>();
        const std::vector<std::unique_ptr<Sim::FirstOrderXPBDMeshObject_Base>>& fo_xpbd_mesh_objs = 
            sim_objects.template get<std::unique_ptr<Sim::FirstOrderXPBDMeshObject_Base>>();
        
        unsigned num_xpbd_objs = xpbd_mesh_objs.size() + fo_xpbd_mesh_objs.size();

        // allocate space for messages and publishers (one for each XPBD mesh object in the sim)

        _mesh_pcl_messages.resize(num_xpbd_objs);
        _mesh_pcl_publishers.resize(num_xpbd_objs);

        
        if (this->get_parameter("publish_matrices").as_bool())
        {
            std::cout << "Resizing vectors... " << std::endl;
            _stiffness_mat_messages.resize(num_xpbd_objs);
            _stiffness_mat_publishers.resize(num_xpbd_objs);

            _vertices_mat_messages.resize(num_xpbd_objs);
            _vertices_mat_publishers.resize(num_xpbd_objs);

            _faces_mat_messages.resize(num_xpbd_objs);
            _faces_mat_publishers.resize(num_xpbd_objs);

            _elements_mat_messages.resize(num_xpbd_objs);
            _elements_mat_publishers.resize(num_xpbd_objs);
            std::cout << "Done." << std::endl;
        }

        // set up callbacks to publish mesh as Mesh msg and PCL point cloud msg
        int index = 0;
        sim_objects.template for_each_element<std::unique_ptr<Sim::XPBDMeshObject_Base>, std::unique_ptr<Sim::FirstOrderXPBDMeshObject_Base>>([&index, this](auto& obj){
            const Geometry::Mesh* deformable_mesh = obj->mesh();
            _setupDeformableMeshPclPublisher(index, deformable_mesh);
            
            if (this->get_parameter("publish_matrices").as_bool())
            {
                std::cout << "Setting up matrix publishers for index " << index << "..." << std::endl;
                _setupStiffnessMatrixPublisher(index, obj.get());
                _setupVerticesMatrixPublisher(index, obj.get());
                _setupFacesMatrixPublisher(index, obj.get());
                _setupElementsMatrixPublisher(index, obj.get());
                std::cout << "Done." << std::endl;
            }

            index++;
        });
    }

private:

    void _setupDeformableMeshPclPublisher(int index, const Geometry::Mesh* deformable_mesh)
    {
        std::string topic_name = "/output/mesh_vertices_pc_" + std::to_string(index);
        _mesh_pcl_publishers[index] = this->create_publisher<sensor_msgs::msg::PointCloud2>(topic_name, 3);

        // set header
        sensor_msgs::msg::PointCloud2& mesh_pcl_message = _mesh_pcl_messages[index];
        mesh_pcl_message.header.stamp = this->now();
        mesh_pcl_message.header.frame_id = "sim/world";

        // add point fields
        mesh_pcl_message.fields.resize(3);
        mesh_pcl_message.fields[0].name = "x";
        mesh_pcl_message.fields[0].offset = 0;
        // mesh_pcl_message.fields[0].datatype = (typeid(Real) == typeid(double)) ? sensor_msgs::msg::PointField::FLOAT64 : sensor_msgs::msg::PointField::FLOAT32;
        mesh_pcl_message.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
        mesh_pcl_message.fields[0].count = 1;

        mesh_pcl_message.fields[1].name = "y";
        mesh_pcl_message.fields[1].offset = sizeof(float);
        // mesh_pcl_message.fields[1].datatype = (typeid(Real) == typeid(double)) ? sensor_msgs::msg::PointField::FLOAT64 : sensor_msgs::msg::PointField::FLOAT32;
        mesh_pcl_message.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
        mesh_pcl_message.fields[1].count = 1;

        mesh_pcl_message.fields[2].name = "z";
        mesh_pcl_message.fields[2].offset = 2*sizeof(float);
        // mesh_pcl_message.fields[2].datatype = (typeid(Real) == typeid(double)) ? sensor_msgs::msg::PointField::FLOAT64 : sensor_msgs::msg::PointField::FLOAT32;
        mesh_pcl_message.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
        mesh_pcl_message.fields[2].count = 1;

        mesh_pcl_message.height = 1;
        mesh_pcl_message.width = deformable_mesh->numVertices();
        mesh_pcl_message.is_dense = true;
        mesh_pcl_message.is_bigendian = false;

        mesh_pcl_message.point_step = 3*sizeof(float);
        mesh_pcl_message.row_step = mesh_pcl_message.point_step * mesh_pcl_message.width;

        mesh_pcl_message.data.resize(mesh_pcl_message.row_step);

        auto mesh_pcl_callback = 
            [this, index, deformable_mesh]() -> void {
                // update vertices
                // memcpy(this->_mesh_pcl_messages[index].data.data(), deformable_mesh->vertices().data(), _mesh_pcl_messages[index].data.size());

                float* pc_data = (float*)this->_mesh_pcl_messages[index].data.data();
                for (int i = 0; i < deformable_mesh->numVertices(); i++)
                {
                    // memcpy((Real*)this->_tumor_partial_view_pc_message.data.data() + 3*i, pc.points[i].data(), sizeof(Real)*3);
                    const Vec3r& vertex = deformable_mesh->vertex(i);
                    *(pc_data + 3*i) = static_cast<float>(vertex[0]);
                    *(pc_data + 3*i+1) = static_cast<float>(vertex[1]);
                    *(pc_data + 3*i+2) = static_cast<float>(vertex[2]);
                }

                this->_mesh_pcl_publishers[index]->publish(this->_mesh_pcl_messages[index]);
            };
        
        _sim->addCallback(1.0/this->get_parameter("publish_rate_hz").as_double(), mesh_pcl_callback, this->get_parameter("use_wall_time_for_publishing").as_bool());
    }

    template <typename XPBDMeshObject_BaseType>
    void _setupStiffnessMatrixPublisher(int index, XPBDMeshObject_BaseType* xpbd_obj)
    {
        std::string topic_name = "/output/stiffness_mat_" + std::to_string(index);
        _stiffness_mat_publishers[index] = this->create_publisher<std_msgs::msg::Float64MultiArray>(topic_name, 3);

        std_msgs::msg::Float64MultiArray& mat_msg = _stiffness_mat_messages[index];
        mat_msg.layout.dim.resize(2);
        mat_msg.layout.dim[0].label = "rows";
        mat_msg.layout.dim[1].label = "cols";
        mat_msg.layout.data_offset = 0;
        
        auto mat_callback = 
            [this, index, xpbd_obj]() -> void {
                MatXr stiffness_mat = xpbd_obj->stiffnessMatrix();

                // make sure size is correct based on number of vertices
                _stiffness_mat_messages[index].layout.dim[0].size = stiffness_mat.rows();
                _stiffness_mat_messages[index].layout.dim[0].stride = stiffness_mat.size();
                _stiffness_mat_messages[index].layout.dim[1].size = stiffness_mat.cols();
                _stiffness_mat_messages[index].layout.dim[1].stride = stiffness_mat.rows();

                _stiffness_mat_messages[index].data.resize(stiffness_mat.size());
                // update vertices
                memcpy(this->_stiffness_mat_messages[index].data.data(), stiffness_mat.data(), this->_stiffness_mat_messages[index].data.size());

                this->_stiffness_mat_publishers[index]->publish(this->_stiffness_mat_messages[index]);
            };

        // add the callback, but specify to use the internal simulation time to determine when to publish, rather than wall clock time
        // i.e. publish every 10 time steps
        _sim->addCallback(1.0/this->get_parameter("publish_rate_hz").as_double(), mat_callback, this->get_parameter("use_wall_time_for_publishing").as_bool());
    }

    template <typename XPBDMeshObject_BaseType>
    void _setupVerticesMatrixPublisher(int index, XPBDMeshObject_BaseType* xpbd_obj)
    {
        std::string topic_name = "/output/vertices_mat_" + std::to_string(index);
        _vertices_mat_publishers[index] = this->create_publisher<std_msgs::msg::Float64MultiArray>(topic_name, 3);

        std_msgs::msg::Float64MultiArray& mat_msg = _vertices_mat_messages[index];
        mat_msg.layout.dim.resize(2);
        mat_msg.layout.dim[0].label = "vertices";
        mat_msg.layout.dim[1].label = "coordinates";
        mat_msg.layout.dim[1].size = 3;
        mat_msg.layout.data_offset = 0;
        
        auto mat_callback = 
            [this, index, xpbd_obj]() -> void {
                const Geometry::Mesh* mesh = xpbd_obj->mesh();

                // make sure size is correct based on number of vertices
                _vertices_mat_messages[index].layout.dim[0].size = mesh->numVertices();
                _vertices_mat_messages[index].layout.dim[0].stride = mesh->numVertices() * 3;
                _vertices_mat_messages[index].layout.dim[1].stride = mesh->numVertices();

                _vertices_mat_messages[index].data.resize(mesh->numVertices()*3);
                // update vertices
                for (int i = 0; i < mesh->numVertices(); i++)
                {
                    memcpy((Real*)this->_vertices_mat_messages[index].data.data() + 3*i, mesh->vertex(i).data(), sizeof(Real)*3);
                }

                this->_vertices_mat_publishers[index]->publish(this->_vertices_mat_messages[index]);
            };

        // add the callback, but specify to use the internal simulation time to determine when to publish, rather than wall clock time
        // i.e. publish every 10 time steps
        _sim->addCallback(1.0/this->get_parameter("publish_rate_hz").as_double(), mat_callback, this->get_parameter("use_wall_time_for_publishing").as_bool());

    }
    
    template <typename XPBDMeshObject_BaseType>
    void _setupFacesMatrixPublisher(int index, XPBDMeshObject_BaseType* xpbd_obj)
    {
        std::string topic_name = "/output/faces_mat_" + std::to_string(index);
        _faces_mat_publishers[index] = this->create_publisher<std_msgs::msg::Int32MultiArray>(topic_name, 3);

        std_msgs::msg::Int32MultiArray& mat_msg = _faces_mat_messages[index];
        mat_msg.layout.dim.resize(2);
        mat_msg.layout.dim[0].label = "faces";
        mat_msg.layout.dim[1].label = "vertices";
        mat_msg.layout.dim[1].size = 3;
        mat_msg.layout.data_offset = 0;
        
        auto mat_callback = 
            [this, index, xpbd_obj]() -> void {
                const Geometry::Mesh* mesh = xpbd_obj->mesh();

                // make sure size is correct based on number of vertices
                _faces_mat_messages[index].layout.dim[0].size = mesh->numFaces();
                _faces_mat_messages[index].layout.dim[0].stride = mesh->numFaces() * 3;
                _faces_mat_messages[index].layout.dim[1].stride = mesh->numFaces();

                _faces_mat_messages[index].data.resize(mesh->numFaces()*3);
                // update faces
                for (int i = 0; i < mesh->numFaces(); i++)
                {
                    memcpy((int*)this->_faces_mat_messages[index].data.data() + 3*i, mesh->face(i).data(), sizeof(int)*3);
                }

                this->_faces_mat_publishers[index]->publish(this->_faces_mat_messages[index]);
            };

        // add the callback, but specify to use the internal simulation time to determine when to publish, rather than wall clock time
        // i.e. publish every 10 time steps
        _sim->addCallback(1.0/this->get_parameter("publish_rate_hz").as_double(), mat_callback, this->get_parameter("use_wall_time_for_publishing").as_bool());
    }

    template <typename XPBDMeshObject_BaseType>
    void _setupElementsMatrixPublisher(int index, XPBDMeshObject_BaseType* xpbd_obj)
    {
        std::string topic_name = "/output/elements_mat_" + std::to_string(index);
        _elements_mat_publishers[index] = this->create_publisher<std_msgs::msg::Int32MultiArray>(topic_name, 3);

        std_msgs::msg::Int32MultiArray& mat_msg = _elements_mat_messages[index];
        mat_msg.layout.dim.resize(2);
        mat_msg.layout.dim[0].label = "elements";
        mat_msg.layout.dim[1].label = "vertices";
        mat_msg.layout.dim[1].size = 4;
        mat_msg.layout.data_offset = 0;
        
        auto mat_callback = 
            [this, index, xpbd_obj]() -> void {
                const Geometry::TetMesh* mesh = xpbd_obj->tetMesh();

                // make sure size is correct based on number of vertices
                _elements_mat_messages[index].layout.dim[0].size = mesh->numElements();
                _elements_mat_messages[index].layout.dim[0].stride = mesh->numElements() * 4;
                _elements_mat_messages[index].layout.dim[1].stride = mesh->numElements();

                _elements_mat_messages[index].data.resize(mesh->numElements()*4);
                // update faces
                for (int i = 0; i < mesh->numElements(); i++)
                {
                    memcpy((int*)this->_elements_mat_messages[index].data.data() + 4*i, mesh->element(i).data(), sizeof(int)*4);
                }

                this->_elements_mat_publishers[index]->publish(this->_elements_mat_messages[index]);
            };

        // add the callback, but specify to use the internal simulation time to determine when to publish, rather than wall clock time
        // i.e. publish every 10 time steps
        _sim->addCallback(1.0/this->get_parameter("publish_rate_hz").as_double(), mat_callback, this->get_parameter("use_wall_time_for_publishing").as_bool());
    }

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
    

protected:
    /** Publishers */
    std::vector<sensor_msgs::msg::PointCloud2> _mesh_pcl_messages;    // pre-allocated mesh point cloud ROS message for speed (assuming number of vertices stays the same)
    std::vector<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr> _mesh_pcl_publishers;    // publishes the current mesh vertices as a ROS point cloud (for easy ROS visualization)

    std::vector<std_msgs::msg::Float64MultiArray> _stiffness_mat_messages;
    std::vector<rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr> _stiffness_mat_publishers;

    std::vector<std_msgs::msg::Float64MultiArray> _vertices_mat_messages;
    std::vector<rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr> _vertices_mat_publishers;

    std::vector<std_msgs::msg::Int32MultiArray> _faces_mat_messages;
    std::vector<rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr> _faces_mat_publishers;

    std::vector<std_msgs::msg::Int32MultiArray> _elements_mat_messages;
    std::vector<rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr> _elements_mat_publishers;

    /** Pointer to the actively running Simulation object */
    SimulationType* _sim;
};

#endif // __SIM_BRIDGE_HPP