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
#include "sensor_msgs/msg/image.hpp"

#include "sim_bridge/msg/sparse_matrix.hpp"
#include "sim_bridge/msg/vertices_list.hpp"
#include "sim_bridge/msg/faces_list.hpp"
#include "sim_bridge/msg/elements_list.hpp"
#include "sim_bridge/msg/mesh_state.hpp"
#include "sim_bridge/msg/factor_graph_state.hpp"
#include <Eigen/Sparse>

#include "sim_bridge/srv/save_checkpoint.hpp"
#include "sim_bridge/srv/restore_checkpoint.hpp"
#include "sim_bridge/srv/factor_graph_state.hpp"

#include "geometry/Mesh.hpp"
#include "simobject/XPBDMeshObjectBase.hpp"
#include "simobject/XPBDMeshObjectBaseWrapper.hpp"

#include "graphics/vtk/VTKViewer.hpp"

#include <chrono>
#include <thread>
#include <variant>
#include <queue>

template <typename SimulationType>
class SimBridge : public rclcpp::Node
{
    public:
    SimBridge(SimulationType* sim)
        : rclcpp::Node("sim_bridge"), _sim(sim)
    {
        this->declare_parameter("use_wall_time_for_publishing", true);
        this->declare_parameter("publish_rate_hz", 30.0);
        this->declare_parameter("image_publish_rate_hz", 10.0);
        this->declare_parameter("publish_images", false);

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

        if (this->get_parameter("publish_images").as_bool())
        {
            _setupImagePublisher();
        }

        // set up callbacks to publish mesh as Mesh msg and PCL point cloud msg
        int index = 0;
        sim_objects.template for_each_element<std::unique_ptr<Sim::XPBDMeshObject_Base>, std::unique_ptr<Sim::FirstOrderXPBDMeshObject_Base>>([&index, this](auto& obj){
            // store the first deformable XPBD object that we come across
            // this will be the one whose information we publish for factor graph
            // for vast majority of simulations, there will only be one deformable mesh anyways
            if (index == 0)
            {
                _first_xpbd_obj = obj.get();
                _xpbd_mesh_at_last_fg_query = *(obj->refinedTetMesh());
            }
            
            const Geometry::Mesh* deformable_mesh = obj->mesh();
            _setupDeformableMeshPclPublisher(index, deformable_mesh);
            index++;
        });

        // set up checkpoint services
        _save_checkpoint_service = this->create_service<sim_bridge::srv::SaveCheckpoint>(
            "/sim/checkpoint/save",
            std::bind(&SimBridge::_saveCheckpoint, this, std::placeholders::_1, std::placeholders::_2)
        );

        _restore_checkpoint_service = this->create_service<sim_bridge::srv::RestoreCheckpoint>(
            "/sim/checkpoint/restore",
            std::bind(&SimBridge::_restoreCheckpoint, this, std::placeholders::_1, std::placeholders::_2)
        );

        // set up factor graph service
        _factor_graph_service = this->create_service<sim_bridge::srv::FactorGraphState>(
            "/sim/factor_graph_state",
            std::bind(&SimBridge::_factorGraphState, this, std::placeholders::_1, std::placeholders::_2) 
        );
    }

private:
    void _setupImagePublisher()
    {
        std::string topic_name  = "/sim/output/image";
        _image_publisher = this->create_publisher<sensor_msgs::msg::Image>(topic_name, 3);

        // make sure that VTK is being used as the graphics backend
        Graphics::VTKViewer* viewer = dynamic_cast<Graphics::VTKViewer*>(_sim->graphicsScene()->viewer());
        if (!viewer)
        {
            std::cerr << KRED << BOLD << "FATAL: " << RST << KRED << "VTK graphics must be used to publish images over ROS2." << RST << std::endl;
            assert(0);
            return;
        }

        // configure initial image
        _image_msg.header.frame_id = "sim/camera";
        _image_msg.height = viewer->height();
        _image_msg.width = viewer->width();
        _image_msg.encoding = "rgb8";
        _image_msg.is_bigendian = false;
        _image_msg.step = _image_msg.width * 3;

        size_t num_bytes = _image_msg.width * _image_msg.height * 3;
        _image_msg.data.resize(num_bytes, 0);

        // create the flipped buffer
        _flipped_image_buffer.resize(num_bytes, 0);

        auto image_callback = [this, viewer]() -> void {
            this->_image_msg.height = viewer->height();
            this->_image_msg.width = viewer->width();

            // resize fliplped image buffer and image messge data based on the current size of the viewer
            size_t num_bytes = this->_image_msg.height * this->_image_msg.width * 3;
            this->_flipped_image_buffer.resize(num_bytes);
            this->_image_msg.data.resize(num_bytes);

            // copy VTK image into flipped buffer (VTK uses a different coordinate system where bottom-left is (0,0), so the image is flipped vertically)
            viewer->copyImageBufferToExternalBuffer(this->_flipped_image_buffer.data());

            // copy row by row, flipping vertically
            size_t row_bytes = this->_image_msg.width * 3;
            for (unsigned y = 0; y < this->_image_msg.height; ++y) {
                unsigned char* src_row = this->_flipped_image_buffer.data() + y * row_bytes;
                unsigned char* dst_row = this->_image_msg.data.data() + (this->_image_msg.height - 1 - y) * row_bytes;
                std::memcpy(dst_row, src_row, row_bytes);
            }

            this->_image_publisher->publish(this->_image_msg);
        };

        _sim->addCallback(1.0/this->get_parameter("image_publish_rate_hz").as_double(), image_callback, this->get_parameter("use_wall_time_for_publishing").as_bool());
    }

    void _setupDeformableMeshPclPublisher(int index, const Geometry::Mesh* deformable_mesh)
    {
        std::string topic_name = "/sim/output/mesh_vertices_pc_" + std::to_string(index);
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
                this->_mesh_pcl_messages[index].header.stamp = this->now();
                this->_mesh_pcl_messages[index].width = deformable_mesh->numVertices();
                this->_mesh_pcl_messages[index].row_step = this->_mesh_pcl_messages[index].width * this->_mesh_pcl_messages[index].point_step;
                this->_mesh_pcl_messages[index].data.resize(this->_mesh_pcl_messages[index].row_step);

                float* pc_data = (float*)this->_mesh_pcl_messages[index].data.data();

                int pc_index = 0;
                for (const auto& v : deformable_mesh->vertices())
                {
                    *(pc_data + 3*pc_index) = static_cast<float>(v[0]);
                    *(pc_data + 3*pc_index+1) = static_cast<float>(v[1]);
                    *(pc_data + 3*pc_index+2) = static_cast<float>(v[2]);

                    pc_index++;
                }

                this->_mesh_pcl_publishers[index]->publish(this->_mesh_pcl_messages[index]);
            };
        
        _sim->addCallback(1.0/this->get_parameter("publish_rate_hz").as_double(), mesh_pcl_callback, this->get_parameter("use_wall_time_for_publishing").as_bool());
    }

    template <typename XPBDMeshObject_BaseType>
    void _setupStiffnessMatrixPublisher(int index, XPBDMeshObject_BaseType* xpbd_obj)
    {
        std::string topic_name = "/sim/output/stiffness_mat_" + std::to_string(index);
        _stiffness_mat_publishers[index] = this->create_publisher<sim_bridge::msg::SparseMatrix>(topic_name, 3);
        
        auto mat_callback = 
            [this, index, xpbd_obj]() -> void {
                sim_bridge::msg::SparseMatrix msg;
                msg.header.stamp = this->now();
                msg.header.frame_id = "sim/world";
                
                Eigen::SparseMatrix<Real> stiffness_mat = xpbd_obj->stiffnessMatrix();
                msg.rows = stiffness_mat.rows();
                msg.cols = stiffness_mat.cols();
                msg.row_indices.reserve(stiffness_mat.nonZeros());
                msg.col_indices.reserve(stiffness_mat.nonZeros());
                msg.values.reserve(stiffness_mat.nonZeros());
                for (int k = 0; k < stiffness_mat.outerSize(); ++k) 
                {
                    for (Eigen::SparseMatrix<Real>::InnerIterator it(stiffness_mat, k); it; ++it) 
                    {
                        msg.row_indices.push_back(it.row());
                        msg.col_indices.push_back(it.col());
                        msg.values.push_back(it.value());
                    }
                }

                this->_stiffness_mat_publishers[index]->publish(msg);
            };

        // add the callback, but specify to use the internal simulation time to determine when to publish, rather than wall clock time
        // i.e. publish every 10 time steps
        _sim->addCallback(1.0/this->get_parameter("publish_rate_hz").as_double(), mat_callback, this->get_parameter("use_wall_time_for_publishing").as_bool());
    }

    template <typename XPBDMeshObject_BaseType>
    void _setupVerticesMatrixPublisher(int index, XPBDMeshObject_BaseType* xpbd_obj)
    {
        std::string topic_name = "/sim/output/vertices_mat_" + std::to_string(index);
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
        std::string topic_name = "/sim/output/faces_mat_" + std::to_string(index);
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
        std::string topic_name = "/sim/output/elements_mat_" + std::to_string(index);
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
    
    void _saveCheckpoint(const std::shared_ptr<sim_bridge::srv::SaveCheckpoint::Request> req, std::shared_ptr<sim_bridge::srv::SaveCheckpoint::Response> res)
    {
        // instruct the sim to save the current state as a checkpoint
        bool success = _sim->saveCheckpoint(req->id);
        res->success = success;
        if (!success)
            res->message = "Failed to serialize state. Checkpoint with ID already exists.";
        
    }

    void _restoreCheckpoint(const std::shared_ptr<sim_bridge::srv::RestoreCheckpoint::Request> req, std::shared_ptr<sim_bridge::srv::RestoreCheckpoint::Response> res)
    {
        // restore the simulation checkpoint associated with the checkpoint ID
        bool success = _sim->restoreCheckpoint(req->id);
        res->success = success;
        if (!success)
            res->message = "Failed to restore state. Checkpoint with ID not found.";
        
    }

    void _factorGraphState(const std::shared_ptr<sim_bridge::srv::FactorGraphState::Request> req, std::shared_ptr<sim_bridge::srv::FactorGraphState::Response> res)
    {
        // assemble vertices, faces, and elements
        // block until sim is ready to give these
        auto future = _sim->addOneTimeCallbackBlocking([&req, &res, this]() {
            std::visit([&](auto& obj) {
                sim_bridge::msg::FactorGraphState& fg_msg = res->fg_state;
                
                Geometry::RefinedTetMesh* mesh = obj->refinedTetMesh();

                // universal header
                // everything in sim/world frame
                std_msgs::msg::Header header;
                header.stamp = this->now();
                header.frame_id = "sim/world";

                fg_msg.header = header;
                fg_msg.sim_mesh.header = header;

                _copyMeshToMeshStateMsg(fg_msg.sim_mesh, header, mesh);

                // compute stiffness matrix
                if (req->compute_stiffness_matrix)
                {
                    Eigen::SparseMatrix<Real> stiffness_mat = obj->stiffnessMatrix();
                    fg_msg.sim_stiffness_mat.header = header;
                    fg_msg.sim_stiffness_mat.rows = stiffness_mat.rows();
                    fg_msg.sim_stiffness_mat.cols = stiffness_mat.cols();
                    std::cout << "Stiffness matrix: " << stiffness_mat.rows() << " x " << stiffness_mat.cols() << " with " << stiffness_mat.nonZeros() << " nonzero entries." << std::endl;
                    fg_msg.sim_stiffness_mat.row_indices.reserve(stiffness_mat.nonZeros());
                    fg_msg.sim_stiffness_mat.col_indices.reserve(stiffness_mat.nonZeros());
                    fg_msg.sim_stiffness_mat.values.reserve(stiffness_mat.nonZeros());
                    for (int k = 0; k < stiffness_mat.outerSize(); ++k) 
                    {
                        for (Eigen::SparseMatrix<Real>::InnerIterator it(stiffness_mat, k); it; ++it) 
                        {
                            fg_msg.sim_stiffness_mat.row_indices.push_back(it.row());
                            fg_msg.sim_stiffness_mat.col_indices.push_back(it.col());
                            fg_msg.sim_stiffness_mat.values.push_back(it.value());
                        }
                    }
                }

                // if desired, update the last mesh to with the same topological operations so that it has the same topology as the current mesh
                if (req->update_last_mesh)
                {
                    // update the vertices of the last mesh according to the last factor graph state
                    // std::cout << "Updating vertices..." << std::endl;
                    for (const auto& ind : this->_xpbd_mesh_at_last_fg_query.vertices().validIndices())
                    {
                        Vec3r v = Eigen::Map<Vec3r>(req->last_mesh.vertices.data.data() + 3*ind);
                        this->_xpbd_mesh_at_last_fg_query.setVertex(ind, v);
                    }
                    // std::cout << "Done." << std::endl;

                    // apply topological operations
                    for (const auto& operation : mesh->topologicalOperationCache())
                    {
                        // std::cout << "Applying operation ";
                        // if (operation.operation == Geometry::RefinedTetMesh::TopologicalOperation::Type::REFINE) std::cout << "Refine";
                        // if (operation.operation == Geometry::RefinedTetMesh::TopologicalOperation::Type::COARSEN) std::cout << "Coarsen";
                        // if (operation.operation == Geometry::RefinedTetMesh::TopologicalOperation::Type::REMOVE) std::cout << "Remove";
                        // std::cout << "(" << operation.element_index << ", " << operation.level << ", " << operation.absolute << ")..." << std::endl;
                        operation.applyOperation(this->_xpbd_mesh_at_last_fg_query);
                        // std::cout << "Done." << std::endl;
                    }

                    // copy new mesh to msg
                    _copyMeshToMeshStateMsg(res->updated_last_mesh, header, &this->_xpbd_mesh_at_last_fg_query);
                }
                // regardless, clear the operations cache
                mesh->clearTopologicalOperationCache();
                this->_xpbd_mesh_at_last_fg_query = *mesh;

            }, this->_first_xpbd_obj);
            
        });

        future.wait();
    }

    void _copyMeshToMeshStateMsg(sim_bridge::msg::MeshState& msg, const std_msgs::msg::Header& header, const Geometry::TetMesh* mesh) const
    {
        // set up vertices
        sim_bridge::msg::VerticesList& vertices = msg.vertices;
        vertices.header = header;
        vertices.size = mesh->numVertices();
        // copy over valid vertices
        vertices.data.resize(3*mesh->vertices().totalSize());
        vertices.invalid_indices.reserve(mesh->vertices().totalSize() - mesh->numVertices());
        for (unsigned i = 0; i < mesh->vertices().totalSize(); i++)
        {
            if (mesh->vertexValid(i))
            {
                Vec3r v = mesh->vertex(i);
                vertices.data[3*i] = v[0];
                vertices.data[3*i+1] = v[1];
                vertices.data[3*i+2] = v[2];
            }
            else
            {
                vertices.invalid_indices.push_back(i);
            }
        }

        // set up faces
        sim_bridge::msg::FacesList& faces = msg.faces;
        faces.header = header;
        faces.size = mesh->numFaces();
        // copy over the valid faces
        faces.data.resize(3*mesh->faces().totalSize());
        faces.invalid_indices.reserve(mesh->faces().totalSize() - mesh->numFaces());
        for (unsigned i = 0; i < mesh->faces().totalSize(); i++)
        {
            if (mesh->faceValid(i))
            {
                Vec3i f = mesh->face(i);
                faces.data[3*i] = f[0];
                faces.data[3*i+1] = f[1];
                faces.data[3*i+2] = f[2];
            }
            else
            {
                faces.invalid_indices.push_back(i);
            }
        }

        // set up elements
        sim_bridge::msg::ElementsList& elements = msg.elements;
        elements.header = header;
        elements.size = mesh->numElements();
        elements.data.resize(4*mesh->elements().totalSize());
        elements.invalid_indices.reserve(mesh->elements().totalSize() - mesh->numElements());
        // copy over the valid elements
        for (unsigned i = 0; i < mesh->elements().totalSize(); i++)
        {
            if (mesh->elementValid(i))
            {
                Vec4i e = mesh->element(i);
                elements.data[4*i] = e[0];
                elements.data[4*i+1] = e[1];
                elements.data[4*i+2] = e[2];
                elements.data[4*i+3] = e[3];
            }
            else
            {
                elements.invalid_indices.push_back(i);
            }
        }
    }

protected:
    /** Publishers */
    std::vector<unsigned char> _flipped_image_buffer;
    sensor_msgs::msg::Image _image_msg;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr _image_publisher;

    std::vector<sensor_msgs::msg::PointCloud2> _mesh_pcl_messages;    // pre-allocated mesh point cloud ROS message for speed (assuming number of vertices stays the same)
    std::vector<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr> _mesh_pcl_publishers;    // publishes the current mesh vertices as a ROS point cloud (for easy ROS visualization)

    std::vector<sim_bridge::msg::SparseMatrix> _stiffness_mat_messages;
    std::vector<rclcpp::Publisher<sim_bridge::msg::SparseMatrix>::SharedPtr> _stiffness_mat_publishers;

    std::vector<std_msgs::msg::Float64MultiArray> _vertices_mat_messages;
    std::vector<rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr> _vertices_mat_publishers;

    std::vector<std_msgs::msg::Int32MultiArray> _faces_mat_messages;
    std::vector<rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr> _faces_mat_publishers;

    std::vector<std_msgs::msg::Int32MultiArray> _elements_mat_messages;
    std::vector<rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr> _elements_mat_publishers;

    /** Save and restore checkpoint services */
    rclcpp::Service<sim_bridge::srv::SaveCheckpoint>::SharedPtr _save_checkpoint_service;
    rclcpp::Service<sim_bridge::srv::RestoreCheckpoint>::SharedPtr _restore_checkpoint_service;

    /** Factor graph service */
    std::variant<Sim::XPBDMeshObject_Base*, Sim::FirstOrderXPBDMeshObject_Base*> _first_xpbd_obj;
    Geometry::RefinedTetMesh _xpbd_mesh_at_last_fg_query;  // copy of the mesh of the xpbd obj when the service was last queried
    rclcpp::Service<sim_bridge::srv::FactorGraphState>::SharedPtr _factor_graph_service;

    /** Pointer to the actively running Simulation object */
    SimulationType* _sim;
};

#endif // __SIM_BRIDGE_HPP