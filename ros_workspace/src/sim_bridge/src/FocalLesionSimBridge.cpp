#include "sim_bridge/FocalLesionSimBridge.hpp"

FocalLesionSimBridge::FocalLesionSimBridge(Sim::VirtuosoCTAnatomySimulation* sim)
    : BPHSimBridge(sim)
{
    // set up factor graph service
    _focal_lesion_factor_graph_service = this->create_service<sim_bridge::srv::FocalLesionFactorGraphState>(
        "/sim/focal_lesion_factor_graph_state",
        std::bind(&FocalLesionSimBridge::_focalLesionFactorGraphState, this, std::placeholders::_1, std::placeholders::_2) 
    );

    // get the element class the corresponds to the lesion
    std::visit([&] (auto& obj_ptr) {
        const auto& material_classes = obj_ptr->materialClasses();
        std::string lesion_material = "Lesion";
        for (unsigned i = 0; i < material_classes.size(); i++)
        {
            if (material_classes[i]->name() == lesion_material)
            {
                this->_lesion_class_index = i;
                break;
            }
        }
    }, _first_xpbd_obj);
    std::variant<Sim::XPBDMeshObject_Base*, Sim::FirstOrderXPBDMeshObject_Base*> _first_xpbd_obj;
}

void FocalLesionSimBridge::_focalLesionFactorGraphState(
        const std::shared_ptr<sim_bridge::srv::FocalLesionFactorGraphState::Request> req,
        std::shared_ptr<sim_bridge::srv::FocalLesionFactorGraphState::Response> res)
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
                _updateLastFactorGraphMesh(req->last_mesh, header, mesh, res->updated_last_mesh);
            }
            // regardless, clear the operations cache
            mesh->clearTopologicalOperationCache();
            this->_xpbd_mesh_at_last_fg_query = *mesh;

            // update lesion information

            // get submesh for the lesion class
            const auto [lesion_vertices, lesion_faces, lesion_elements] = obj->tetMesh()->submeshForElementClass(this->_lesion_class_index);
            // copy over data
            // vertices
            res->lesion_vertices.reserve(lesion_vertices.size());
            for (const auto& lesion_vertex : lesion_vertices)
                res->lesion_vertices.push_back(lesion_vertex);

            // faces
            res->lesion_faces.header = header;
            res->lesion_faces.size = lesion_faces.size();
            res->lesion_faces.data.reserve(3*lesion_faces.size());
            res->lesion_faces.invalid_indices.clear();  // all faces valid, don't need to worry about invalid indices
            for (const auto& lesion_face : lesion_faces)
            {
                res->lesion_faces.data.push_back(lesion_face[0]);
                res->lesion_faces.data.push_back(lesion_face[1]);
                res->lesion_faces.data.push_back(lesion_face[2]);
            }

            // elements
            res->lesion_elements.reserve(lesion_elements.size());
            for (const auto& lesion_element : lesion_elements)
                res->lesion_elements.push_back(lesion_element);

            

        }, this->_first_xpbd_obj);
        
    });

    future.wait();
}