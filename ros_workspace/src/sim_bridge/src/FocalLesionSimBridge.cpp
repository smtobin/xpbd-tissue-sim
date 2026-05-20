#include "sim_bridge/FocalLesionSimBridge.hpp"

FocalLesionSimBridge::FocalLesionSimBridge(Sim::VirtuosoCTAnatomySimulation* sim)
    : BPHSimBridge(sim)
{
    // set up factor graph service
    _focal_lesion_factor_graph_service = this->create_service<sim_bridge::srv::FactorGraphState>(
        "/sim/focal_lesion_factor_graph_state",
        std::bind(&SimBridge::_focalLesionFactorGraphState, this, std::placeholders::_1, std::placeholders::_2) 
    );

    // get the element class the corresponds to the lesion
    std::visit([&] (auto& obj_ptr) {
        const auto& element_classes = obj_ptr->elementClasses();
        std::string class_name = "Lesion";
        for (unsigned i = 0; i < element_classes.size(); i++)
        {
            if (element_classes[i].name() == class_name)
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
                _updateLastFactorGraphMesh(req->last_mesh, header, req->updated_last_mesh);
            }
            // regardless, clear the operations cache
            mesh->clearTopologicalOperationCache();
            this->_xpbd_mesh_at_last_fg_query = *mesh;

            // TODO: update information about the lesion

        }, this->_first_xpbd_obj);
        
    });

    future.wait();
}