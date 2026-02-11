#include "sim_bridge/VirtuosoCTAnatomySimBridge.hpp"

VirtuosoCTAnatomySimBridge::VirtuosoCTAnatomySimBridge(Sim::VirtuosoCTAnatomySimulation* sim)
    : VirtuosoSimBridge(sim)
{
    _setupCTtoVBTransformListener();
}


void VirtuosoCTAnatomySimBridge::_setupCTtoVBTransformListener()
{
    this->declare_parameter("CT_frame_name", "ct/base");

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

    _sim->addCallback(0.05, ct_to_vb_callback);
}