#include "sim_bridge/LesionForceSimBridge.hpp"

LesionForceSimBridge::LesionForceSimBridge(Sim::LesionForceSimulation* sim)
    : FocalLesionSimBridge(sim), _lesion_sim(sim)
{
    auto lesion_body_force_callback = [&](geometry_msgs::msg::Vector3Stamped::UniquePtr msg) {
        Vec3r force(msg->vector.x, msg->vector.y, msg->vector.z);
        std::cout << "LesionForceSimBridge - received lesion body force: " << force.transpose() << std::endl;
        
        auto callback = [this, force]() -> void
        {
            this->_lesion_sim->setLesionBodyForce(force);
        };
        this->_lesion_sim->addOneTimeCallback(callback);
    };

    _lesion_body_force_subscriber = this->create_subscription<geometry_msgs::msg::Vector3Stamped>("sim/input/lesion_body_force", 10, lesion_body_force_callback);
}