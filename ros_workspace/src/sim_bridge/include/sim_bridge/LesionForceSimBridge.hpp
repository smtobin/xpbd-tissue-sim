#ifndef __LESION_FORCE_SIM_BRIDGE_HPP
#define __LESION_FORCE_SIM_BRIDGE_HPP

#include "sim_bridge/FocalLesionSimBridge.hpp"
#include "simulation/LesionForceSimulation.hpp"

class LesionForceSimBridge : public FocalLesionSimBridge
{
public:
    LesionForceSimBridge(Sim::LesionForceSimulation* sim);

protected:
    rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr _lesion_body_force_subscriber;

    Sim::LesionForceSimulation* _lesion_sim;
};

#endif // __LESION_FORCE_SIM_BRIDGE_HPP