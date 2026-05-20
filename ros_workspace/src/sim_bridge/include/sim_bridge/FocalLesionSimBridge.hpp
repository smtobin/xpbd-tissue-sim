#ifndef __FOCAL_LESION_SIM_BRIDGE_HPP
#define __FOCAL_LESION_SIM_BRIDGE_HPP

#include "sim_bridge/BPHSimBridge.hpp"

#include "sim_bridge/srv/focal_lesion_factor_graph_state.hpp"

class FocalLesionSimBridge : public BPHSimBridge
{
public:
    FocalLesionSimBridge(Sim::VirtuosoCTAnatomySimulation* sim);

private:
    void _focalLesionFactorGraphState(
        const std::shared_ptr<sim_bridge::srv::FocalLesionFactorGraphState::Request> req,
        std::shared_ptr<sim_bridge::srv::FocalLesionFactorGraphState::Response> res);

    int _lesion_class_index;
    rclcpp::Service<sim_bridge::srv::FocalLesionFactorGraphState>::SharedPtr _focal_lesion_factor_graph_service;
};

#endif // __FOCAL_LESION_SIM_BRIDGE_HPP