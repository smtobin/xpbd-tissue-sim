#ifndef __VIRTUOSO_TISSUE_GRASPING_SIM_BRIDGE_HPP
#define __VIRTUOSO_TISSUE_GRASPING_SIM_BRIDGE_HPP

#include "sim_bridge/VirtuosoSimBridge.hpp"

#include "simulation/VirtuosoCTAnatomySimulation.hpp"

class VirtuosoCTAnatomySimBridge : public VirtuosoSimBridge
{
public:
    VirtuosoCTAnatomySimBridge(Sim::VirtuosoCTAnatomySimulation* sim);

private:

    void _setupCTtoVBTransformListener();
};

#endif // __VIRTUOSO_TISSUE_GRASPING_SIM_BRIDGE_HPP