#include "simulation/StiffnessMapSimulation.hpp"

namespace Sim
{

StiffnessMapSimulation::StiffnessMapSimulation(const Config::StiffnessMapSimulationConfig* config)
    : Simulation(config)
{
    _prescribed_displacement = Vec3r::Zero();
}

void StiffnessMapSimulation::setup()
{
    Simulation::setup();

    // find the XPBDMeshObject (which we're assuming to be the tissue)
    // first check 1st-order XPBDMeshObjects
    auto& fo_xpbd_objs = _objects.template get<std::unique_ptr<FirstOrderXPBDMeshObject_Base>>();
    auto& xpbd_objs = _objects.template get<std::unique_ptr<XPBDMeshObject_Base>>();
    if (fo_xpbd_objs.size() > 0)
        _tissue_obj = fo_xpbd_objs.front().get();
    else if (xpbd_objs.size() > 0)
        _tissue_obj = xpbd_objs.front().get();
    else
        assert((xpbd_objs.size() + fo_xpbd_objs.size() > 0) && "There must be at least 1 XPBDMeshObject or FirstOrderXPBDMeshObject in the simulation (there is no tissue object!).");
}

void StiffnessMapSimulation::_timeStep()
{
    Simulation::_timeStep();
}

} // namespace Sim