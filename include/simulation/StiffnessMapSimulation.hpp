#ifndef __STIFFNESS_MAP_SIMULATION_HPP
#define __STIFFNESS_MAP_SIMULATION_HPP

#include "simulation/Simulation.hpp"
#include "config/simulation/StiffnessMapSimulationConfig.hpp"

#include "simobject/XPBDMeshObjectBaseWrapper.hpp"

namespace Sim
{

class StiffnessMapSimulation : public Simulation
{
public:

    StiffnessMapSimulation(const Config::StiffnessMapSimulationConfig* config);

    virtual std::string type() const override { return "SitffnessMapSimulation"; }

    virtual void setup() override;

    const Geometry::TetMesh* tissueMesh() const { assert(_tissue_obj); return _tissue_obj.tetMesh(); }

protected:
    
    void _timeStep() override;

protected:

    XPBDMeshObject_BasePtrWrapper _tissue_obj;    // the tissue XPBD object that is being manipulated

    /** For querying points on the mesh */
    int _face_ind;
    Vec3r _face_barys;
    Vec3r _prescribed_displacement; 
    Real _time_to_steady_state;
    Real _displacement_application_time;
};

} // namespace Sim

#endif // __STIFFNESS_MAP_SIMULATION_HPP