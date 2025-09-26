#ifndef __FIXED_CUBE_SIMULATION
#define __FIXED_CUBE_SIMULATION

#include "simulation/Simulation.hpp"
#include "simobject/XPBDMeshObjectBaseWrapper.hpp"

// #include "config/FixedCubeSimulationConfig.hpp"

namespace Config
{
    class FixedCubeSimulationConfig;
}

namespace Sim
{

class FixedCubeSimulation : public Simulation
{
public:

    enum class CubeFace
    {
        LEFT=0,
        RIGHT,
        TOP,
        BOTTOM
    };

    FixedCubeSimulation(const Config::FixedCubeSimulationConfig* config);

    virtual std::string type() const override { return "FixedCubeSimulation"; }

    virtual void setup() override;

    protected:

    virtual void _timeStep() override;


    int _text_file_save_interval;
    int _num_dt_since_last_save = 0;

    CubeFace _fixed_face;
    Geometry::CoordinateFrame _point_cloud_sample_frame;
    XPBDMeshObject_BasePtrWrapper _cube_obj;    // the deformable cube XPBD object
};

} // namespace Sim

#endif // __FIXED_CUBE_SIMULATION