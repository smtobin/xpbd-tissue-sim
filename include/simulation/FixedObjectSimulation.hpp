#ifndef __FIXED_CUBE_SIMULATION
#define __FIXED_CUBE_SIMULATION

#include "simulation/Simulation.hpp"
#include "simobject/XPBDMeshObjectBaseWrapper.hpp"

// #include "config/FixedObjectSimulationConfig.hpp"

namespace Config
{
    class FixedObjectSimulationConfig;
}

namespace Sim
{

class FixedObjectSimulation : public Simulation
{
public:

    enum class CubeFace
    {
        LEFT=0,
        RIGHT,
        TOP,
        BOTTOM,
        NONE
    };

    FixedObjectSimulation(const Config::FixedObjectSimulationConfig* config);

    virtual std::string type() const override { return "FixedObjectSimulation"; }

    virtual void setup() override;

    const Geometry::CoordinateFrame& pointCloudSampleFrame() const { return _point_cloud_sample_frame; }

    protected:

    virtual void _timeStep() override;

    std::optional<std::string> _fixed_faces_filename;   // a .txt filename that lists all the faces that should be held fixed

    int _num_saved_text_files = 0;
    int _text_file_save_interval;
    std::string _text_file_save_folder;
    int _num_dt_since_last_save = 0;

    CubeFace _fixed_face;
    Geometry::CoordinateFrame _point_cloud_sample_frame;
    XPBDMeshObject_BasePtrWrapper _cube_obj;    // the deformable cube XPBD object
};

} // namespace Sim

#endif // __FIXED_CUBE_SIMULATION