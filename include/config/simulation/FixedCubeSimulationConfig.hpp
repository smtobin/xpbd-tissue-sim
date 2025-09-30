#ifndef __FIXED_CUBE_SIMULATION_CONFIG_HPP
#define __FIXED_CUBE_SIMULATION_CONFIG_HPP

#include "config/simulation/SimulationConfig.hpp"

#include "simulation/FixedCubeSimulation.hpp"

namespace Config
{

class FixedCubeSimulationConfig : public SimulationConfig
{
    static std::map<std::string, Sim::FixedCubeSimulation::CubeFace> CUBE_FIXED_FACE_OPTIONS()
    {
        static std::map<std::string, Sim::FixedCubeSimulation::CubeFace> cube_fixed_face_options{
            {"left", Sim::FixedCubeSimulation::CubeFace::LEFT},
            {"right", Sim::FixedCubeSimulation::CubeFace::RIGHT},
            {"top", Sim::FixedCubeSimulation::CubeFace::TOP},
            {"bottom", Sim::FixedCubeSimulation::CubeFace::BOTTOM}};
        
        return cube_fixed_face_options;
    }

public:
    explicit FixedCubeSimulationConfig(const YAML::Node& node)
        : SimulationConfig(node)
    {
        _extractParameter("text-file-save-interval", node, _text_file_save_interval);

        _extractParameterWithOptions("cube-fixed-face", node, _cube_fixed_face, CUBE_FIXED_FACE_OPTIONS());

        _extractParameter("point-cloud-sample-position", node, _pc_sample_position);
        _extractParameter("point-cloud-sample-orientation", node, _pc_sample_orientation);
    }

    int textFileSaveInterval() const { return _text_file_save_interval.value; }

    Sim::FixedCubeSimulation::CubeFace cubeFixedFace() const { return _cube_fixed_face.value; }

    Vec3r pointCloudSamplePosition() const { return _pc_sample_position.value; }
    Vec3r pointCloudSampleOrientation() const { return _pc_sample_orientation.value; }

private:
    ConfigParameter<int> _text_file_save_interval = ConfigParameter<int>(0);

    ConfigParameter<Sim::FixedCubeSimulation::CubeFace> _cube_fixed_face = ConfigParameter<Sim::FixedCubeSimulation::CubeFace>(Sim::FixedCubeSimulation::CubeFace::BOTTOM);

    ConfigParameter<Vec3r> _pc_sample_position = ConfigParameter<Vec3r>(Vec3r(0,0,0));
    ConfigParameter<Vec3r> _pc_sample_orientation = ConfigParameter<Vec3r>(Vec3r(0,0,0));
};

} // namespace Config

#endif // __FIXED_CUBE_SIMULATION_CONFIG_HPP