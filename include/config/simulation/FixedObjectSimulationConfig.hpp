#ifndef __FIXED_CUBE_SIMULATION_CONFIG_HPP
#define __FIXED_CUBE_SIMULATION_CONFIG_HPP

#include "config/simulation/SimulationConfig.hpp"

#include "simulation/FixedObjectSimulation.hpp"

namespace Config
{

class FixedObjectSimulationConfig : public SimulationConfig
{
    static std::map<std::string, Sim::FixedObjectSimulation::CubeFace> CUBE_FIXED_FACE_OPTIONS()
    {
        static std::map<std::string, Sim::FixedObjectSimulation::CubeFace> cube_fixed_face_options{
            {"left", Sim::FixedObjectSimulation::CubeFace::LEFT},
            {"right", Sim::FixedObjectSimulation::CubeFace::RIGHT},
            {"top", Sim::FixedObjectSimulation::CubeFace::TOP},
            {"bottom", Sim::FixedObjectSimulation::CubeFace::BOTTOM},
            {"none", Sim::FixedObjectSimulation::CubeFace::NONE}};
        
        return cube_fixed_face_options;
    }

public:
    explicit FixedObjectSimulationConfig(const YAML::Node& node)
        : SimulationConfig(node)
    {
        _extractParameter("fixed-faces-filename", node, _fixed_faces_filename);

        _extractParameter("text-file-save-interval", node, _text_file_save_interval);
        _extractParameter("text-file-save-folder", node, _text_file_save_folder);

        _extractParameterWithOptions("cube-fixed-face", node, _cube_fixed_face, CUBE_FIXED_FACE_OPTIONS());

        _extractParameter("point-cloud-sample-position", node, _pc_sample_position);
        _extractParameter("point-cloud-sample-orientation", node, _pc_sample_orientation);
    }

    std::optional<std::string> fixedFacesFilename() const { return _fixed_faces_filename.value; }

    int textFileSaveInterval() const { return _text_file_save_interval.value; }
    std::string textFileSaveFolder() const { return _text_file_save_folder.value; }

    Sim::FixedObjectSimulation::CubeFace cubeFixedFace() const { return _cube_fixed_face.value; }

    Vec3r pointCloudSamplePosition() const { return _pc_sample_position.value; }
    Vec3r pointCloudSampleOrientation() const { return _pc_sample_orientation.value; }

private:
    ConfigParameter<std::optional<std::string>> _fixed_faces_filename;

    ConfigParameter<int> _text_file_save_interval = ConfigParameter<int>(0);
    ConfigParameter<std::string> _text_file_save_folder = ConfigParameter<std::string>("./");

    ConfigParameter<Sim::FixedObjectSimulation::CubeFace> _cube_fixed_face = ConfigParameter<Sim::FixedObjectSimulation::CubeFace>(Sim::FixedObjectSimulation::CubeFace::BOTTOM);

    ConfigParameter<Vec3r> _pc_sample_position = ConfigParameter<Vec3r>(Vec3r(0,0,0));
    ConfigParameter<Vec3r> _pc_sample_orientation = ConfigParameter<Vec3r>(Vec3r(0,0,0));
};

} // namespace Config

#endif // __FIXED_CUBE_SIMULATION_CONFIG_HPP