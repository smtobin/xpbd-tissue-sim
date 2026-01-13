#ifndef __VIRTUOSO_TISSUE_GRASPING_TISSUE_SIMULATION_CONFIG_HPP
#define __VIRTUOSO_TISSUE_GRASPING_TISSUE_SIMULATION_CONFIG_HPP

#include "config/simulation/VirtuosoSimulationConfig.hpp"

namespace Config
{

class VirtuosoTissueGraspingSimulationConfig : public VirtuosoSimulationConfig
{

    public:
    explicit VirtuosoTissueGraspingSimulationConfig(const YAML::Node& node)
        : VirtuosoSimulationConfig(node)
    {
        _extractParameter("fixed-faces-filename", node, _fixed_faces_filename);

        _extractParameter("device-name1", node, _device_name1);
        _extractParameter("device-name2", node, _device_name2);

        _extractParameter("express-meshes-in-vb-frame", node, _express_meshes_in_vb_frame);
    }

    std::optional<std::string> fixedFacesFilename() const { return _fixed_faces_filename.value; }

    std::optional<std::string> deviceName1() const { return _device_name1.value; }
    std::optional<std::string> deviceName2() const { return _device_name2.value; }

    bool expressMeshesInVBFrame() const { return _express_meshes_in_vb_frame.value; }

    protected:
    ConfigParameter<std::optional<std::string>> _fixed_faces_filename;

    ConfigParameter<std::optional<std::string>> _device_name1;
    ConfigParameter<std::optional<std::string>> _device_name2;

    ConfigParameter<bool> _express_meshes_in_vb_frame = ConfigParameter<bool>(false);
};

} // namespace Config

#endif // __VIRTUOSO_TISSUE_GRASPING_TISSUE_SIMULATION_CONFIG_HPP