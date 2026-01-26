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

        _extractParameter("device-name1", node, _device_name1);
        _extractParameter("device-name2", node, _device_name2);

        _extractParameter("CT-to-VB-transform-translation", node, _CT_to_VB_translation);
        _extractParameter("CT-to-VB-transform-rotation", node, _CT_to_VB_rotation);
    }

    std::optional<std::string> deviceName1() const { return _device_name1.value; }
    std::optional<std::string> deviceName2() const { return _device_name2.value; }

    Geometry::TransformationMatrix CTtoVBTransform() const
    {
        Mat3r rot_mat = GeometryUtils::quatToMat(GeometryUtils::eulXYZ2Quat(_CT_to_VB_rotation.value[0], _CT_to_VB_rotation.value[1], _CT_to_VB_rotation.value[2]));
        return Geometry::TransformationMatrix(rot_mat, _CT_to_VB_translation.value);
    }

    protected:

    ConfigParameter<std::optional<std::string>> _device_name1;
    ConfigParameter<std::optional<std::string>> _device_name2;
    
    ConfigParameter<Vec3r> _CT_to_VB_translation = ConfigParameter<Vec3r>(Vec3r(0, 0, 0.1));
    ConfigParameter<Vec3r> _CT_to_VB_rotation = ConfigParameter<Vec3r>(Vec3r::Zero());
};

} // namespace Config

#endif // __VIRTUOSO_TISSUE_GRASPING_TISSUE_SIMULATION_CONFIG_HPP