#ifndef __VIRTUOSO_ROBOT_CONFIG_HPP
#define __VIRTUOSO_ROBOT_CONFIG_HPP

#include "config/simobject/ObjectConfig.hpp"
#include "config/simobject/VirtuosoArmConfig.hpp"

namespace Sim
{
    class Object;
    class VirtuosoRobot;
}

namespace Config
{

class VirtuosoRobotConfig : public ObjectConfig
{
    public:
    using ObjectType = Sim::VirtuosoRobot;

    public:

    explicit VirtuosoRobotConfig(const YAML::Node& node)
        : ObjectConfig(node)
    {
        _extractParameter("endoscope-diameter", node, _endoscope_diameter);
        _extractParameter("endoscope-length", node, _endoscope_length);
        _extractParameter("arm-separation-distance", node, _arm_separation_dist);
        _extractParameter("optic-vertical-distance", node, _optic_vertical_dist);
        _extractParameter("optic-tilt", node, _optic_tilt);

        // create a MeshObject for each object specified in the YAML file
        for (const auto& obj_node : node["arms"])
        {
            std::string type;
            try 
            {
                // extract type information
                type = obj_node["type"].as<std::string>();
            }
            catch (const std::exception& e)
            {
                std::cerr << e.what() << std::endl;
                std::cerr << "Type of object is needed!" << std::endl;
                continue;
            }
            

            // create the specified type of object based on type string
            if (type == "VirtuosoArm")                   
                _arm_configs.emplace_back(obj_node);
            else
            {
                std::cerr << "Unknown type of object! \"" << type << "\" is not a type of VirtuosoArm." << std::endl;
                assert(0);
            }
        }
    }

    explicit VirtuosoRobotConfig( const std::string& name, const std::string& material_class,
        const Vec3r& initial_pos, const Vec3r& initial_rot, const Vec3r& initial_velocity, bool collisions, bool graphics_only,
        Real endoscope_diameter, Real endoscope_length, Real arm_separation_distance, Real optic_vertical_distance, Real optic_tilt_deg,
        const std::vector<VirtuosoArmConfig>& arm_configs,
        const ObjectRenderConfig& render_config)
        : ObjectConfig(name, material_class, initial_pos, initial_rot, initial_velocity, collisions, graphics_only, render_config),
            _arm_configs(arm_configs)
    {
        _endoscope_diameter.value = endoscope_diameter;
        _endoscope_length.value = endoscope_length;
        _arm_separation_dist.value = arm_separation_distance;
        _optic_vertical_dist.value = optic_vertical_distance;
        _optic_tilt.value = optic_tilt_deg;
    }

    std::unique_ptr<ObjectType> createObject(const Sim::Simulation* sim) const;

    const std::vector<VirtuosoArmConfig>& armConfigs() const { return _arm_configs; }
    Real endoscopeDiameter() const { return _endoscope_diameter.value; }
    Real endoscopeLength() const { return _endoscope_length.value; }
    Real armSeparationDistance() const { return _arm_separation_dist.value; }
    Real opticVerticalDistance() const { return _optic_vertical_dist.value; }
    Real opticTilt() const { return _optic_tilt.value; }

    protected:
    ConfigParameter<Real> _endoscope_diameter = ConfigParameter<Real>(8.67e-3);
    ConfigParameter<Real> _endoscope_length = ConfigParameter<Real>(0.05);
    ConfigParameter<Real> _arm_separation_dist = ConfigParameter<Real>(3.05e-3);
    ConfigParameter<Real> _optic_vertical_dist = ConfigParameter<Real>(3.24e-3);
    ConfigParameter<Real> _optic_tilt = ConfigParameter<Real>(30);

    std::vector<VirtuosoArmConfig> _arm_configs;
};

} // namespace Config

#endif // __VIRTUOSO_ROBOT_CONFIG_HPP