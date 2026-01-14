#ifndef __RIGID_OBJECT_CONFIG_HPP
#define __RIGID_OBJECT_CONFIG_HPP

#include "config/simobject/ObjectConfig.hpp"

namespace Config
{


class RigidObjectConfig : public ObjectConfig
{
    public:

    explicit RigidObjectConfig(const YAML::Node& node)
        : ObjectConfig(node)
    {
        // extract parameters from Config
        _extractParameter("angular-velocity", node, _initial_ang_velocity);
        _extractParameter("density", node, _density);
        _extractParameter("fixed", node, _fixed);
    }

    explicit RigidObjectConfig(const std::string& name, const std::string& material_class, const Vec3r& initial_position, const Vec3r& initial_rotation,
                               const Vec3r& initial_velocity, const Vec3r& initial_angular_velocity, Real density,
                               bool collisions, bool graphics_only, bool fixed,
                               const ObjectRenderConfig& render_config)
        : ObjectConfig(name, material_class, initial_position, initial_rotation, initial_velocity, collisions, graphics_only, render_config)
    {
        _initial_ang_velocity.value = initial_angular_velocity;
        _density.value = density;
        _fixed.value = fixed;
    }

    Vec3r initialAngularVelocity() const { return _initial_ang_velocity.value; }
    Real density() const { return _density.value; }
    bool fixed() const { return _fixed.value; }

    protected:
    ConfigParameter<Vec3r> _initial_ang_velocity = ConfigParameter<Vec3r>(Vec3r(0,0,0));
    ConfigParameter<Real> _density = ConfigParameter<Real>(1000);
    ConfigParameter<bool> _fixed = ConfigParameter<bool>(false);

};

} // namespace config

#endif // __RIGID_OBJECT_CONFIG_HPP