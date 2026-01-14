#ifndef __RIGID_PRIMITIVE_CONFIGS_HPP
#define __RIGID_PRIMITIVE_CONFIGS_HPP

#include "config/simobject/RigidObjectConfig.hpp"

namespace Sim
{
    class RigidSphere;
    class RigidBox;
    class RigidCylinder;
}

namespace Config
{

class RigidSphereConfig : public RigidObjectConfig
{
    public:
    using ObjectType = Sim::RigidSphere;

    public:

    explicit RigidSphereConfig(const YAML::Node& node)
        : RigidObjectConfig(node)
    {
        _extractParameter("radius", node, _radius);
    }

    explicit RigidSphereConfig(const std::string& name, const std::string& material_class, const Vec3r& initial_position, const Vec3r& initial_rotation,
        const Vec3r& initial_velocity, const Vec3r& initial_angular_velocity, Real density, Real radius,
        bool collisions, bool graphics_only, bool fixed,
        const ObjectRenderConfig& render_config)
        : RigidObjectConfig(name, material_class, initial_position, initial_rotation, initial_velocity, initial_angular_velocity, density, collisions, graphics_only, fixed, render_config)
    {
        _radius.value = radius;
    }

    std::unique_ptr<ObjectType> createObject(const Sim::Simulation* sim) const;


    Real radius() const { return _radius.value; }

    protected:
    ConfigParameter<Real> _radius = ConfigParameter<Real>(1);

};

///////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////

class RigidBoxConfig : public RigidObjectConfig
{
    public:
    using ObjectType = Sim::RigidBox;

    public:

    explicit RigidBoxConfig(const YAML::Node& node)
        : RigidObjectConfig(node)
    {
        _extractParameter("size", node, _size);
    }

    explicit RigidBoxConfig(const std::string& name, const std::string& material_class, const Vec3r& initial_position, const Vec3r& initial_rotation,
        const Vec3r& initial_velocity, const Vec3r& initial_angular_velocity, Real density, const Vec3r& size,
        bool collisions, bool graphics_only, bool fixed,
        const ObjectRenderConfig& render_config)
        : RigidObjectConfig(name, material_class, initial_position, initial_rotation, initial_velocity, initial_angular_velocity, density, collisions, graphics_only, fixed, render_config)
    {
        _size.value = size;
    }

    std::unique_ptr<ObjectType> createObject(const Sim::Simulation* sim) const;

    Vec3r size() const { return _size.value; }

    protected:
    ConfigParameter<Vec3r> _size = ConfigParameter<Vec3r>(Vec3r(1,1,1));
};

////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////

class RigidCylinderConfig : public RigidObjectConfig
{
    public:
    using ObjectType = Sim::RigidCylinder;

    public:
    
    explicit RigidCylinderConfig(const YAML::Node& node)
        : RigidObjectConfig(node)
    {
        _extractParameter("radius", node, _radius);
        _extractParameter("height", node, _height);
    }

    explicit RigidCylinderConfig(const std::string& name, const std::string& material_class, const Vec3r& initial_position, const Vec3r& initial_rotation,
        const Vec3r& initial_velocity, const Vec3r& initial_angular_velocity, Real density, Real radius, Real height,
        bool collisions, bool graphics_only, bool fixed,
        const ObjectRenderConfig& render_config)
        : RigidObjectConfig(name, material_class, initial_position, initial_rotation, initial_velocity, initial_angular_velocity, density, collisions, graphics_only, fixed, render_config)
    {
        _radius.value = radius;
        _height.value = height;
    }

    std::unique_ptr<ObjectType> createObject(const Sim::Simulation* sim) const;

    Real radius() const { return _radius.value; }
    Real height() const { return _height.value; }

    protected:
    ConfigParameter<Real> _radius = ConfigParameter<Real>(1);
    ConfigParameter<Real> _height = ConfigParameter<Real>(1);
};

} // namespace Config

#endif // __RIGID_PRIMITIVE_CONFIGS_HPP