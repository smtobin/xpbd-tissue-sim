#ifndef __RIGID_MESH_OBJECT_CONFIG_HPP
#define __RIGID_MESH_OBJECT_CONFIG_HPP

#include "config/simobject/RigidObjectConfig.hpp"
#include "config/simobject/MeshObjectConfig.hpp"

namespace Sim
{
    class RigidMeshObject;
}

namespace Config
{
    
class RigidMeshObjectConfig : public RigidObjectConfig, public MeshObjectConfig
{
    public:
    using ObjectType = Sim::RigidMeshObject;

    public:

    explicit RigidMeshObjectConfig(const YAML::Node& node)
        : RigidObjectConfig(node), MeshObjectConfig(node)
    {
        _extractParameter("sdf-filename", node, _sdf_filename);
    }

    explicit RigidMeshObjectConfig( const std::string& name, const Vec3r& initial_position, const Vec3r& initial_rotation,
                                    const Vec3r& initial_velocity, const Vec3r& initial_angular_velocity, Real density,
                                    bool collisions, bool graphics_only, bool fixed,
                                    const std::string& filename, const std::optional<Real>& max_size, const std::optional<Vec3r>& size, const std::optional<Vec3r>& scaling,
                                    bool draw_points, bool draw_edges, bool draw_faces, const Vec4r& color,
                                    const std::optional<std::string>& sdf_filename,
                                    const ObjectRenderConfig& render_config )
        : RigidObjectConfig(name, initial_position, initial_rotation, initial_velocity, initial_angular_velocity, density, collisions, graphics_only, fixed, render_config),
          MeshObjectConfig(filename, max_size, size, scaling, draw_points, draw_edges, draw_faces, color)
    {
        _sdf_filename.value = sdf_filename;
    }

    std::unique_ptr<ObjectType> createObject(const Sim::Simulation* sim) const;

    std::optional<std::string> sdfFilename() const { return _sdf_filename.value; }

    protected:
    ConfigParameter<std::optional<std::string>> _sdf_filename;
};

} // namespace Config

#endif // __RIGID_MESH_OBJECT_CONFIG