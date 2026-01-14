#ifndef __FIRST_ORDER_XPBD_MESH_OBJECT_CONFIG_HPP
#define __FIRST_ORDER_XPBD_MESH_OBJECT_CONFIG_HPP

#include "config/simobject/XPBDMeshObjectConfig.hpp"

namespace Config
{

class FirstOrderXPBDMeshObjectConfig : public XPBDMeshObjectConfig
{
    public:
    using ObjectType = Sim::XPBDMeshObject_Base_<true>;

    public:
    /** Creates a Config from a YAML node, which consists of the specialized parameters needed for XPBDMeshObject.
     * @param node : the YAML node (i.e. dictionary of key-value pairs) that information is pulled from
     */
    explicit FirstOrderXPBDMeshObjectConfig(const YAML::Node& node)
        : XPBDMeshObjectConfig(node)
    {
        _extractParameter("damping-multiplier", node, _damping_multiplier);
        _extractParameter("adjust-damping-to-material", node, _adjust_damping_to_material);      
    }

    explicit FirstOrderXPBDMeshObjectConfig(  
                                    const std::string& name, const std::string& material_class,
                                    const Vec3r& initial_position, const Vec3r& initial_rotation,                  // Object params
                                    const Vec3r& initial_velocity, bool collisions, bool graphics_only,

                                    const std::string& filename, const std::optional<Real>& max_size, const std::optional<Vec3r>& size, const std::optional<Vec3r>& scaling,    // MeshObject params
                                    bool draw_points, bool draw_edges, bool draw_faces, const Vec4r& color,

                                    const std::vector<std::string>& mat_names, const std::optional<std::string>& element_classes_filename,
                                    const std::optional<std::string>& fixed_faces_filename,

                                    bool self_collisions, int num_solver_iters, int num_local_collision_iters, 
                                    XPBDObjectSolverTypeEnum solver_type, XPBDMeshObjectConstraintConfigurationEnum constraint_type,                   // XPBDMeshObject params
                                    XPBDSolverResidualPolicyEnum residual_policy,
                                
                                    Real damping_multiplier, bool adjust_damping_to_material,
                                
                                    const ObjectRenderConfig& render_config)  
                                                                                                                                            // FirstOrderXPBDMeshObject params
        : XPBDMeshObjectConfig(name, material_class, initial_position, initial_rotation, initial_velocity, collisions, graphics_only,
                                filename, max_size, size, scaling, draw_points, draw_edges, draw_faces, color,
                                mat_names, element_classes_filename, fixed_faces_filename,
                                self_collisions, num_solver_iters, num_local_collision_iters, solver_type, constraint_type, residual_policy,
                                render_config)
    {
        _damping_multiplier.value = damping_multiplier;
        _adjust_damping_to_material.value = adjust_damping_to_material;
    }

    std::unique_ptr<ObjectType> createObject(const Sim::Simulation* sim) const;

    Real dampingMultiplier() const { return _damping_multiplier.value; }
    bool adjustDampingToMaterial() const { return _adjust_damping_to_material.value; }

    protected:
    ConfigParameter<Real> _damping_multiplier = ConfigParameter<Real>(1);
    ConfigParameter<bool> _adjust_damping_to_material = ConfigParameter<bool>(false);
};

} // namespace Config

#endif // __FIRST_ORDER_XPBD_MESH_OBJECT_CONFIG_HPP