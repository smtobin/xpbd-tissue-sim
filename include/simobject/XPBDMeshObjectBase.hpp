#ifndef __XPBD_MESH_OBJECT_BASE_HPP
#define __XPBD_MESH_OBJECT_BASE_HPP

// #include "config/simobject/XPBDMeshObjectConfig.hpp"
// #include "config/simobject"

#include "simobject/Object.hpp"
#include "simobject/MeshObject.hpp"
#include "simobject/ElasticMaterial.hpp"
#include "simobject/RigidObject.hpp"

#include "solver/xpbd_projector/ConstraintProjectorReference.hpp"
#include "solver/constraint/StaticDeformableCollisionConstraint.hpp"
#include "solver/constraint/RigidDeformableCollisionConstraint.hpp"
#include "solver/constraint/AttachmentConstraint.hpp"

#include "geometry/DeformableMeshSDF.hpp"

#include "common/XPBDEnumTypes.hpp"

#include <variant>

// TODO: resolve circular dependenciees! Too many bandaids everywhere
namespace Config
{
    class XPBDMeshObjectConfig;
    class FirstOrderXPBDMeshObjectConfig;
}

// TODO: fix circular dependency and remove the need for this forward declaration
namespace Solver
{
    class DeviatoricConstraint;
    class HydrostaticConstraint;
    class StaticDeformableCollisionConstraint;
    class RigidDeformableCollisionConstraint;
    class CollisionConstraint;
    class AttachmentConstraint;

    template<bool, class T>
    class ConstraintProjector;

    template<bool, class T>
    class RigidBodyConstraintProjector;

    template<bool, class T1, class T2>
    class CombinedConstraintProjector;

    template<bool, class... Ts>
    class XPBDSolver;
}

namespace Sim
{

template<bool IsFirstOrder>
class XPBDMeshObject_Base_;

using XPBDMeshObject_Base = XPBDMeshObject_Base_<false>;
using FirstOrderXPBDMeshObject_Base = XPBDMeshObject_Base_<true>;

/** Base class for XPBDMeshObject. This is useful because then we can store a base pointer of this class type without worrying about the specific
 * XPBDSolver type and Constraint types used by the object (these show up as additional template parameters in XPBDMeshObject).
 * Basic functionalities are defined here, such as vertex properties (mass, velocity, etc.).
 * 
 * When the template parameter IsFirstOrder is true, the derived XPBDMeshObject uses the 1st-order formulation of XPBD, and as such defines a few
 * necessasry additional methods and members for the 1st-order algorithm (such as per-vertex damping).
 */
template<bool IsFirstOrder> 
class XPBDMeshObject_Base_ : public Object, public TetMeshObject
{
public:
    using SDFType = Geometry::DeformableMeshSDF;
    using ConfigType = typename std::conditional<IsFirstOrder, Config::FirstOrderXPBDMeshObjectConfig, Config::XPBDMeshObjectConfig>::type;

    public:
    explicit XPBDMeshObject_Base_(const Simulation* sim, const ConfigType* config);

    virtual ~XPBDMeshObject_Base_() {}

    /** Returns a const-ref to the elastic material for each tetrahedra in the mesh.
     * @returns the elastic material
     */
    const std::vector<ElasticMaterial>& materials() const { return _materials; }

    /** Creates the SDF if it doesn't exist already. */
    virtual void createSDF() override;

    /** Returns the SDF if it exists, null otherwise
     */
    virtual const SDFType* SDF() const override { return _sdf.has_value() ? &_sdf.value() : nullptr; }


    /** === Querying vertex properties === */

    /** Fixes a vertex in the mesh so that it will not move.
     * @param index : the index of the vertex to make fixed
     */
    void fixVertex(int index) { _is_fixed_vertex[index] = true; }

    /** Whether the vertex at the specified index is fixed.
     * @param index : the index of the vertex to query if fixed
     * @returns if the queried vertex is fixed or not
     */
    bool vertexFixed(int index) const { return _is_fixed_vertex[index]; }

    /** The mass of the vertex at the specified index.
     * @param index : the index of the vertex
     * @returns the mass of the vertex at the specified index
     */
    Real vertexMass(int index) const { return _vertex_masses[index]; }

    /** The velocity of the vertex at the specified index.
     * @param index : the index of the vertex
     * @returns the velocity of the vertex at the specified index
     */
    Vec3r vertexVelocity(int index) const { return _vertex_velocities.col(index); }

    /** The previous position of the vertex at the specified index.
     * @param index : the index of the vertex
     * @returns the previous position of the vertex at the specified index
     */
    Vec3r vertexPreviousPosition(int index) const { return _previous_vertices.col(index); }

    /** Returns the "constraint inertia" associated with the vertex.
     * For normal 2nd-order XPBD, this is just the vertex mass.
     * For 1st-Order XPBD, the vertex damping is used as the "mass" in the XPBD updates.
     * @param index : the index of the vertex
     * @returns the appropriate "inertia", depending on IsFirstOrder, to be used in the XPBD update.
     */
    Real vertexConstraintInertia(int index) const
    {
        if constexpr (IsFirstOrder)
            return vertexDamping(index);
        else
            return vertexMass(index);
    }

    /** === Adding/removing additional constraints === */

    /** Adds a collision constraint between a face on this object and a point on a static object in the scene.
     * @param sdf : the SDF of the static object
     * @param surface_point : the surface point on the static object
     * @param collision_normal : the collision normal
     * @param face_ind : the index of the face in collision
     * @param u,v,w : the barycentric coordinates of the point on the face in collision
     * @returns a reference to the constraint projector that was added for the collision constraint
     */
    virtual Solver::ConstraintProjectorReference<Solver::ConstraintProjector<IsFirstOrder, Solver::StaticDeformableCollisionConstraint>>
    addStaticCollisionConstraint(const Geometry::SDF* sdf, const Vec3r& surface_point, const Vec3r& collision_normal,
        int face_ind, const Real u, const Real v, const Real w) = 0;

    /** Adds a collision constraint between a face on this object and a point on a rigid object in the scene.
     * @param sdf : the SDF of the rigid object
     * @param rigid_obj : a pointer to the rigid object in collision
     * @param rigid_body_point : the surface point on the rigid object in collision (global coordinates)
     * @param collision_normal : the collision normal
     * @param face_ind : the index of the face in collision
     * @param u,v,w : the barycentric coordinates of the point on the face in collision
     * @returns a reference to the constraint projector that was added for the collision constraint
     */
    virtual Solver::ConstraintProjectorReference<Solver::RigidBodyConstraintProjector<IsFirstOrder, Solver::RigidDeformableCollisionConstraint>>
    addRigidDeformableCollisionConstraint(const Geometry::SDF* sdf, Sim::RigidObject* rigid_obj, const Vec3r& rigid_body_point, const Vec3r& collision_normal,
        int face_ind, const Real u, const Real v, const Real w) = 0;

    /** Clears all collision constraints that are on this object. */
    virtual void clearCollisionConstraints() = 0;
    
    /** Adds an attachment constraint applied to the vertex at the specified index. TODO: clean this up a bit? The Vec3r pointer is a bit gross.
     * @param v_ind : the index of the vertex
     * @param attach_pos_ptr : a pointer to the position for the vertex to be attached to
     * @param attachment_offset : an optional offset between the attachment position and the vertex. The vertex position will be (*attach_pos_ptr + attachment_offset).
     */
    virtual Solver::ConstraintProjectorReference<Solver::ConstraintProjector<IsFirstOrder, Solver::AttachmentConstraint>> 
    addAttachmentConstraint(int v_ind, const Vec3r* attach_pos_ptr, const Vec3r& attachment_offset) = 0;

    /** Clears all attachment constraint that are on this object. */
    virtual void clearAttachmentConstraints() = 0;

    /** Performs a check for self collision.
     * If any surface vertices are inside tetrahedra (queries made using Embree), add a collision constraint to fix that.
     * Assumes that the Embree scene is up to date.
     */
    virtual void selfCollisionCheck() = 0;


    /** === Querying the solver === */

    /** @returns the most recently calculated primary residual from the solver object */
    virtual VecXr lastPrimaryResidual() const = 0;

    /** @returns the most recently calculated constraint residual from the solver object */
    virtual VecXr lastConstraintResidual() const = 0;


    /** === Miscellaneous useful methods === */

    /** Computes the total strain energy associated with elastic deformation.
    */
    virtual Real totalStrainEnergy() const = 0;

    /** Computes the elastic force on the vertex at the specified index. This is essentially just the current constraint force for all "elastic" constraints
     * that affect the specified vertex. An "elastic" constraint is one that is internal to the mesh and corresponds to the mechanics of the mesh material.
     * @param index : the index of the vertex
     * @returns the elastic force vector on the vertex at the specified index
     */
    virtual Vec3r elasticForceAtVertex(int index) const = 0;

    /** Computes the current global stiffness matrix of the mesh. This is done with a first-order approximation of delC^T * alpha * delC.
     * @returns the global stiffness matrix
     */
    virtual MatXr stiffnessMatrixOLD() const = 0;

    /** Computes the current global stiffness matrix of the mesh. This is done with a first-order approximation of delC^T * alpha * delC.
     * @returns the global stiffness matrix
     */
    virtual MatXr stiffnessMatrix() const = 0;


    /** === Methods specific to 1st-Order algorithm === */

    /** Returns the vertex damping for the vertex at the specified index.
     * @param index : the index of the vertex
     * @returns the 1st-order vertex damping for the specified vertexs
     */
    template<bool B = IsFirstOrder>
    typename std::enable_if<B, Real>::type vertexDamping(int index) const { return _vertex_B[index]; }


protected:
    /** Stores the vertices from the end of the previous time step */
    Geometry::Mesh::VerticesMat _previous_vertices;
    /** Stores the current velocities of each vertex */
    Geometry::Mesh::VerticesMat _vertex_velocities;

    /** The initial bulk velocity of the mesh. Set by the config. TODO: is this needed? */
    Vec3r _initial_velocity;

    /** The elastic materials for the mesh.
     * The index in the vector corresponds to the class (an integer) associated with this material.
     * The class is a per-element property stored by the TetMesh object.
     */
    std::vector<ElasticMaterial> _materials;

    /** Stores the vertex masses. */
    std::vector<Real> _vertex_masses;
    /** Stores the vertex "volumes". This is the total volume of all tetrahedra attached to a vertex, divided by 4. */
    std::vector<Real> _vertex_volumes;
    /** Whether or not a given vertex is fixed. */
    std::vector<bool> _is_fixed_vertex;

    /** Signed Distance Field for the deformable object. Must be created explicitly with createSDF(). */
    std::optional<SDFType> _sdf;


    /** === Class members specific to when the object is 1st-order === */
    // std::conditional is used to optionally create the additional member variables
    // std::monostate (an empty struct) is used to represent a not-present member variable (when IsFirstOrder is false)

    /** The damping multiplier for the mesh (this is b from the 1st-order paper) */
    typename std::conditional<IsFirstOrder, Real, std::monostate>::type _damping_multiplier;

    /** Whether or not to adjust the damping according to the Young's modulus and Poisson's ratio of the material.
     * The "damping-multiplier" set by the config is taken to be b*(1+nu)/E, so b gets set correctly when E or nu change.
     * This is useful for meshes that have multiple materials involved.
     */
    typename std::conditional<IsFirstOrder, bool, std::monostate>::type _adjust_b_to_material;

    /** The damping at each vertex */
    typename std::conditional<IsFirstOrder, std::vector<Real>, std::monostate>::type _vertex_B;
};

}

#endif // __XBPD_MESH_OBJECT_BASE_HPP