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
#include "solver/constraint/OffsetAttachmentConstraint.hpp"
#include "solver/constraint/ElementOffsetAttachmentConstraint.hpp"
#include "solver/constraint/AttachmentConstraint.hpp"

#include "geometry/DeformableMeshSDF.hpp"

#include "fem/HeatConductionFEMSolver.hpp"

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
    class OffsetAttachmentConstraint;

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
class XPBDMeshObject_Base_ : public Object, public RefinedTetMeshObject
{
public:
    using SDFType = Geometry::DeformableMeshSDF;
    using ConfigType = typename std::conditional<IsFirstOrder, Config::FirstOrderXPBDMeshObjectConfig, Config::XPBDMeshObjectConfig>::type;

    public:
    XPBDMeshObject_Base_() = default;
    explicit XPBDMeshObject_Base_(const Simulation* sim, const ConfigType* config);

    virtual ~XPBDMeshObject_Base_() {}

    virtual void serialize(std::vector<std::byte>& buf) const override;
    virtual void deserialize(const std::byte*& buf) override;

    /** Returns a const-ref to the elastic material for each tetrahedra in the mesh.
     * @returns the elastic material
     */
    const std::vector<const MaterialClass*>& materialClasses() const { return _material_classes; }

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
    Vec3r vertexVelocity(int index) const { return _vertex_velocities.at(index); }

    /** The previous position of the vertex at the specified index.
     * @param index : the index of the vertex
     * @returns the previous position of the vertex at the specified index
     */
    Vec3r vertexPreviousPosition(int index) const { return _previous_vertices.at(index); }

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
    addStaticCollisionConstraint(
        const Geometry::SDF* sdf, const Vec3r& p, const Vec3r& n,
        int v1, int v2, int v3, const Real u, const Real v, const Real w,
        int element_ind, int face_ind
    ) = 0;

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
    
    /** Adds an offset attachment constraint applied to the vertex at the specified index.
     * @param v_ind : the index of the vertex
     * @param attach_pos_ptr : a pointer to the position for the vertex to be attached to
     * @param attachment_offset : an optional offset between the attachment position and the vertex. The vertex position will be (*attach_pos_ptr + attachment_offset).
     */
    virtual Solver::ConstraintProjectorReference<Solver::ConstraintProjector<IsFirstOrder, Solver::OffsetAttachmentConstraint>>  
    addOffsetAttachmentConstraint(int v_ind, const Vec3r* attach_pos_ptr, const Vec3r& attachment_offset) = 0;

    /** Clears all offset attachment constraint that are on this object. */
    virtual void clearOffsetAttachmentConstraints() = 0;

    /** Adds an element offset attachment constraint applied to the vertex at the specified index.
     * @param elem_ind : the index of the element
     * @param bary_coords : the barycentric coordinates describing the point within the element that is attached
     * @param attach_pos_ptr : a pointer to the position for the vertex to be attached to
     * @param attachment_offset : an optional offset between the attachment position and the vertex. The vertex position will be (*attach_pos_ptr + attachment_offset).
     */
    virtual Solver::ConstraintProjectorReference<Solver::ConstraintProjector<IsFirstOrder, Solver::ElementOffsetAttachmentConstraint>>  
    addElementOffsetAttachmentConstraint(int elem_ind, const Vec4r& bary_coords, const Vec3r* attach_pos_ptr, const Vec3r& attachment_offset) = 0;

    /** Clears all element offset attachment constraint that are on this object. */
    virtual void clearElementOffsetAttachmentConstraints() = 0;

    /** Adds an attachment constraint applied to the vertex at the specified index
     * @param v_ind : the index of the vertex
     * @param attach_pos_ptr : a pointer to the position for the vertex to be attached to
     */
    virtual Solver::ConstraintProjectorReference<Solver::ConstraintProjector<IsFirstOrder, Solver::AttachmentConstraint>>  
    addAttachmentConstraint(int v_ind, const Vec3r* attach_pos_ptr) = 0;

    /** Adds an attachment constraint applied to the vertex at the specified index
     * @param v_ind : the index of the vertex
     * @param attach_ind : the index of the attachment point in the vector
     * @param attach_pos_ptr : a pointer to the vector that contains the attachment point
     */
    virtual Solver::ConstraintProjectorReference<Solver::ConstraintProjector<IsFirstOrder, Solver::AttachmentConstraint>>  
    addAttachmentConstraint(int v_ind, int attach_ind, const std::vector<Vec3r>* attach_pos_ptr) = 0;

    /** Clears all attachment constraint that are on this object. */
    virtual void clearAttachmentConstraints() = 0;

    /** Computes the total force exerted on this object from the attachment constraints. */
    virtual Vec3r attachmentConstraintTotalForce() const = 0;

    /** Performs a check for self collision.
     * If any surface vertices are inside tetrahedra (queries made using Embree), add a collision constraint to fix that.
     * Assumes that the Embree scene is up to date.
     */
    virtual void selfCollisionCheck() = 0;

    
    /** === Editing mesh topology === */

    /** Removes an element from the mesh object.
     * This will update the mesh representation and disable any internal constraints associated with that element.
     */
    virtual void removeElement(int elem_index) = 0;

    /** Refines an element in the mesh object via recursive, hierarchical refinement.
     * The element gets split into 8 equal-volume child tetrahedra, and each child gets split into 8 equal-volume child tetrahedra, etc... 
     * until the specified refinement level is reached.
     * @param elem_index : the index of the element
     * @param refinement_level : the number of refinements to do
     * @param absolute : when True, the refinement_level parameter is taken to be the "absolute" refinement_level. 
     * I.e. refinement_level = 0 is the base tet mesh, refinement_level = 1 is a base element split into 8 children, etc.
     * The refinement stops once the absolute refinement level has been reached, and does nothing if the refinement level is not greater than the current level of the element.
     * When False, the refinement_level parameter is taken to be the "relative" refinement level, and always refines by the number of levels specified.
     */
    virtual void refineElement(int elem_index, int refinement_level, bool absolute) = 0;

    /** Coarsens an element in the mesh object via recursive coarsening. (basically undoes refinement from refineElement() ).
     * This will not coarsen the mesh to be coarser than the original tet mesh.
     * 
     * If coarsening one level, the element and all of its siblings will be replaced by their parent element (8 elements -> 1 element)
     * If coarsening two levels, the element and all of its siblings and cousins will be replaced by their grandparent element (64 elements -> 1 element)
     * etc.
     * 
     * To undo all refinement that resulted in the leaf element, use coarsening_level=0 and absolute=true.
     * 
     * If the specified element was not created with mesh refinement, this function does nothing.
     * 
     * @param elem_index : the index of the element to coarsen
     * @param coarsening_level : the number of coarsening operations to do (i.e. the number of levels up the tree to traverse)
     * @param absolute : defined the same as for refineElement()
     */
    virtual void coarsenElement(int elem_index, int coarsening_level, bool absolute) = 0;


    /** === Querying the solver === */

    /** @returns the most recently calculated primary residual from the solver object */
    virtual VecXr lastPrimaryResidual() const = 0;

    /** @returns the most recently calculated constraint residual from the solver object */
    virtual VecXr lastConstraintResidual() const = 0;

    /** Queries whether or not the heat solver exists. */
    bool hasHeatSolver() const { return _heat_solver.has_value(); }

    /** @returns the heat solver */
    FEM::HeatConductionFEMSolver& heatSolver() { return *_heat_solver; }
    const FEM::HeatConductionFEMSolver& heatSolver() const { return *_heat_solver; }

    bool adaptiveMeshRefinement() const { return _adaptive_mesh_refinement; }
    int maxRefinementLevel() const { return _max_refinement_level; }
    Real refinementDistanceThreshold() const { return _refinement_distance_threshold; }


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
    virtual Eigen::SparseMatrix<Real> stiffnessMatrix() const = 0;


    /** === Methods specific to 1st-Order algorithm === */

    /** Returns the vertex damping for the vertex at the specified index.
     * @param index : the index of the vertex
     * @returns the 1st-order vertex damping for the specified vertexs
     */
    template<bool B = IsFirstOrder>
    typename std::enable_if<B, Real>::type vertexDamping(int index) const { return _vertex_B[index]; }


protected:
    /** TODO: does _previous_vertices and _vertex_velocities need to be vertices_vec_type? Or can they just be plain old std::vector? */
    /** Stores the vertices from the end of the previous time step */
    std::vector<Vec3r> _previous_vertices;
    /** Stores the current velocities of each vertex */
    std::vector<Vec3r> _vertex_velocities;

    /** The initial bulk velocity of the mesh. Set by the config. TODO: is this needed? */
    Vec3r _initial_velocity;

    /** The elastic materials for the mesh.
     * The index in the vector corresponds to the class (an integer) associated with this material.
     * The class is a per-element property stored by the TetMesh object.
     */
    std::vector<const MaterialClass*> _material_classes;

    /** Stores the vertex masses. */
    std::vector<Real> _vertex_masses;
    /** Whether or not a given vertex is fixed. */
    std::vector<bool> _is_fixed_vertex;

    /** Signed Distance Field for the deformable object. Must be created explicitly with createSDF(). */
    std::optional<SDFType> _sdf;

    /** Heat conduction solver for computing thermal effects. This is optional, and specified in the config file to be created. */
    std::optional<FEM::HeatConductionFEMSolver> _heat_solver;

    /** Whether or not to adaptively refine the mesh. Set by the config. */
    bool _adaptive_mesh_refinement;

    /** When adaptive mesh refinement is enabled, the maximum number of recursive refinements of base elements that are allowed. Set by the config. */
    int _max_refinement_level;

    /** The distance threshold to use to decide when refinement happens. */
    Real _refinement_distance_threshold;


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