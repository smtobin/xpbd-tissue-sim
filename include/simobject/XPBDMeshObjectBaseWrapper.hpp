#ifndef __XPBD_MESH_OBJECT_BASE_WRAPPER_HPP
#define __XPBD_MESH_OBJECT_BASE_WRAPPER_HPP

#include "simobject/XPBDMeshObjectBase.hpp"

#include "solver/xpbd_projector/ConstraintProjectorReference.hpp"

#include <variant>

namespace Sim
{

/** A wrapper class around XPBDMeshObject_Base_*. (note that this class stores pointers to XPBDMeshObject_Base_)
 * Since XPBDMeshObject_Base_ is templated with a single boolean template parameter indicating whether the object is using the
 * 1st-order or 2nd-order XPBD algorithm, we can't just store a XPBDMeshObject_Base_*.
 * 
 * This class circumvents that by using a variant type to store either XPBDMeshObject_Base_<false>* or XPBDMeshObject_Base_<true>*.
 * This allows other classes that interact with XPBDMeshObject (such as VirtuosoArm or some of the Simulation classes) to treat
 * both 1st-order and 2nd-order XPBDMeshObjects the same, which should be the case since there's only a few minor implementational
 * differences between them that don't matter in 95% of cases.
 * 
 * Basically all this class does is wrap all the functionality of XPBDMeshObject_Base_, agnostic to the boolean template parameter.
 * 
 * Is this better than just having a virtual base class that is one level below XPBDMeshObject_Base_? Not sure. I want avoid deep
 * class hierarchies, but this class has a lot of boilerplate that needs to be changed every time XPBDMeshObject_Base_ changes, which
 * isn't ideal.
 * I also like strong typing, and lean towards keeping as much type information as possible, which this approach does more than
 * having a base class beneath XPBDMeshObject_Base_.
 */
class XPBDMeshObject_BasePtrWrapper
{
public:
    using SDFType = XPBDMeshObject_Base::SDFType;

    template<bool IsFirstOrder>
    XPBDMeshObject_BasePtrWrapper(XPBDMeshObject_Base_<IsFirstOrder>* obj) : _variant(obj) {}

    /** When not initialized, the variant will store nullptr. */
    XPBDMeshObject_BasePtrWrapper() : _variant( (XPBDMeshObject_Base_<true>*)nullptr ) {}

    void serialize(std::vector<std::byte>& buf) const
    {
        pack(buf, _variant);
    }

    void deserialize(const std::byte*& buf)
    {
        unpack(buf, _variant);
    }

    /** If the current stored pointer is not nullptr, evaluates to true. */
    explicit operator bool() const
    {
        return std::visit([](const auto& obj) { 
            return (obj != nullptr); 
        }, _variant);
    }

    void operator=(FirstOrderXPBDMeshObject_Base* other)
    {
        _variant = other;
    }

    void operator=(XPBDMeshObject_Base* other)
    {
        _variant = other;
    }

    /** Return the stored object as the specified type, if the type matches.
     * If the type doesn't match, return nullptr
     */
    template<bool IsFirstOrder>
    XPBDMeshObject_Base_<IsFirstOrder>* getAs()
    {
        if (std::holds_alternative<XPBDMeshObject_Base_<IsFirstOrder>>(_variant))
        {
            return std::get<XPBDMeshObject_Base_<IsFirstOrder>>(_variant);
        }
        else
        {
            return nullptr;
        }
    }

    template<bool IsFirstOrder>
    const XPBDMeshObject_Base_<IsFirstOrder>* getAs() const
    {
        if (std::holds_alternative<XPBDMeshObject_Base_<IsFirstOrder>>(_variant))
        {
            return std::get<XPBDMeshObject_Base_<IsFirstOrder>>(_variant);
        }
        else
        {
            return nullptr;
        }
    }

    TetMeshObject* getAsTetMeshObject()
    {
        return std::visit([](auto& obj) { return (TetMeshObject*)obj; }, _variant);
    }

    const TetMeshObject* getAsTetMeshObject() const
    {
        return std::visit([](const auto& obj) { return (const TetMeshObject*)obj; }, _variant);
    }

    /** === Object functionality === */
    std::string name() const
    {
        return std::visit([](const auto& obj) { return obj->name(); }, _variant);
    }

    Geometry::AABB boundingBox() const
    {
        return std::visit([](const auto& obj) { return obj->boundingBox(); }, _variant);
    }

    /** === XPBDMeshObject_Base_ functionality === */
    /** TODO: should some of these methods use perfect forwarding? */

    const std::vector<const MaterialClass*>& materialClasses() const
    {
        return std::visit([](const auto& obj) -> const std::vector<const MaterialClass*>& { return obj->materialClasses(); }, _variant);
    }

    const SDFType* SDF() const
    {
        return std::visit([](const auto& obj) -> const SDFType* { return obj->SDF(); }, _variant );
    }

    void fixVertex(int index)
    {
        return std::visit([&](auto& obj) { return obj->fixVertex(index); }, _variant);
    }

    bool vertexFixed(int index) const
    {
        return std::visit([&](const auto& obj) { return obj->vertexFixed(index); }, _variant);
    }

    Real vertexMass(int index) const
    {
        return std::visit([&](const auto& obj) { return obj->vertexMass(index); }, _variant);
    }

    Vec3r vertexVelocity(int index) const
    {
        return std::visit([&](const auto& obj) { return obj->vertexVelocity(index); }, _variant);
    }

    Vec3r vertexPreviousPosition(int index) const
    {
        return std::visit([&](const auto& obj) { return obj->vertexPreviousPosition(index); }, _variant);
    }

    Real vertexConstraintInertia(int index) const
    {
        return std::visit([&](const auto& obj) { return obj->vertexConstraintInertia(index); }, _variant);
    }

    /** === Adding external constraints ===
     * Returns a generic ConstraintProjectorReferenceWrapper that does not care if the underlying ConstraintProjector is 1st-Order or 2nd-Order,
     * much like this wrapper class. This allows us to do things with externally added constraints, like compute and apply constraint forces to
     * objects that the XPBDMeshObject is interacting with, without caring if the object is 1st-Order or 2nd-Order. 
     */

    Solver::ConstraintProjectorReferenceWrapper<Solver::StaticDeformableCollisionConstraint>
    addStaticCollisionConstraint(const Geometry::SDF* sdf, const Vec3r& surface_point, const Vec3r& collision_normal,
        int v1, int v2, int v3, const Real u, const Real v, const Real w,
        int element_ind, int face_ind)
    {
        return std::visit([&](auto& obj)
        {
            return Solver::ConstraintProjectorReferenceWrapper<Solver::StaticDeformableCollisionConstraint>(
                obj->addStaticCollisionConstraint(sdf, surface_point, collision_normal, v1, v2, v3, u, v, w, element_ind, face_ind)
            );
        }, _variant);
    }

    Solver::ConstraintProjectorReferenceWrapper<Solver::RigidDeformableCollisionConstraint>
    addRigidDeformableCollisionConstraint(const Geometry::SDF* sdf, Sim::RigidObject* rigid_obj, const Vec3r& rigid_body_point, const Vec3r& collision_normal,
        int face_ind, const Real u, const Real v, const Real w)
    {
        return std::visit([&](auto& obj)
        {
            return Solver::ConstraintProjectorReferenceWrapper<Solver::RigidDeformableCollisionConstraint>(
                obj->addRigidDeformableCollisionConstraint(sdf, rigid_obj, rigid_body_point, collision_normal, face_ind, u, v, w)
            );
        }, _variant);
    }

    Solver::ConstraintProjectorReferenceWrapper<Solver::OffsetAttachmentConstraint>
    addOffsetAttachmentConstraint(int v_ind, const Vec3r* attach_pos_ptr, const Vec3r& attachment_offset)
    {
        return std::visit([&](auto& obj)
        {
            return Solver::ConstraintProjectorReferenceWrapper<Solver::OffsetAttachmentConstraint>(
                obj->addOffsetAttachmentConstraint(v_ind, attach_pos_ptr, attachment_offset)
            );
        }, _variant);
        
    }

    Solver::ConstraintProjectorReferenceWrapper<Solver::ElementOffsetAttachmentConstraint>
    addElementOffsetAttachmentConstraint(int elem_ind, const Vec4r& bary_coords, const Vec3r* attach_pos_ptr, const Vec3r& attachment_offset)
    {
        return std::visit([&](auto& obj)
        {
            return Solver::ConstraintProjectorReferenceWrapper<Solver::ElementOffsetAttachmentConstraint>(
                obj->addElementOffsetAttachmentConstraint(elem_ind, bary_coords, attach_pos_ptr, attachment_offset)
            );
        }, _variant);
        
    }

    void clearCollisionConstraints()
    {
        return std::visit([](auto& obj) { obj->clearCollisionConstraints(); }, _variant);
    }

    

    void clearOffsetAttachmentConstraints()
    {
        return std::visit([](auto& obj) { obj->clearOffsetAttachmentConstraints(); }, _variant);
    }

    void clearElementOffsetAttachmentConstraints()
    {
        return std::visit([](auto& obj) { obj->clearElementOffsetAttachmentConstraints(); }, _variant);
    }

    /** === Mesh topology === */

    void removeElement(int elem_index)
    {
        return std::visit([&](auto& obj) { obj->removeElement(elem_index); }, _variant);
    }

    void refineElement(int elem_index, int refinement_level, bool absolute)
    {
        return std::visit([&](auto& obj) { obj->refineElement(elem_index, refinement_level, absolute); }, _variant);
    }

    void coarsenElement(int elem_index, int coarsening_level, bool absolute)
    {
        return std::visit([&](auto& obj) { obj->coarsenElement(elem_index, coarsening_level, absolute); }, _variant);
    }

    /** === Miscellaneous === */

    Real totalStrainEnergy() const
    {
        return std::visit([&](const auto& obj) { return obj->totalStrainEnergy(); }, _variant);
    }

    Vec3r elasticForceAtVertex(int index) const
    {
        return std::visit([&](const auto& obj) { return obj->elasticForceAtVertex(index); }, _variant);
    }

    MatXr stiffnessMatrix() const
    {
        return std::visit([](const auto& obj) { return obj->stiffnessMatrix(); }, _variant);
    }

    void selfCollisionCheck()
    {
        return std::visit([](auto& obj) { return obj->selfCollisionCheck(); }, _variant);
    }

    VecXr lastPrimaryResidual() const
    {
        return std::visit([&](const auto& obj) { return obj->lastPrimaryResidual(); }, _variant);
    }

    VecXr lastConstraintResidual() const
    {
        return std::visit([&](const auto& obj) { return obj->lastConstraintResidual(); }, _variant);
    }

    /** Queries whether or not the heat solver exists. */
    bool hasHeatSolver() const
    { 
        return std::visit([&](const auto& obj) { return obj->hasHeatSolver(); }, _variant);
    }

    /** @returns the heat solver */
    FEM::HeatConductionFEMSolver& heatSolver()
    {
        return std::visit([&](auto& obj) -> FEM::HeatConductionFEMSolver& { return obj->heatSolver(); }, _variant);
    }

    const FEM::HeatConductionFEMSolver& heatSolver() const
    { 
        return std::visit([&](const auto& obj) -> const FEM::HeatConductionFEMSolver& { return obj->heatSolver(); }, _variant);
    }


    /** === TetMeshObject functionality === */

    const Geometry::Mesh* mesh() const
    {
        return std::visit([](const auto& obj) { return obj->mesh(); }, _variant);
    }

    Geometry::Mesh* mesh()
    {
        return std::visit([](auto& obj) { return obj->mesh(); }, _variant);
    }

    const Geometry::TetMesh* tetMesh() const
    {
        return std::visit([](const auto& obj) { return obj->tetMesh(); }, _variant);
    }

    Geometry::TetMesh* tetMesh()
    {
        return std::visit([](auto& obj) { return obj->tetMesh(); }, _variant);
    }

    const Geometry::RefinedTetMesh* refinedTetMesh() const
    {
        return std::visit([](const auto& obj) { return obj->refinedTetMesh(); }, _variant);
    }

    Geometry::RefinedTetMesh* refinedTetMesh()
    {
        return std::visit([](auto& obj) { return obj->refinedTetMesh(); }, _variant);
    }

private:
    std::variant<XPBDMeshObject_Base_<true>*, XPBDMeshObject_Base_<false>*> _variant;
};

} // namespace Sim

#endif // __XPBD_MESH_OBJECT_BASE_WRAPPER_HPP