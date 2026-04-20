#ifndef __VIRTUOSO_ARM_TOOLS_HPP
#define __VIRTUOSO_ARM_TOOLS_HPP

#include "common/types.hpp"
#include "geometry/SDF.hpp"
#include "geometry/VirtuosoArmToolSDFs.hpp"
#include "geometry/CoordinateFrame.hpp"
#include "simobject/Object.hpp"
#include "simobject/RigidPrimitives.hpp"

#include "solver/xpbd_projector/ConstraintProjectorReference.hpp"
#include "solver/constraint/StaticDeformableCollisionConstraint.hpp"

namespace Sim
{

struct TubeProperties
{
    Real outer_dia=0;
    Real inner_dia=0;
    Real E=0;
    Real G=0;
    Real I=0;
    Real J=0;
};

/** Base class for all tool types */
class VirtuosoArmTool_Base : public Object
{
public:
    VirtuosoArmTool_Base(const Sim::Simulation* sim, const ConfigType* config, const Geometry::CoordinateFrame* it_tip_frame)
        : Object(sim, config), 
        _it_tip_frame(it_tip_frame)
    {}

    virtual ~VirtuosoArmTool_Base() = default;

    virtual void setup() override {}
    virtual void update() override {}
    virtual void velocityUpdate() override {}

    const Geometry::CoordinateFrame* innerTubeTipFramePtr() const { return _it_tip_frame; }
    void setInnerTubeTipFramePtr(const Geometry::CoordinateFrame* tip_frame) { _it_tip_frame = tip_frame; }

    /** Whether or not the tool is a tube that is nested inside the lumen of the inner tube.
     * If true, this affects the effective bending stiffness of the Virtuoso arm.
     */
    virtual bool isTube() const = 0;
    /** If the tool is a nested tube, then this will return the tube properties. */
    virtual TubeProperties tubeProperties() const { return TubeProperties{}; }

    /** Nominal offset from the tip of the inner tube to the tip of the tool.
     * Does not account for any deformation that may happen to the tool (i.e. in the case of the palpation tool)
     */
    virtual Vec3r tipOffset() const = 0;

    /** The current frame of the tip, given the current location of the inner tube tip and any deformation on the tool. */
    virtual Geometry::CoordinateFrame tipFrame() const = 0;

    /** Add a XPBD->static collision constraint projector.
     * Used for getting collision force with deformable objects.
     */
    void addCollisionConstraint(Solver::ConstraintProjectorReferenceWrapper<Solver::StaticDeformableCollisionConstraint>&& collision_proj_ref)
    {
        _collision_proj_refs.push_back(std::move(collision_proj_ref));
    }

    /** Clear XPBD->static collision constraint projectors. */
    void clearCollisionConstraints()
    {
        _collision_proj_refs.clear();
    }

    /** Computes the total tip force and moment due to collision constraint forces, in the GLOBAL frame.
     * Returns a pair (tip force, tip moment).
     */
    std::pair<Vec3r, Vec3r> collisionTipForceAndMoment()
    {
        Vec3r total_force = Vec3r::Zero();
        Vec3r total_moment = Vec3r::Zero();
        for (const auto& collision_proj_ref : _collision_proj_refs)
        {
            if (collision_proj_ref.exists() && collision_proj_ref.isValid())
            {
                // the collision constraints give forces on the tissue, so we must negate them to get the forces on the Virtuoso
                std::vector<Vec3r> forces = collision_proj_ref.constraintForces();
                Vec3r net_force = -std::reduce(forces.cbegin(), forces.cend());
                
                Vec3r cp = collision_proj_ref.constraint()->staticObjectContactPoint();
                total_force += net_force;
                total_moment += (cp - _it_tip_frame->origin()).cross(net_force);
            }
        }

        return {total_force, total_moment};
    }

    /** TODO: define tool actions here somehow? */
protected:
    /** Pointer to frame at the end of the inner tube.
     * Since this is a pointer, it will get updated automatically when the tube frames are recalculated.
     */
    const Geometry::CoordinateFrame* _it_tip_frame;

    /** Vector of XPBD->static collision constraint projectors. */
    std::vector<Solver::ConstraintProjectorReferenceWrapper<Solver::StaticDeformableCollisionConstraint>> _collision_proj_refs;
};

//////////////////////////////////////////////////////////////////////////////////////

/** Spatula tool
 * A rigid spatula affixed at the end of the inner tube.
 */
class VirtuosoArmSpatulaTool : public VirtuosoArmTool_Base
{
public:
    using SDFType = Geometry::VirtuosoArmSpatulaToolSDF;

    /** Static parameters specifying the shape of the rounded rectangle (length, width, radius) and the thickness of the extruded shape. */
    constexpr static Real LENGTH = 4e-3;
    constexpr static Real WIDTH = 2e-3;
    constexpr static Real RADIUS = 0.5e-3;
    constexpr static Real THICKNESS = 0.5e-3;

    VirtuosoArmSpatulaTool(const Sim::Simulation* sim, const ConfigType* config, const Geometry::CoordinateFrame* it_tip_frame)
        : VirtuosoArmTool_Base(sim, config, it_tip_frame)
    {
        _char_dim = THICKNESS;
        _name = _name + "_spatula";
    }

    /** Spatula tool is not a nested tube */
    virtual bool isTube() const override { return false; }

    /** Center frame of spatula */
    Geometry::CoordinateFrame spatulaFrame() const
    {
        // spatula frame is along the z-direction
        Geometry::TransformationMatrix T(Mat3r::Identity(), Vec3r(0, 0, LENGTH/2));
        return _it_tip_frame->operator*(T);
    }

    virtual Vec3r tipOffset() const override { return Vec3r(0,0,LENGTH); }

    virtual Geometry::CoordinateFrame tipFrame() const override
    {
        // spatula frame is along the z-direction
        Geometry::TransformationMatrix T(Mat3r::Identity(), tipOffset());
        return _it_tip_frame->operator*(T);
    }

    /** Returns the axis-aligned bounding-box (AABB) for this Object in global simulation coordinates. */
    virtual Geometry::AABB boundingBox() const
    {
        Geometry::TransformationMatrix T = spatulaFrame().transform();
        Vec3r halfsize = T.rotMat().col(0) * WIDTH/2 + T.rotMat().col(1) * THICKNESS/2 + T.rotMat().col(2) * LENGTH/2;
        Vec3r center = T.translation();
        Vec3r c1 = center - halfsize;
        Vec3r c2 = center + halfsize;

        Geometry::AABB bbox(std::min(c1[0], c2[0]), std::min(c1[1], c2[1]), std::min(c1[2], c2[2]),
                            std::max(c1[0], c2[0]), std::max(c1[1], c2[1]), std::max(c1[2], c2[2]) );
        return bbox;
    }

    virtual void createSDF() override 
    { 
        if(!_sdf.has_value()) 
            _sdf = SDFType(this); 
    }

    virtual const SDFType* SDF() const override { return _sdf.has_value() ? &_sdf.value() : nullptr; }


private:
    std::optional<SDFType> _sdf;
};


/////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Ceramic: 1mm dia, 4m length
// Wire: 0.33mm dia, 4.5m length
/** Cautery tool
 * A rigid 1mm dia, 4mm length ceramic cylinder with a 0.33mm dia, 4.5mm length wire protrusion.
 */
class VirtuosoArmCauteryTool : public VirtuosoArmTool_Base
{
public:
    using SDFType = Geometry::VirtuosoArmCauteryToolSDF;

    /** Static parameters specifying the shape of the rounded rectangle (length, width, radius) and the thickness of the extruded shape. */
    constexpr static Real CERAMIC_LENGTH = 4e-3;
    constexpr static Real CERAMIC_DIA = 1e-3;
    constexpr static Real WIRE_LENGTH = 4.5e-3;
    constexpr static Real WIRE_DIA = 0.33e-3;

    VirtuosoArmCauteryTool(const Sim::Simulation* sim, const ConfigType* config, const Geometry::CoordinateFrame* it_tip_frame)
        : VirtuosoArmTool_Base(sim, config, it_tip_frame)
    {
        _char_dim = WIRE_DIA;
        _name = _name + "_cautery";
    }

    /** Spatula tool is not a nested tube */
    virtual bool isTube() const override { return false; }

    virtual Vec3r tipOffset() const override { return Vec3r(0,0,CERAMIC_LENGTH + WIRE_LENGTH); }

    virtual Geometry::CoordinateFrame tipFrame() const override
    {
        Geometry::TransformationMatrix T(Mat3r::Identity(), tipOffset());
        return _it_tip_frame->operator*(T);
    }

    Geometry::CoordinateFrame ceramicFrame() const
    {
        Geometry::TransformationMatrix T(Mat3r::Identity(), Vec3r(0,0,CERAMIC_LENGTH/2));
        return _it_tip_frame->operator*(T);
    }

    Geometry::CoordinateFrame wireFrame() const
    {
        Geometry::TransformationMatrix T(Mat3r::Identity(), Vec3r(0,0,CERAMIC_LENGTH + WIRE_LENGTH/2));
        return _it_tip_frame->operator*(T);
    }

    /** Returns the axis-aligned bounding-box (AABB) for this Object in global simulation coordinates. */
    virtual Geometry::AABB boundingBox() const
    {
        Geometry::TransformationMatrix T = _it_tip_frame->transform();
        Vec3r halfsize = T.rotMat().col(0) * CERAMIC_DIA/2 + T.rotMat().col(1) * CERAMIC_DIA/2 + T.rotMat().col(2) * CERAMIC_LENGTH/2;
        Vec3r base = T.translation();
        Vec3r tip = tipFrame().origin();

        Geometry::AABB bbox(std::min(base[0], tip[0]), std::min(base[1], tip[1]), std::min(base[2], tip[2]),
                            std::max(base[0], tip[0]), std::max(base[1], tip[1]), std::max(base[2], tip[2]) );
        return bbox;
    }

    virtual void createSDF() override 
    { 
        if(!_sdf.has_value()) 
            _sdf = SDFType(this); 
    }

    virtual const SDFType* SDF() const override { return _sdf.has_value() ? &_sdf.value() : nullptr; }


private:
    std::optional<SDFType> _sdf;
};

} // namespace Sim

#endif // __VIRTUOSO_ARM_TOOLS_HPP