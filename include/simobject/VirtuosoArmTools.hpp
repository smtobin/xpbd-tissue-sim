#ifndef __VIRTUOSO_ARM_TOOLS_HPP
#define __VIRTUOSO_ARM_TOOLS_HPP

#include "common/types.hpp"
#include "geometry/SDF.hpp"
#include "geometry/VirtuosoArmToolSDFs.hpp"
#include "geometry/CoordinateFrame.hpp"
#include "simobject/Object.hpp"
#include "simobject/RigidPrimitives.hpp"

namespace Sim
{

struct TubeProperties
{
    Real outer_dia;
    Real inner_dia;
    Real E;
    Real G;
    Real I;
    Real J;
};

/** Base class for all tool types */
class VirtuosoArmTool_Base : public Object
{
public:
    VirtuosoArmTool_Base(Sim::Simulation* sim, const ConfigType* config, const Geometry::CoordinateFrame* it_tip_frame)
        : Object(sim, config), 
        _it_tip_frame(it_tip_frame)
    {}

    virtual ~VirtuosoArmTool_Base() = default;

    virtual void setup() override {}
    virtual void update() override {}
    virtual void velocityUpdate() override {}

    /** Whether or not the tool is a tube that is nested inside the lumen of the inner tube.
     * If true, this affects the effective bending stiffness of the Virtuoso arm.
     */
    virtual bool isTube() const = 0;
    /** If the tool is a nested tube, then this will return the tube properties. */
    virtual TubeProperties tubeProperties() const { return TubeProperties{}; }

    /** Offset from the tip of the inner tube to the tip of the tool */
    virtual Vec3r tipOffset() const = 0;

    /** TODO: define tool actions here somehow? */
protected:
    /** Pointer to frame at the end of the inner tube.
     * Since this is a pointer, it will get updated automatically when the tube frames are recalculated.
     */
    const Geometry::CoordinateFrame* _it_tip_frame;
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

    VirtuosoArmSpatulaTool(Sim::Simulation* sim, ConfigType* config, const Geometry::CoordinateFrame* it_tip_frame)
        : VirtuosoArmTool_Base(sim, config, it_tip_frame)
    {
        _char_dim = THICKNESS;
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


} // namespace Sim

#endif // __VIRTUOSO_ARM_TOOLS_HPP