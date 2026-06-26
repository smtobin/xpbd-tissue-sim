#ifndef __VIRTUOSO_ARM_GRASPING_TOOL_GRAPHICS_OBJECT_HPP
#define __VIRTUOSO_ARM_GRASPING_TOOL_GRAPHICS_OBJECT_HPP

#include "simobject/VirtuosoArmTools.hpp"
#include "graphics/GraphicsObject.hpp"

namespace Graphics
{

class VirtuosoArmGraspingToolGraphicsObject : public GraphicsObject
{
    public:
    /** A struct for storing just the information needed for rendering a Virtuoso robot object. */
    struct RenderInfo
    {
        Geometry::CoordinateFrame grasp_frame;
    };

    explicit VirtuosoArmGraspingToolGraphicsObject(const std::string& name, const Sim::VirtuosoArmGraspingTool* grasper)
        : GraphicsObject(name), _grasper(grasper), _latest_rinfo(&_rinfo1)
    {
    }

    /** Updates the snapshot of the Virtuoso robot that should be rendered. */
    void update() override
    {
        RenderInfo* write_info = (_latest_rinfo.load() == &_rinfo1) ? &_rinfo2 : &_rinfo1;

        write_info->grasp_frame = _grasper->graspFrame();
        _latest_rinfo.store(write_info, std::memory_order_release);
    }

    const Sim::VirtuosoArmGraspingTool* grasper() const { return _grasper; }

    protected:
    const Sim::VirtuosoArmGraspingTool* _grasper;

    /** Double-buffered render infos. Updated via the update() function. */
    RenderInfo _rinfo1;
    RenderInfo _rinfo2;

    /** Atomic variable to synchronize double buffer. */
    std::atomic<RenderInfo*> _latest_rinfo;

};

} // namespace Graphics

#endif