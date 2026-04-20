#ifndef __VIRTUOSO_ARM_CAUTERY_TOOL_GRAPHICS_OBJECT_HPP
#define __VIRTUOSO_ARM_CAUTERY_TOOL_GRAPHICS_OBJECT_HPP

#include "simobject/VirtuosoArmTools.hpp"
#include "graphics/GraphicsObject.hpp"

namespace Graphics
{

class VirtuosoArmCauteryToolGraphicsObject : public GraphicsObject
{
    public:
    /** A struct for storing just the information needed for rendering a Virtuoso robot object. */
    struct RenderInfo
    {
        Geometry::CoordinateFrame ceramic_frame;
        Geometry::CoordinateFrame wire_frame;
    };

    explicit VirtuosoArmCauteryToolGraphicsObject(const std::string& name, const Sim::VirtuosoArmCauteryTool* cautery)
        : GraphicsObject(name), _cautery(cautery), _latest_rinfo(&_rinfo1)
    {
    }

    /** Updates the snapshot of the Virtuoso robot that should be rendered. */
    void update() override
    {
        RenderInfo* write_info = (_latest_rinfo.load() == &_rinfo1) ? &_rinfo2 : &_rinfo1;

        write_info->ceramic_frame = _cautery->ceramicFrame();
        write_info->wire_frame = _cautery->wireFrame();
        _latest_rinfo.store(write_info, std::memory_order_release);
    }

    const Sim::VirtuosoArmCauteryTool* cautery() const { return _cautery; }

    protected:
    const Sim::VirtuosoArmCauteryTool* _cautery;

    /** Double-buffered render infos. Updated via the update() function. */
    RenderInfo _rinfo1;
    RenderInfo _rinfo2;

    /** Atomic variable to synchronize double buffer. */
    std::atomic<RenderInfo*> _latest_rinfo;

};

} // namespace Graphics

#endif