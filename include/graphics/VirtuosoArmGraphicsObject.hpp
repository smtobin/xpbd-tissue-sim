#ifndef __VIRTUOSO_ARM_GRAPHICS_OBJECT_HPP
#define __VIRTUOSO_ARM_GRAPHICS_OBJECT_HPP

#include "graphics/GraphicsObject.hpp"
#include "simobject/VirtuosoArm.hpp"

namespace Graphics
{

class VirtuosoArmGraphicsObject : public GraphicsObject
{
    public:
    /** A struct for storing just the information needed to render a Virtuoso arm. */
    struct RenderInfo
    {
        Sim::VirtuosoArm::OuterTubeFramesArray ot_frames;
        Sim::VirtuosoArm::InnerTubeFramesArray it_frames;
        Real ot_outer_diameter = 1e-3;
        Real it_outer_diameter = 1e-3;
    };

    explicit VirtuosoArmGraphicsObject(const std::string& name, const Sim::VirtuosoArm* virtuoso_arm)
        : GraphicsObject(name), _virtuoso_arm(virtuoso_arm), _latest_rinfo(&_rinfo1)
    {
    }

    /** Updates the snapshot of the Virtuoso arm that should be rendered. */
    void update() override
    {
        RenderInfo* write_info = (_latest_rinfo.load() == &_rinfo1) ? &_rinfo2 : &_rinfo1;

        write_info->ot_frames = _virtuoso_arm->outerTubeFrames();
        write_info->it_frames = _virtuoso_arm->innerTubeFrames();
        write_info->ot_outer_diameter = _virtuoso_arm->outerTubeOuterDiameter();
        write_info->it_outer_diameter = _virtuoso_arm->innerTubeOuterDiameter();

        _latest_rinfo.store(write_info, std::memory_order_release);
    }

    const Sim::VirtuosoArm* virtuosoArm() const { return _virtuoso_arm; }

    protected:
    const Sim::VirtuosoArm* _virtuoso_arm;

    /** Double-buffered render infos. Updated via the update() function. */
    RenderInfo _rinfo1;
    RenderInfo _rinfo2;

    /** Atomic variable to synchronize double buffer. */
    std::atomic<RenderInfo*> _latest_rinfo;

};

} // namespace Graphics

#endif // __VIRTUOSO_ARM_GRAPHICS_OBJECT_HPP