#ifndef __VIRTUOSO_ROBOT_GRAPHICS_OBJECT_HPP
#define __VIRTUOSO_ROBOT_GRAPHICS_OBJECT_HPP

#include "simobject/VirtuosoRobot.hpp"
#include "graphics/GraphicsObject.hpp"

namespace Graphics
{

class VirtuosoRobotGraphicsObject : public GraphicsObject
{
    public:
    /** A struct for storing just the information needed for rendering a Virtuoso robot object. */
    struct RenderInfo
    {
        Geometry::CoordinateFrame endoscope_frame;
        Real endoscope_length;
        Real endoscope_diameter;
    };

    explicit VirtuosoRobotGraphicsObject(const std::string& name, const Sim::VirtuosoRobot* virtuoso_robot)
        : GraphicsObject(name), _virtuoso_robot(virtuoso_robot), _latest_rinfo(&_rinfo1)
    {
    }

    /** Updates the snapshot of the Virtuoso robot that should be rendered. */
    void update() override
    {
        RenderInfo* write_info = (_latest_rinfo.load() == &_rinfo1) ? &_rinfo2 : &_rinfo1;

        write_info->endoscope_frame = _virtuoso_robot->endoscopeFrame();
        write_info->endoscope_length = _virtuoso_robot->endoscopeLength();
        write_info->endoscope_diameter = _virtuoso_robot->endoscopeDiameter();

        _latest_rinfo.store(write_info, std::memory_order_release);
    }

    const Sim::VirtuosoRobot* virtuosoRobot() const { return _virtuoso_robot; }

    protected:
    const Sim::VirtuosoRobot* _virtuoso_robot;

    /** Double-buffered render infos. Updated via the update() function. */
    RenderInfo _rinfo1;
    RenderInfo _rinfo2;

    /** Atomic variable to synchronize double buffer. */
    std::atomic<RenderInfo*> _latest_rinfo;

};

} // namespace Graphics

#endif // __VIRTUOSO_ROBOT_GRAPHICS_OBJECT_HPP