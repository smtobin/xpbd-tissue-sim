#ifndef __CYLINDER_GRAPHICS_OBJECT_HPP
#define __CYLINDER_GRAPHICS_OBJECT_HPP

#include "graphics/GraphicsObject.hpp"
#include "simobject/RigidPrimitives.hpp"

namespace Graphics
{

class CylinderGraphicsObject : public GraphicsObject
{
    public:
    
    /** A struct for storing just the information needed to render a cylinder. */
    struct RenderInfo
    {
        Geometry::TransformationMatrix transform;
        Real radius;
        Real height;
    };

    explicit CylinderGraphicsObject(const std::string& name, const Sim::RigidCylinder* cylinder)
        : GraphicsObject(name), _cylinder(cylinder), _latest_rinfo(&_rinfo1)
    {
    }

    /** Update the snapshot of the cylinder that should be rendered */
    void update() override
    {
        RenderInfo* write_info = (_latest_rinfo.load() == &_rinfo1) ? &_rinfo2 : &_rinfo1;

        write_info->transform = _cylinder->transform();
        write_info->radius = _cylinder->radius();
        write_info->height = _cylinder->height();

        _latest_rinfo.store(write_info, std::memory_order_release);
    }

    const Sim::RigidCylinder* cylinder() const { return _cylinder; }

    protected:
    const Sim::RigidCylinder* _cylinder;

    /** Double-buffered render infos. Updated via the update() function. */
    RenderInfo _rinfo1;
    RenderInfo _rinfo2;

    /** Atomic variable to synchronize double buffer. */
    std::atomic<RenderInfo*> _latest_rinfo;
};

} // namespace Graphics

#endif // __CYLINDER_GRAPHICS_OBJECT_HPP