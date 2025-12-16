#ifndef __SPHERE_GRAPHICS_OBJECT_HPP
#define __SPHERE_GRAPHICS_OBJECT_HPP

#include "graphics/GraphicsObject.hpp"
#include "simobject/RigidPrimitives.hpp"

namespace Graphics
{

class SphereGraphicsObject : public GraphicsObject
{
    public:
    
    /** A struct for storing just the information needed to render a sphere. */
    struct RenderInfo
    {
        Geometry::TransformationMatrix transform;
        Real radius;
    };
    
    explicit SphereGraphicsObject(const std::string& name, const Sim::RigidSphere* sphere)
        : GraphicsObject(name), _sphere(sphere), _latest_rinfo(&_rinfo1)
    {
    }

    /** Update the snapshot of the sphere that should be rendered */
    void update() override
    {
        RenderInfo* write_info = (_latest_rinfo.load() == &_rinfo1) ? &_rinfo2 : &_rinfo1;

        write_info->transform = _sphere->transform();
        write_info->radius = _sphere->radius();

        _latest_rinfo.store(write_info, std::memory_order_release);
    }
    
    const Sim::RigidSphere* sphere() const { return _sphere; }

    protected:
    const Sim::RigidSphere* _sphere;

    /** Double-buffered render infos. Updated via the update() function. */
    RenderInfo _rinfo1;
    RenderInfo _rinfo2;

    /** Atomic variable to synchronize double buffer. */
    std::atomic<RenderInfo*> _latest_rinfo;

};

} // namespace Graphics

#endif // __SPHERE_GRAPHICS_OBJECT_HPP