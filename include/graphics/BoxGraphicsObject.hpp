#ifndef __BOX_GRAPHICS_OBJECT_HPP
#define __BOX_GRAPHICS_OBJECT_HPP

#include "graphics/GraphicsObject.hpp"
#include "simobject/RigidPrimitives.hpp"

namespace Graphics
{

class BoxGraphicsObject : public GraphicsObject
{
    public:

    /** A struct for storing just the information needed to render a box. */
    struct RenderInfo
    {
        Geometry::TransformationMatrix transform;
        Vec3r size;
    };

    explicit BoxGraphicsObject(const std::string& name, const Sim::RigidBox* box)
        : GraphicsObject(name), _box(box), _latest_rinfo(&_rinfo1)
    {
    }

    /** Update the snapshot of the cylinder that should be rendered */
    void update() override
    {
        RenderInfo* write_info = (_latest_rinfo.load() == &_rinfo1) ? &_rinfo2 : &_rinfo1;

        write_info->transform = _box->transform();
        write_info->size = _box->size();

        _latest_rinfo.store(write_info, std::memory_order_release);
    }

    const Sim::RigidBox* box() const { return _box; }

    protected:
    const Sim::RigidBox* _box;

    /** Double-buffered render infos. Updated via the update() function. */
    RenderInfo _rinfo1;
    RenderInfo _rinfo2;

    /** Atomic variable to synchronize double buffer. */
    std::atomic<RenderInfo*> _latest_rinfo;
};

} // namespace Graphics

#endif // __BOX_GRAPHICS_OBJECT_HPP