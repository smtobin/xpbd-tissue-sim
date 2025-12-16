#ifndef __EASY3D_SPHERE_GRAPHICS_OBJECT_HPP
#define __EASY3D_SPHERE_GRAPHICS_OBJECT_HPP

#include "graphics/SphereGraphicsObject.hpp"

#include <easy3d/core/surface_mesh.h>
#include <easy3d/core/model.h>

namespace Graphics
{

class Easy3DSphereGraphicsObject : public SphereGraphicsObject, public easy3d::Model
{
    public:
    explicit Easy3DSphereGraphicsObject(const std::string& name, const Sim::RigidSphere* sphere);

    virtual void updateGraphicsBuffers() override;

    /** Returns the easy3d::vec3 vertex cache.
     * Does NOT check if vertices are stale.
     * 
     * A required override for the easy3d::Model class.
     */
    std::vector<easy3d::vec3>& points() override { return _e3d_mesh.points(); };

    /** Returns the easy3d::vec3 vertex cache.
     * Does NOT check if vertices are stale.
     * 
     * A required override for the easy3d::Model class.
     */
    const std::vector<easy3d::vec3>& points() const override { return _e3d_mesh.points(); };

    private:
    void _transformPoints();

    protected:
    Real _last_radius;
    easy3d::SurfaceMesh _e3d_mesh;
    std::vector<easy3d::vec3> _initial_points;
};

} // namespace Graphics

#endif // __EASY3D_SPHERE_GRAPHICS_OBJECT_HPP