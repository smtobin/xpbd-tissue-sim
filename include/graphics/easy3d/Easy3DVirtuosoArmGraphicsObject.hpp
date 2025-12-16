#ifndef __EASY_3D_VIRTUOSO_ARM_GRAPHICS_OBJECT_HPP
#define __EASY_3D_VIRTUOSO_ARM_GRAPHICS_OBJECT_HPP

#include "graphics/VirtuosoArmGraphicsObject.hpp"

#include <easy3d/core/surface_mesh.h>
#include <easy3d/core/model.h>

namespace Graphics
{

class Easy3DVirtuosoArmGraphicsObject : public VirtuosoArmGraphicsObject, public easy3d::Model
{
    public:
    explicit Easy3DVirtuosoArmGraphicsObject(const std::string& name, const Sim::VirtuosoArm* virtuoso_arm);

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
    /** Creates the initial easy3d::SurfaceMesh for the Virtuoso arm that will be updated throughout the simulation */
    void _generateInitialMesh();

    /** Updates the entire mesh given the current state of the Virtuoso arm. */
    void _updateMesh();

    private:
    constexpr static int _OT_TUBULAR_RES = 20;
    constexpr static int _IT_TUBULAR_RES = 20;
    constexpr static int _TT_TUBULAR_RES = 20;
    
    easy3d::SurfaceMesh _e3d_mesh;

};

} // namespace Graphics

#endif // __EASY_3D_VIRTUOSO_ARM_GRAPHICS_OBJECT_HPP