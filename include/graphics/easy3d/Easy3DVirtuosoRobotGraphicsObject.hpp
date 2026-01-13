#ifndef __EASY3D_VIRTUOSO_ROBOT_GRAPHICS_OBJECT_HPP
#define __EASY3D_VIRTUOSO_ROBOT_GRAPHICS_OBJECT_HPP

#include "graphics/VirtuosoRobotGraphicsObject.hpp"

#include <easy3d/core/surface_mesh.h>
#include <easy3d/core/model.h>

namespace Graphics
{

class Easy3DVirtuosoRobotGraphicsObject : public VirtuosoRobotGraphicsObject, public easy3d::Model
{
    public:
    explicit Easy3DVirtuosoRobotGraphicsObject(const std::string& name, const Sim::VirtuosoRobot* virtuoso_robot);

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

    /** Updates the entire mesh given the current state of the Virtuoso arm. */
    void _updateMesh();

    private:
    
    easy3d::SurfaceMesh _e3d_mesh;
    

};

} // namespace Graphics

#endif // __EASY3D_VIRTUOSO_ROBOT_GRAPHICS_OBJECT_HPP