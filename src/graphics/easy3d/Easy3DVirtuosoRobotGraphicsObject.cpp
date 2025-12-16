#include "graphics/easy3d/Easy3DVirtuosoRobotGraphicsObject.hpp"

#include <easy3d/algo/surface_mesh_factory.h>
#include <easy3d/renderer/renderer.h>
#include <easy3d/renderer/drawable_points.h>
#include <easy3d/renderer/drawable_triangles.h>

namespace Graphics
{

Easy3DVirtuosoRobotGraphicsObject::Easy3DVirtuosoRobotGraphicsObject(const std::string& name, const Sim::VirtuosoRobot* virtuoso_robot)
    : VirtuosoRobotGraphicsObject(name, virtuoso_robot)
{
    _e3d_mesh = easy3d::SurfaceMeshFactory::cylinder(30, _virtuoso_robot->endoscopeDiameter()/2.0, _virtuoso_robot->endoscopeLength());
    const Geometry::TransformationMatrix& endoscope_transform = _virtuoso_robot->endoscopeFrame().transform();
    const Mat3r& rot_mat = endoscope_transform.rotMat();
    easy3d::Mat3<float> e3d_rot_mat;
    e3d_rot_mat(0,0) = rot_mat(0,0); e3d_rot_mat(0,1) = rot_mat(0,1); e3d_rot_mat(0,2) = rot_mat(0,2);
    e3d_rot_mat(1,0) = rot_mat(1,0); e3d_rot_mat(1,1) = rot_mat(1,1); e3d_rot_mat(1,2) = rot_mat(1,2);
    e3d_rot_mat(2,0) = rot_mat(2,0); e3d_rot_mat(2,1) = rot_mat(2,1); e3d_rot_mat(2,2) = rot_mat(2,2);

    for (auto& pt : _e3d_mesh.points())
    {
        pt[2] -= _virtuoso_robot->endoscopeLength();

        pt = e3d_rot_mat*pt;

        pt[0] += endoscope_transform.translation()[0];
        pt[1] += endoscope_transform.translation()[1];
        pt[2] += endoscope_transform.translation()[2];
    }

    std::shared_ptr<easy3d::Renderer> renderer = std::make_shared<easy3d::Renderer>(&_e3d_mesh, true);
    _e3d_mesh.set_renderer(renderer);

    // std::shared_ptr<easy3d::Renderer> renderer = std::make_shared<easy3d::Renderer>(this, true);

    // create a PointsDrawable for the points of the tetrahedral mesh
    // easy3d::PointsDrawable* points_drawable = renderer->add_points_drawable("vertices");
    // // specify the update function for the points
    // points_drawable->set_update_func([](easy3d::Model* m, easy3d::Drawable* d) {
    //     // update the vertex buffer with the vertices of the mesh
    //     d->update_vertex_buffer(m->points(), true);
    // });

    // set a uniform color for the mesh
    // easy3d::vec4 color(0, 0, 0, 0);
    // points_drawable->set_uniform_coloring(color);

    set_renderer(renderer);

    

    // color the mesh gray
    for (auto& tri_drawable : renderer->triangles_drawables())
    {
        easy3d::vec4 color(0.7, 0.7, 0.7, 1.0);
        tri_drawable->set_uniform_coloring(color);
    }
}

void Easy3DVirtuosoRobotGraphicsObject::updateGraphicsBuffers()
{

}

void Easy3DVirtuosoRobotGraphicsObject::_updateMesh()
{

}

} // namespace Graphics