#include "graphics/easy3d/Easy3DCylinderGraphicsObject.hpp"

#include <easy3d/algo/surface_mesh_factory.h>
#include <easy3d/renderer/renderer.h>

namespace Graphics
{

Easy3DCylinderGraphicsObject::Easy3DCylinderGraphicsObject(const std::string& name, const Sim::RigidCylinder* cyl)
    : CylinderGraphicsObject(name, cyl)
{
    _e3d_mesh = easy3d::SurfaceMeshFactory::cylinder(30, _cylinder->radius(), _cylinder->height());

    // translate the cylinder down so that it is centered about the origin 
    for (auto& p : _e3d_mesh.points())
    {
        p[2] -= _cylinder->height()/2.0;
    }
    _initial_points = _e3d_mesh.points();

    _transformPoints();

    std::shared_ptr<easy3d::Renderer> renderer = std::make_shared<easy3d::Renderer>(&_e3d_mesh, true);
    _e3d_mesh.set_renderer(renderer);
    set_renderer(renderer);
}

void Easy3DCylinderGraphicsObject::updateGraphicsBuffers() 
{
    _transformPoints();
    renderer()->update();
}

void Easy3DCylinderGraphicsObject::_transformPoints()
{
    std::vector<easy3d::vec3>& mesh_points = _e3d_mesh.points();
    const easy3d::vec3 e3d_position(_cylinder->position()[0], _cylinder->position()[1], _cylinder->position()[2]);

    const Vec4r& quat = _cylinder->orientation().normalized();
    const easy3d::Quat e3d_quat(static_cast<float>(quat[0]), static_cast<float>(quat[1]), static_cast<float>(quat[2]), static_cast<float>(quat[3]));
    const easy3d::mat3 e3d_rot_mat = easy3d::mat3::rotation(e3d_quat);
    for (size_t i = 0; i < mesh_points.size(); i++)
    {
        mesh_points[i] = e3d_rot_mat * _initial_points[i] + e3d_position;
    }
}


} // namespace Graphics