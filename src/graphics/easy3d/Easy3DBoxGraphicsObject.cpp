#include "graphics/easy3d/Easy3DBoxGraphicsObject.hpp"

#include <easy3d/algo/surface_mesh_factory.h>
#include <easy3d/renderer/renderer.h>

namespace Graphics
{

Easy3DBoxGraphicsObject::Easy3DBoxGraphicsObject(const std::string& name, const Sim::RigidBox* box)
    : BoxGraphicsObject(name, box)
{
    _e3d_mesh = easy3d::SurfaceMeshFactory::hexahedron();
    for (auto& p : _e3d_mesh.points())
    {
        // for some reason, when Easy3D creates a hexahedron primitive, it is a cube centered at the origin with length of diagonal = 1,
        // meaning that the side length is 2/sqrt(3). To scale it to be a unit cube, we multiply each vertex by sqrt(3)/2, and then scale
        // it to the size that we want.
        p[0] *= std::sqrt(3)/2.0 * box->size()[0];
        p[1] *= std::sqrt(3)/2.0 * box->size()[1];
        p[2] *= std::sqrt(3)/2.0 * box->size()[2];
    }

    _initial_points = _e3d_mesh.points();

    _transformPoints();

    std::shared_ptr<easy3d::Renderer> renderer = std::make_shared<easy3d::Renderer>(&_e3d_mesh, true);
    _e3d_mesh.set_renderer(renderer);
    set_renderer(renderer);
}

void Easy3DBoxGraphicsObject::updateGraphicsBuffers() 
{
    _transformPoints();    
    renderer()->update();
}

void Easy3DBoxGraphicsObject::_transformPoints()
{
    std::vector<easy3d::vec3>& mesh_points = _e3d_mesh.points();
    const easy3d::vec3 e3d_position(_box->position()[0], _box->position()[1], _box->position()[2]);

    const Vec4r& quat = _box->orientation().normalized();
    const easy3d::Quat e3d_quat(static_cast<float>(quat[0]), static_cast<float>(quat[1]), static_cast<float>(quat[2]), static_cast<float>(quat[3]));
    const easy3d::mat3 e3d_rot_mat = easy3d::mat3::rotation(e3d_quat);
    for (size_t i = 0; i < mesh_points.size(); i++)
    {
        mesh_points[i] = e3d_rot_mat * _initial_points[i] + e3d_position;
    }
}


} // namespace Graphics