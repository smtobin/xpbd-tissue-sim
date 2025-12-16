#include "graphics/MeshGraphicsObject.hpp"

namespace Graphics {

MeshGraphicsObject::MeshGraphicsObject(const std::string& name, const Geometry::Mesh* mesh)
    : GraphicsObject(name), _mesh(mesh), _latest_rmesh(&_rmesh1)
{
    _draw_faces = true;
    _draw_points = false;
    _draw_edges = true;
}

MeshGraphicsObject::MeshGraphicsObject(const std::string& name, const Geometry::Mesh* mesh, const Config::MeshObjectConfig* mesh_object_config)
    : GraphicsObject(name), _mesh(mesh), _latest_rmesh(&_rmesh1)
{
    _draw_faces = mesh_object_config->drawFaces();
    _draw_edges = mesh_object_config->drawEdges();
    _draw_points = mesh_object_config->drawPoints();

    _color = mesh_object_config->color();
}

MeshGraphicsObject::~MeshGraphicsObject()
{

}

void MeshGraphicsObject::update()
{
    RenderInfo* write_mesh = (_latest_rmesh.load() == &_rmesh1) ? &_rmesh2 : &_rmesh1;

    write_mesh->vertices = _mesh->vertices();
    write_mesh->faces = _mesh->faces();

    _latest_rmesh.store(write_mesh, std::memory_order_release);
}

} // namespace Graphics