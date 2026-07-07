#include "graphics/TetMeshGraphicsObject.hpp"

namespace Graphics {

TetMeshGraphicsObject::TetMeshGraphicsObject(const std::string& name, const Geometry::TetMesh* mesh)
    : GraphicsObject(name), _mesh(mesh), _latest_rmesh(&_rmesh1)
{
    _draw_faces = true;
    _draw_points = false;
    _draw_edges = true;
    update();
}

TetMeshGraphicsObject::TetMeshGraphicsObject(const std::string& name, const Geometry::TetMesh* mesh, const Config::MeshObjectConfig* mesh_object_config)
    : GraphicsObject(name), _mesh(mesh), _latest_rmesh(&_rmesh1)
{
    _draw_faces = mesh_object_config->drawFaces();
    _draw_edges = mesh_object_config->drawEdges();
    _draw_points = mesh_object_config->drawPoints();

    _color = mesh_object_config->color();
}

TetMeshGraphicsObject::~TetMeshGraphicsObject()
{

}

void TetMeshGraphicsObject::update()
{
    RenderInfo* write_mesh = (_latest_rmesh.load() == &_rmesh1) ? &_rmesh2 : &_rmesh1;

    write_mesh->vertices = _mesh->vertices();
    write_mesh->faces = _mesh->faces();
    write_mesh->vertex_normals = _mesh->vertexNormals();
    write_mesh->vertex_properties = _mesh->vertexProperties();
    write_mesh->face_properties = _mesh->faceProperties();
    write_mesh->element_properties = _mesh->elementProperties();
    write_mesh->topology_version = _mesh->topologyVersion();

    // get the embedded submeshes
    /** TODO: for now we just hard-code a single additional element class */
    const auto [sub_vertices, sub_faces, sub_elements] = _mesh->submeshForElementClass(1);
    write_mesh->interior_faces.resize(1);
    write_mesh->interior_faces[0] = sub_faces;
    Geometry::Mesh::computeVertexNormals(_mesh->vertices(), sub_faces, write_mesh->vertex_normals);

    _latest_rmesh.store(write_mesh, std::memory_order_release);
}

} // namespace Graphics