#ifndef __VTK_MESH_GRAPHICS_OBJECT_HPP
#define __VTK_MESH_GRAPHICS_OBJECT_HPP

#include "graphics/MeshGraphicsObject.hpp"

#include "config/render/ObjectRenderConfig.hpp"

#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkPolyDataNormals.h>
#include <vtkSmartPointer.h>
#include <vtkActor.h>

namespace Graphics
{

class VTKMeshGraphicsObject : public MeshGraphicsObject
{
    public:
    explicit VTKMeshGraphicsObject(const std::string& name, const Geometry::Mesh* mesh, const Config::ObjectRenderConfig& render_config);

    virtual void update() override;

    vtkSmartPointer<vtkActor> facesActor() { return _faces_vtk_actor; }
    vtkSmartPointer<vtkActor> edgesActor() { return _edges_vtk_actor; }

    private:
    void _setVertices();
    void _setFaces();
    void _setColors();

    vtkSmartPointer<vtkPolyData> _vtk_poly_data;
    vtkSmartPointer<vtkActor> _faces_vtk_actor;
    vtkSmartPointer<vtkActor> _edges_vtk_actor;

    vtkSmartPointer<vtkPolyDataMapper> _face_mapper;
    vtkSmartPointer<vtkPolyDataNormals> _normals_generator;
};

} // namespace Graphics

#endif // __VTK_MESH_GRAPHICS_OBJECT_HPP