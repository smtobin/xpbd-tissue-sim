#ifndef __VTK_MESH_GRAPHICS_OBJECT_HPP
#define __VTK_MESH_GRAPHICS_OBJECT_HPP

#include "graphics/MeshGraphicsObject.hpp"

#include "config/render/ObjectRenderConfig.hpp"

#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkPolyDataNormals.h>
#include <vtkExtractEdges.h>
#include <vtkSmartPointer.h>
#include <vtkActor.h>

namespace Graphics
{

class VTKMeshGraphicsObject : public MeshGraphicsObject
{
    public:
    explicit VTKMeshGraphicsObject(const std::string& name, const Geometry::Mesh* mesh, const Config::ObjectRenderConfig& render_config);

    virtual void updateGraphicsBuffers() override;

    vtkSmartPointer<vtkActor> facesActor() { return _faces_vtk_actor; }
    vtkSmartPointer<vtkActor> edgesActor() { return _edges_vtk_actor; }

    private:
    void _setVertices(const RenderInfo* rmesh);
    void _setFaces(const RenderInfo* rmesh);
    void _setColors(const RenderInfo* rmesh);

    unsigned long _latest_topology_version = 0;

    vtkSmartPointer<vtkPolyData> _vtk_poly_data;

    vtkSmartPointer<vtkPolyData> _front_poly_data;
    
    vtkSmartPointer<vtkActor> _faces_vtk_actor;
    vtkSmartPointer<vtkActor> _edges_vtk_actor;

    vtkSmartPointer<vtkExtractEdges> _edge_extractor;
    vtkSmartPointer<vtkPolyDataMapper> _edge_mapper;

    vtkSmartPointer<vtkPolyDataMapper> _face_mapper;
    vtkSmartPointer<vtkPolyDataNormals> _normals_generator;
};

} // namespace Graphics

#endif // __VTK_MESH_GRAPHICS_OBJECT_HPP