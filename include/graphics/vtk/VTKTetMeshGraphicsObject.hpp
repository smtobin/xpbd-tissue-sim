#ifndef __VTK_TET_MESH_GRAPHICS_OBJECT_HPP
#define __VTK_TET_MESH_GRAPHICS_OBJECT_HPP

#include "graphics/TetMeshGraphicsObject.hpp"

#include "config/render/ObjectRenderConfig.hpp"

#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkPolyDataNormals.h>
#include <vtkExtractEdges.h>
#include <vtkSmartPointer.h>
#include <vtkActor.h>

namespace Graphics
{

class VTKTetMeshGraphicsObject : public TetMeshGraphicsObject
{
    public:
    explicit VTKTetMeshGraphicsObject(const std::string& name, const Geometry::TetMesh* mesh, const Config::ObjectRenderConfig& render_config);

    virtual void updateGraphicsBuffers() override;

    vtkSmartPointer<vtkActor> facesActor() { return _faces_vtk_actor; }
    vtkSmartPointer<vtkActor> edgesActor() { return _edges_vtk_actor; }

    private:

    void _setNormals(const RenderInfo* rmesh);

    void _setVertices(const RenderInfo* rmesh);
    void _setFaces(const RenderInfo* rmesh);

    void _setColorsForTemperature(const RenderInfo* rmesh);
    void _setColorsForCutSurface(const RenderInfo* rmesh);

    unsigned long _latest_topology_version = 0;

    vtkSmartPointer<vtkPolyData> _vtk_poly_data;

    vtkSmartPointer<vtkPolyData> _front_poly_data;
    
    vtkSmartPointer<vtkActor> _faces_vtk_actor;
    vtkSmartPointer<vtkActor> _edges_vtk_actor;

    vtkSmartPointer<vtkExtractEdges> _edge_extractor;
    vtkSmartPointer<vtkPolyDataMapper> _edge_mapper;

    vtkSmartPointer<vtkPolyDataMapper> _face_mapper;
    // vtkSmartPointer<vtkPolyDataNormals> _normals_generator;

    /** When a mesh has multiple 'classes', multiple colors can be specified for different parts of the mesh. */
    std::optional<std::vector<Vec3r>> _colors;

    /** The default, bulk color of the mesh. Set by the render config. */
    Vec3r _bulk_color;

    /** The color of cut portions of the mesh. Set by the render config. */
    Vec3r _cut_color;

    /** Whether or not to smooth normals. */
    bool _smooth_normals;
};

} // namespace Graphics

#endif // __VTK_MESH_GRAPHICS_OBJECT_HPP