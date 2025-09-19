#ifndef __VTK_MESH_GRAPHICS_OBJECT_HPP
#define __VTK_MESH_GRAPHICS_OBJECT_HPP

#include "graphics/MeshGraphicsObject.hpp"

#include "config/render/ObjectRenderConfig.hpp"

#include <vtkPolyData.h>
#include <vtkSmartPointer.h>
#include <vtkActor.h>

namespace Graphics
{

class VTKMeshGraphicsObject : public MeshGraphicsObject
{
    public:
    explicit VTKMeshGraphicsObject(const std::string& name, const Geometry::Mesh* mesh, const Config::ObjectRenderConfig& render_config);

    virtual void update() override;

    vtkSmartPointer<vtkActor> actor() { return _vtk_actor; }

    private:
    void _setVertices();
    void _setFaces();

    vtkSmartPointer<vtkPolyData> _vtk_poly_data;
    vtkSmartPointer<vtkActor> _vtk_actor;
};

} // namespace Graphics

#endif // __VTK_MESH_GRAPHICS_OBJECT_HPP