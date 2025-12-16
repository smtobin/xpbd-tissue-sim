#ifndef __VTK_BOX_GRAPHICS_OBJECT_HPP
#define __VTK_BOX_GRAPHICS_OBJECT_HPP

#include "graphics/BoxGraphicsObject.hpp"

#include "config/render/ObjectRenderConfig.hpp"

#include <vtkActor.h>
#include <vtkCubeSource.h>
#include <vtkTransform.h>

namespace Graphics
{

class VTKBoxGraphicsObject : public BoxGraphicsObject
{
    public:
    explicit VTKBoxGraphicsObject(const std::string& name, const Sim::RigidBox* box, const Config::ObjectRenderConfig& render_config);

    virtual void updateGraphicsBuffers() override;

    vtkSmartPointer<vtkActor> actor() { return _box_actor; }

    private:
    vtkSmartPointer<vtkCubeSource> _cube_source;    // cube source algorithm that generates box mesh (keep a reference to it so we can potentially change it's size)
    vtkSmartPointer<vtkActor> _box_actor;           // VTK actor for the box
    vtkSmartPointer<vtkTransform> _vtk_transform;   // transform for the actor (updated according to position and orientation of box)
};



} // namespace Graphics

#endif // __VTK_BOX_GRAPHICS_OBJECT_HPP