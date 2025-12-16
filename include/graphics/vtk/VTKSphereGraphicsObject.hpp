#ifndef __VTK_SPHERE_GRAPHICS_OBJECT_HPP
#define __VTK_SPHERE_GRAPHICS_OBJECT_HPP

#include "graphics/SphereGraphicsObject.hpp"

#include "config/render/ObjectRenderConfig.hpp"

#include <vtkSphereSource.h>
#include <vtkActor.h>
#include <vtkTransform.h>

namespace Graphics
{

class VTKSphereGraphicsObject : public SphereGraphicsObject
{
    public:
    explicit VTKSphereGraphicsObject(const std::string& name, const Sim::RigidSphere* sphere, const Config::ObjectRenderConfig& render_config);

    virtual void updateGraphicsBuffers() override;

    vtkSmartPointer<vtkActor> actor() { return _sphere_actor; }

    private:
    vtkSmartPointer<vtkSphereSource> _sphere_source;    // sphere source algorithm that generates box mesh (keep a reference to it so we can potentially change it's size)
    vtkSmartPointer<vtkActor> _sphere_actor;           // VTK actor for the sphere
    vtkSmartPointer<vtkTransform> _vtk_transform;   // transform for the actor (updated according to position and orientation of sphere)
};

} // namespace Graphics

#endif // __VTK_SPHERE_GRAPHICS_OBJECT_HPP