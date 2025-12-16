#ifndef __VTK_CYLINDER_GRAPHICS_OBJECT
#define __VTK_CYLINDER_GRAPHICS_OBJECT

#include "graphics/CylinderGraphicsObject.hpp"

#include "config/render/ObjectRenderConfig.hpp"

#include <vtkActor.h>
#include <vtkCylinderSource.h>
#include <vtkTransform.h>

namespace Graphics
{

class VTKCylinderGraphicsObject : public CylinderGraphicsObject
{
    public:
    explicit VTKCylinderGraphicsObject(const std::string& name, const Sim::RigidCylinder* cylinder, const Config::ObjectRenderConfig& render_config);

    virtual void updateGraphicsBuffers() override;

    vtkSmartPointer<vtkActor> actor() { return _cyl_actor; }

    private:
    vtkSmartPointer<vtkCylinderSource> _cyl_source;    // cylinder source algorithm that generates box mesh (keep a reference to it so we can potentially change it's size)
    vtkSmartPointer<vtkActor> _cyl_actor;           // VTK actor for the cylinder
    vtkSmartPointer<vtkTransform> _vtk_transform;   // transform for the actor (updated according to position and orientation of cylinder)
};

} // namespace Graphics

#endif // __VTK_CYLINDER_GRAPHICS_OBJECT