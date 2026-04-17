#ifndef __VTK_VIRTUOSO_ARM_SPATULA_TOOL_GRAPHICS_OBJECT
#define __VTK_VIRTUOSO_ARM_SPATULA_TOOL_GRAPHICS_OBJECT

#include "graphics/VirtuosoArmSpatulaToolGraphicsObject.hpp"

#include "config/render/ObjectRenderConfig.hpp"

#include <vtkSmartPointer.h>
#include <vtkTransform.h>
#include <vtkActor.h>

#include <vtkLight.h>

namespace Graphics
{

class VTKVirtuosoArmSpatulaToolGraphicsObject : public VirtuosoArmSpatulaToolGraphicsObject
{
    public:
    explicit VTKVirtuosoArmSpatulaToolGraphicsObject(const std::string& name, const Sim::VirtuosoArmSpatulaTool* spatula, const Config::ObjectRenderConfig& render_config);

    virtual void updateGraphicsBuffers() override;

    vtkSmartPointer<vtkActor> actor() { return _vtk_actor; }

    private:
    vtkSmartPointer<vtkActor> _vtk_actor;           // VTK actor for the robot
    vtkSmartPointer<vtkTransform> _vtk_transform;   // transform for the actor (updated according to position and orientation of robot)
};

} // namespace Graphics


#endif // __VTK_VIRTUOSO_ARM_SPATULA_TOOL_GRAPHICS_OBJECT