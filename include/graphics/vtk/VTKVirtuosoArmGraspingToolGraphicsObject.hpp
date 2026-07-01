#ifndef __VTK_VIRTUOSO_ARM_GRASPING_TOOL_GRAPHICS_OBJECT
#define __VTK_VIRTUOSO_ARM_GRASPING_TOOL_GRAPHICS_OBJECT

#include "graphics/VirtuosoArmGraspingToolGraphicsObject.hpp"

#include "config/render/ObjectRenderConfig.hpp"

#include <vtkSmartPointer.h>
#include <vtkTransform.h>
#include <vtkActor.h>


#include <vtkLight.h>

namespace Graphics
{

class VTKVirtuosoArmGraspingToolGraphicsObject : public VirtuosoArmGraspingToolGraphicsObject
{
    public:
    explicit VTKVirtuosoArmGraspingToolGraphicsObject(const std::string& name, const Sim::VirtuosoArmGraspingTool* grasper, const Config::ObjectRenderConfig& render_config);

    virtual void updateGraphicsBuffers() override;

    vtkSmartPointer<vtkActor> graspCenterActor() { return _grasp_center_actor; }

    private:
    vtkSmartPointer<vtkTransform> _grasp_center_transform;   // transform for the grasping center
    vtkSmartPointer<vtkActor> _grasp_center_actor;           // VTK actor for the sphere
};

} // namespace Graphics


#endif