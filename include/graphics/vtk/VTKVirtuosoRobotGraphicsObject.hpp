#ifndef __VTK_VIRTUOSO_ROBOT_GRAPHICS_OBJECT_HPP
#define __VTK_VIRTUOSO_ROBOT_GRAPHICS_OBJECT_HPP

#include "graphics/VirtuosoRobotGraphicsObject.hpp"

#include "config/render/ObjectRenderConfig.hpp"

#include <vtkSmartPointer.h>
#include <vtkTransform.h>
#include <vtkActor.h>

#include <vtkLight.h>

namespace Graphics
{

class VTKVirtuosoRobotGraphicsObject : public VirtuosoRobotGraphicsObject
{
    public:
    explicit VTKVirtuosoRobotGraphicsObject(const std::string& name, const Sim::VirtuosoRobot* robot, const Config::ObjectRenderConfig& render_config);

    virtual void updateGraphicsBuffers() override;

    vtkSmartPointer<vtkActor> actor() { return _vtk_actor; }

    vtkSmartPointer<vtkLight> endoscopeLight() { return _endoscope_light; }

    private:
    vtkSmartPointer<vtkActor> _vtk_actor;           // VTK actor for the robot
    vtkSmartPointer<vtkTransform> _vtk_transform;   // transform for the actor (updated according to position and orientation of robot)

    vtkSmartPointer<vtkLight> _endoscope_light;
};

} // namespace Graphics

#endif // __VTK_VIRTUOSO_ROBOT_GRAPHICS_OBJECT_HPP