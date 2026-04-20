#ifndef __VTK_VIRTUOSO_ARM_CAUTERY_TOOL_GRAPHICS_OBJECT
#define __VTK_VIRTUOSO_ARM_CAUTERY_TOOL_GRAPHICS_OBJECT

#include "graphics/VirtuosoArmCauteryToolGraphicsObject.hpp"

#include "config/render/ObjectRenderConfig.hpp"

#include <vtkSmartPointer.h>
#include <vtkTransform.h>
#include <vtkActor.h>

#include <vtkLight.h>

namespace Graphics
{

class VTKVirtuosoArmCauteryToolGraphicsObject : public VirtuosoArmCauteryToolGraphicsObject
{
    public:
    explicit VTKVirtuosoArmCauteryToolGraphicsObject(const std::string& name, const Sim::VirtuosoArmCauteryTool* cautery, const Config::ObjectRenderConfig& render_config);

    virtual void updateGraphicsBuffers() override;

    vtkSmartPointer<vtkActor> ceramicActor() { return _ceramic_actor; }
    vtkSmartPointer<vtkActor> wireActor() { return _wire_actor; }

    private:
    vtkSmartPointer<vtkActor> _ceramic_actor;           // VTK actor for the ceramic part of the cautery tool
    vtkSmartPointer<vtkActor> _wire_actor;              // VTK actor for the wire part of the cautery tool
    vtkSmartPointer<vtkTransform> _ceramic_transform;   // transform for the ceramic actor
    vtkSmartPointer<vtkTransform> _wire_transform;      // transform for the wire actor
};

} // namespace Graphics


#endif