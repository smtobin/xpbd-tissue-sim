#ifndef __VTK_VIRTUOSO_ARM_GRAPHICS_OBJECT
#define __VTK_VIRTUOSO_ARM_GRAPHICS_OBJECT

#include "graphics/VirtuosoArmGraphicsObject.hpp"

#include "config/render/ObjectRenderConfig.hpp"

#include <vtkPolyData.h>
#include <vtkSmartPointer.h>
#include <vtkActor.h>

namespace Graphics
{

class VTKVirtuosoArmGraphicsObject : public VirtuosoArmGraphicsObject
{
    public:
    explicit VTKVirtuosoArmGraphicsObject(const std::string& name, const Sim::VirtuosoArm* arm, const Config::ObjectRenderConfig& render_config);

    virtual void updateGraphicsBuffers() override;

    vtkSmartPointer<vtkActor> actor() { return _vtk_actor; }

    private:
    void _generateInitialPolyData();
    void _updatePolyData();

    constexpr static int _OT_TUBULAR_RES = 20;
    constexpr static int _IT_TUBULAR_RES = 20;
    constexpr static int _TT_TUBULAR_RES = 20;

    vtkSmartPointer<vtkPolyData> _vtk_poly_data;
    vtkSmartPointer<vtkActor> _vtk_actor;           // VTK actor for the sphere
};

} // namespace Graphics

#endif // __VTK_VIRTUOSO_ARM_GRAPHICS_OBJECT