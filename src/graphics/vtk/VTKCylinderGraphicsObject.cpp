#include "graphics/vtk/VTKCylinderGraphicsObject.hpp"
#include "graphics/vtk/VTKUtils.hpp"

#include <vtkPolyDataMapper.h>

namespace Graphics
{

VTKCylinderGraphicsObject::VTKCylinderGraphicsObject(const std::string& name, const Sim::RigidCylinder* cyl, const Config::ObjectRenderConfig& render_config)
    : CylinderGraphicsObject(name, cyl)
{
    // create the vtkActor from a cylinder source
    _cyl_source = vtkSmartPointer<vtkCylinderSource>::New();
    _cyl_source->SetHeight(cyl->height());
    _cyl_source->SetRadius(cyl->radius());
    
    vtkNew<vtkPolyDataMapper> data_mapper;
    if (render_config.smoothNormals())
    {
        // smooth normals
        vtkNew<vtkPolyDataNormals> normal_generator;
        normal_generator->SetInputConnection(_cyl_source->GetOutputPort());
        normal_generator->SetFeatureAngle(30.0);
        normal_generator->SplittingOff();
        normal_generator->ConsistencyOn();
        normal_generator->ComputePointNormalsOn();
        normal_generator->ComputeCellNormalsOff();
        normal_generator->Update();

        data_mapper->SetInputConnection(normal_generator->GetOutputPort());
    }
    else
    {
        data_mapper->SetInputConnection(_cyl_source->GetOutputPort());
    }

    _cyl_actor = vtkSmartPointer<vtkActor>::New();
    _cyl_actor->SetMapper(data_mapper);

    // set up rendering from render config
    VTKUtils::setupActorFromRenderConfig(_cyl_actor.Get(), render_config);

    _vtk_transform = vtkSmartPointer<vtkTransform>::New();

    // IMPORTANT: use row-major ordering since that is what VTKTransform expects (default for Eigen is col-major)
    Eigen::Matrix<Real, 4, 4, Eigen::RowMajor> cyl_transform_mat = cyl->transform().asMatrix();
    _vtk_transform->SetMatrix(cyl_transform_mat.data());

    // vtkCylinderSource creates a cylinder along the y-axis, but we expect the cylinder to be along the z-axis
    // hence we need to first rotate the cylinder provided by vtkCylinderSource by -90 deg about the x-axis
    _vtk_transform->PreMultiply();
    _vtk_transform->RotateX(-90);

    _cyl_actor->SetUserTransform(_vtk_transform);
}

void VTKCylinderGraphicsObject::updateGraphicsBuffers()
{
    RenderInfo* rinfo = _latest_rinfo.load(std::memory_order_acquire);
    
    // IMPORTANT: use row-major ordering since that is what VTKTransform expects (default for Eigen is col-major)
    Eigen::Matrix<Real, 4, 4, Eigen::RowMajor> cyl_transform_mat = rinfo->transform.asMatrix();
    _vtk_transform->SetMatrix(cyl_transform_mat.data());

    // vtkCylinderSource creates a cylinder along the y-axis, but we expect the cylinder to be along the z-axis
    // hence we need to first rotate the cylinder provided by vtkCylinderSource by -90 deg about the x-axis
    _vtk_transform->PreMultiply();
    _vtk_transform->RotateX(-90);
}

} // namespace Graphics