#include "graphics/vtk/VTKSphereGraphicsObject.hpp"
#include "graphics/vtk/VTKUtils.hpp"

#include <vtkPolyDataMapper.h>
#include <vtkMatrix4x4.h>

namespace Graphics
{

VTKSphereGraphicsObject::VTKSphereGraphicsObject(const std::string& name, const Sim::RigidSphere* sphere, const Config::ObjectRenderConfig& render_config)
    : SphereGraphicsObject(name, sphere)
{
    // create the vtkActor from a sphere source
    _sphere_source = vtkSmartPointer<vtkSphereSource>::New();
    _sphere_source->SetRadius(sphere->radius());
    _sphere_source->SetThetaResolution(20);
    _sphere_source->SetPhiResolution(20);

    
    vtkNew<vtkPolyDataMapper> data_mapper;

    if (render_config.smoothNormals())
    {
        // smooth normals
        vtkNew<vtkPolyDataNormals> normal_generator;
        normal_generator->SetInputConnection(_sphere_source->GetOutputPort());
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
        data_mapper->SetInputConnection(_sphere_source->GetOutputPort());
    }

    _sphere_actor = vtkSmartPointer<vtkActor>::New();
    _sphere_actor->SetMapper(data_mapper);

    VTKUtils::setupActorFromRenderConfig(_sphere_actor.Get(), render_config);

    _vtk_transform = vtkSmartPointer<vtkTransform>::New();

    // IMPORTANT: use row-major ordering since that is what VTKTransform expects (default for Eigen is col-major)
    Eigen::Matrix<Real, 4, 4, Eigen::RowMajor> sphere_transform_mat = sphere->transform().asMatrix();
    _vtk_transform->SetMatrix(sphere_transform_mat.data());

    _sphere_actor->SetUserTransform(_vtk_transform);
}

void VTKSphereGraphicsObject::updateGraphicsBuffers()
{
    RenderInfo* rinfo = _latest_rinfo.load(std::memory_order_acquire);

    // IMPORTANT: use row-major ordering since that is what VTKTransform expects (default for Eigen is col-major)
    Eigen::Matrix<Real, 4, 4, Eigen::RowMajor> sphere_transform_mat = rinfo->transform.asMatrix();
    _vtk_transform->SetMatrix(sphere_transform_mat.data());
}

} // namespace Graphics