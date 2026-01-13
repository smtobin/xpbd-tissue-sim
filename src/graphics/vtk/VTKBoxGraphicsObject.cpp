#include "graphics/vtk/VTKBoxGraphicsObject.hpp"

#include "graphics/vtk/VTKUtils.hpp"

#include <vtkPolyDataMapper.h>
#include <vtkTransform.h>

namespace Graphics
{

VTKBoxGraphicsObject::VTKBoxGraphicsObject(const std::string& name, const Sim::RigidBox* box, const Config::ObjectRenderConfig& render_config)
    : BoxGraphicsObject(name, box)
{
    // create the vtkActor from a box source
    Vec3r box_size = box->size();
    _cube_source = vtkSmartPointer<vtkCubeSource>::New();
    _cube_source->SetXLength(box_size[0]);
    _cube_source->SetYLength(box_size[1]);
    _cube_source->SetZLength(box_size[2]);
    // cube_source->SetCenter(box_loc[0], box_loc[1], box_loc[2]);
    
    vtkNew<vtkPolyDataMapper> data_mapper;
    if (render_config.smoothNormals())
    {
        // smooth normals
        vtkNew<vtkPolyDataNormals> normal_generator;
        normal_generator->SetInputConnection(_cube_source->GetOutputPort());
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
        data_mapper->SetInputConnection(_cube_source->GetOutputPort());
    }

    _box_actor = vtkSmartPointer<vtkActor>::New();
    _box_actor->SetMapper(data_mapper);

    // set up rendering from render config
    VTKUtils::setupActorFromRenderConfig(_box_actor.Get(), render_config);

    _vtk_transform = vtkSmartPointer<vtkTransform>::New();

    // IMPORTANT: use row-major ordering since that is what VTKTransform expects (default for Eigen is col-major)
    Eigen::Matrix<Real, 4, 4, Eigen::RowMajor> box_transform_mat = _box->transform().asMatrix();
    _vtk_transform->SetMatrix(box_transform_mat.data());

    _box_actor->SetUserTransform(_vtk_transform);
}

void VTKBoxGraphicsObject::updateGraphicsBuffers()
{
    RenderInfo* rinfo = _latest_rinfo.load(std::memory_order_acquire);

    // IMPORTANT: use row-major ordering since that is what VTKTransform expects (default for Eigen is col-major)
    Eigen::Matrix<Real, 4, 4, Eigen::RowMajor> box_transform_mat = rinfo->transform.asMatrix();
    _vtk_transform->SetMatrix(box_transform_mat.data());
}

} // namespace Graphics