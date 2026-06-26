#include "graphics/vtk/VTKVirtuosoArmGraspingToolGraphicsObject.hpp"
#include "graphics/vtk/VTKUtils.hpp"

#include <vtkCylinderSource.h>
#include <vtkPolyDataMapper.h>
#include <vtkLinearExtrusionFilter.h>
#include <vtkTransformPolyDataFilter.h>
#include <vtkSphereSource.h>

namespace Graphics
{

VTKVirtuosoArmGraspingToolGraphicsObject::VTKVirtuosoArmGraspingToolGraphicsObject(const std::string& name, const Sim::VirtuosoArmGraspingTool* grasper, const Config::ObjectRenderConfig& render_config)
    : VirtuosoArmGraspingToolGraphicsObject(name, grasper)
{
    // create the vtkActors from a cylinder source
    vtkNew<vtkSphereSource> grasp_center_sphere_source;
    grasp_center_sphere_source->SetRadius(Sim::VirtuosoArmGraspingTool::GRASPING_RADIUS);
    grasp_center_sphere_source->SetPhiResolution(10);
    grasp_center_sphere_source->SetThetaResolution(20);

    vtkNew<vtkPolyDataMapper> grasp_center_mapper;
    if (render_config.smoothNormals())
    {
        // smooth normals
        vtkNew<vtkPolyDataNormals> normal_generator;
        normal_generator->SetInputConnection(grasp_center_sphere_source->GetOutputPort());
        normal_generator->SetFeatureAngle(30.0);
        normal_generator->SplittingOff();
        normal_generator->ConsistencyOn();
        normal_generator->ComputePointNormalsOn();
        normal_generator->ComputeCellNormalsOff();
        normal_generator->Update();

        grasp_center_mapper->SetInputConnection(normal_generator->GetOutputPort());
    }
    else
    {
        grasp_center_mapper->SetInputConnection(grasp_center_sphere_source->GetOutputPort());
    }

    _grasp_center_actor = vtkSmartPointer<vtkActor>::New();
    _grasp_center_actor->SetMapper(grasp_center_mapper);

    

    Config::ObjectRenderConfig grasp_center_render_config(
        Config::ObjectRenderConfig::RenderType::PBR,
        std::nullopt, std::nullopt, std::nullopt,
        0.0, 0.5, 0.1, Vec3r(1.0, 1.0, 0.0),
        true, true, false, false
    );

    // set up rendering from render config
    VTKUtils::setupActorFromRenderConfig(_grasp_center_actor.Get(), grasp_center_render_config);


    // create transforms
    _grasp_center_transform = vtkSmartPointer<vtkTransform>::New();

    // IMPORTANT: use row-major ordering since that is what VTKTransform expects (default for Eigen is col-major)
    Eigen::Matrix<Real, 4, 4, Eigen::RowMajor> grasp_transform_mat = grasper->graspFrame().transform().asMatrix();
    _grasp_center_transform->SetMatrix(grasp_transform_mat.data());

    _grasp_center_actor->SetUserTransform(_grasp_center_transform);
}

void VTKVirtuosoArmGraspingToolGraphicsObject::updateGraphicsBuffers()
{
    RenderInfo* rinfo = _latest_rinfo.load(std::memory_order_acquire);

    Geometry::TransformationMatrix grasp = rinfo->grasp_frame.transform();

    // IMPORTANT: use row-major ordering since that is what VTKTransform expects (default for Eigen is col-major)
    Eigen::Matrix<Real, 4, 4, Eigen::RowMajor> grasp_transform_mat = grasp.asMatrix();
    _grasp_center_transform->SetMatrix(grasp_transform_mat.data());


    // _grasp_center_actor->SetUserTransform(_grasp_center_transform);
}

} // namespace Graphics