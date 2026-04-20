#include "graphics/vtk/VTKVirtuosoArmCauteryToolGraphicsObject.hpp"
#include "graphics/vtk/VTKUtils.hpp"

#include <vtkCylinderSource.h>
#include <vtkPolyDataMapper.h>
#include <vtkLinearExtrusionFilter.h>
#include <vtkTransformPolyDataFilter.h>

namespace Graphics
{

VTKVirtuosoArmCauteryToolGraphicsObject::VTKVirtuosoArmCauteryToolGraphicsObject(const std::string& name, const Sim::VirtuosoArmCauteryTool* cautery, const Config::ObjectRenderConfig& render_config)
    : VirtuosoArmCauteryToolGraphicsObject(name, cautery)
{
    // create the vtkActors from a cylinder source
    vtkNew<vtkCylinderSource> ceramic_cyl_source;
    ceramic_cyl_source->SetHeight(Sim::VirtuosoArmCauteryTool::CERAMIC_LENGTH);
    ceramic_cyl_source->SetRadius(Sim::VirtuosoArmCauteryTool::CERAMIC_DIA/2);
    ceramic_cyl_source->SetResolution(32);

    vtkNew<vtkPolyDataMapper> ceramic_mapper;
    if (render_config.smoothNormals())
    {
        // smooth normals
        vtkNew<vtkPolyDataNormals> normal_generator;
        normal_generator->SetInputConnection(ceramic_cyl_source->GetOutputPort());
        normal_generator->SetFeatureAngle(30.0);
        normal_generator->SplittingOff();
        normal_generator->ConsistencyOn();
        normal_generator->ComputePointNormalsOn();
        normal_generator->ComputeCellNormalsOff();
        normal_generator->Update();

        ceramic_mapper->SetInputConnection(normal_generator->GetOutputPort());
    }
    else
    {
        ceramic_mapper->SetInputConnection(ceramic_cyl_source->GetOutputPort());
    }

    _ceramic_actor = vtkSmartPointer<vtkActor>::New();
    _ceramic_actor->SetMapper(ceramic_mapper);

    vtkNew<vtkCylinderSource> wire_cyl_source;
    wire_cyl_source->SetHeight(Sim::VirtuosoArmCauteryTool::WIRE_LENGTH);
    wire_cyl_source->SetRadius(Sim::VirtuosoArmCauteryTool::WIRE_DIA/2);
    wire_cyl_source->SetResolution(32);

    vtkNew<vtkPolyDataMapper> wire_mapper;
    if (render_config.smoothNormals())
    {
        // smooth normals
        vtkNew<vtkPolyDataNormals> normal_generator;
        normal_generator->SetInputConnection(wire_cyl_source->GetOutputPort());
        normal_generator->SetFeatureAngle(30.0);
        normal_generator->SplittingOff();
        normal_generator->ConsistencyOn();
        normal_generator->ComputePointNormalsOn();
        normal_generator->ComputeCellNormalsOff();
        normal_generator->Update();

        wire_mapper->SetInputConnection(normal_generator->GetOutputPort());
    }
    else
    {
        wire_mapper->SetInputConnection(wire_cyl_source->GetOutputPort());
    }

    _wire_actor = vtkSmartPointer<vtkActor>::New();
    _wire_actor->SetMapper(wire_mapper);

    Config::ObjectRenderConfig ceramic_render_config(
        Config::ObjectRenderConfig::RenderType::PBR,
        std::nullopt, std::nullopt, std::nullopt,
        0.0, 0.5, 1.0, Vec3r(0.9, 0.9, 0.9),
        true, true, false, false
    );

    // set up rendering from render config
    VTKUtils::setupActorFromRenderConfig(_ceramic_actor.Get(), ceramic_render_config);
    VTKUtils::setupActorFromRenderConfig(_wire_actor.Get(), render_config);


    // create transforms
    _ceramic_transform = vtkSmartPointer<vtkTransform>::New();
    _wire_transform = vtkSmartPointer<vtkTransform>::New();

    // IMPORTANT: use row-major ordering since that is what VTKTransform expects (default for Eigen is col-major)
    Eigen::Matrix<Real, 4, 4, Eigen::RowMajor> ceramic_transform_mat = cautery->ceramicFrame().transform().asMatrix();
    Eigen::Matrix<Real, 4, 4, Eigen::RowMajor> wire_transform_mat = cautery->wireFrame().transform().asMatrix();
    _ceramic_transform->SetMatrix(ceramic_transform_mat.data());
    _wire_transform->SetMatrix(wire_transform_mat.data());

    // vtkCylinderSource creates a cylinder along the y-axis, but we expect the cylinder to be along the z-axis
    // hence we need to first rotate the cylinder provided by vtkCylinderSource by -90 deg about the x-axis
    _ceramic_transform->PreMultiply();
    _ceramic_transform->RotateX(-90);

    _wire_transform->PreMultiply();
    _wire_transform->RotateX(-90);

    _ceramic_actor->SetUserTransform(_ceramic_transform);
    _wire_actor->SetUserTransform(_wire_transform);
}

void VTKVirtuosoArmCauteryToolGraphicsObject::updateGraphicsBuffers()
{
    RenderInfo* rinfo = _latest_rinfo.load(std::memory_order_acquire);

    Geometry::TransformationMatrix ceramic = rinfo->ceramic_frame.transform();
    Geometry::TransformationMatrix wire = rinfo->wire_frame.transform();

    // IMPORTANT: use row-major ordering since that is what VTKTransform expects (default for Eigen is col-major)
    Eigen::Matrix<Real, 4, 4, Eigen::RowMajor> ceramic_transform_mat = ceramic.asMatrix();
    Eigen::Matrix<Real, 4, 4, Eigen::RowMajor> wire_transform_mat = wire.asMatrix();
    _ceramic_transform->SetMatrix(ceramic_transform_mat.data());
    _wire_transform->SetMatrix(wire_transform_mat.data());

    // vtkCylinderSource creates a cylinder along the y-axis, but we expect the cylinder to be along the z-axis
    // hence we need to first rotate the cylinder provided by vtkCylinderSource by -90 deg about the x-axis
    _ceramic_transform->PreMultiply();
    _ceramic_transform->RotateX(-90);

    _wire_transform->PreMultiply();
    _wire_transform->RotateX(-90);

    _ceramic_actor->SetUserTransform(_ceramic_transform);
    _wire_actor->SetUserTransform(_wire_transform);
}

} // namespace Graphics