#include "graphics/vtk/VTKVirtuosoRobotGraphicsObject.hpp"
#include "graphics/vtk/VTKUtils.hpp"

#include <vtkCylinderSource.h>
#include <vtkPolyDataMapper.h>

namespace Graphics
{

VTKVirtuosoRobotGraphicsObject::VTKVirtuosoRobotGraphicsObject(const std::string& name, const Sim::VirtuosoRobot* robot, const Config::ObjectRenderConfig& render_config)
    : VirtuosoRobotGraphicsObject(name, robot)
{
    // create the vtkActor from a cylinder source
    vtkNew<vtkCylinderSource> cyl_source;;
    cyl_source->SetHeight(robot->endoscopeLength());
    cyl_source->SetRadius(0.5*robot->endoscopeDiameter());
    cyl_source->SetResolution(20);
    
    vtkNew<vtkPolyDataMapper> data_mapper;
    if (render_config.smoothNormals())
    {
        // smooth normals
        vtkNew<vtkPolyDataNormals> normal_generator;
        normal_generator->SetInputConnection(cyl_source->GetOutputPort());
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
        data_mapper->SetInputConnection(cyl_source->GetOutputPort());
    }

    _vtk_actor = vtkSmartPointer<vtkActor>::New();
    _vtk_actor->SetMapper(data_mapper);

    // setup rendering based on render config
    VTKUtils::setupActorFromRenderConfig(_vtk_actor.Get(), render_config);

    _vtk_actor->GetProperty()->SetColor(0.0, 0.0, 0.0);
    _vtk_actor->GetProperty()->SetMetallic(0);
    _vtk_actor->GetProperty()->SetRoughness(0.3);

    // create the endoscope light attached to the end of the endoscope
    Vec3r light_pos = robot->camFrame().origin();
    Vec3r light_dir = robot->camFrame().transform().rotMat().col(2);
    Real focal_dist = 0.1;
    Vec3r focal_point = light_pos + light_dir * focal_dist;
    _endoscope_light = vtkSmartPointer<vtkLight>::New();
    _endoscope_light->SetLightTypeToSceneLight();
    _endoscope_light->SetPositional(true);
    _endoscope_light->SetPosition(light_pos[0], light_pos[1], light_pos[2]);
    _endoscope_light->SetFocalPoint(focal_point[0], focal_point[1], focal_point[2]);
    _endoscope_light->SetConeAngle(90);
    _endoscope_light->SetAttenuationValues(1,0,0);
    _endoscope_light->SetColor(1.0, 1.0, 1.0);
    _endoscope_light->SetIntensity(10.0);    /** TODO: make this a settable parameter somehow (and potentially other parameters of the lights) */

    _vtk_transform = vtkSmartPointer<vtkTransform>::New();

    Geometry::TransformationMatrix endoscope_transform = _virtuoso_robot->endoscopeFrame().transform();
    const Vec3r cyl_frame_pos = endoscope_transform.translation() - endoscope_transform.rotMat().col(2) * _virtuoso_robot->endoscopeLength()/2.0;

    Geometry::TransformationMatrix cyl_transform(endoscope_transform.rotMat(), cyl_frame_pos);
     // IMPORTANT: use row-major ordering since that is what VTKTransform expects (default for Eigen is col-major)
    Eigen::Matrix<Real, 4, 4, Eigen::RowMajor> transform_mat = cyl_transform.asMatrix();
    _vtk_transform->SetMatrix(transform_mat.data());

    // vtkCylinderSource creates a cylinder along the y-axis, but we expect the endoscope to be along the z-axis
    // hence we need to first rotate the cylinder provided by vtkCylinderSource by -90 deg about the x-axis
    _vtk_transform->PreMultiply();
    _vtk_transform->RotateX(-90);

    _vtk_actor->SetUserTransform(_vtk_transform);
}

void VTKVirtuosoRobotGraphicsObject::updateGraphicsBuffers()
{
    RenderInfo* rinfo = _latest_rinfo.load(std::memory_order_acquire);

    Geometry::TransformationMatrix endoscope_transform = rinfo->endoscope_frame.transform();
    const Vec3r cyl_frame_pos = endoscope_transform.translation() - endoscope_transform.rotMat().col(2) * rinfo->endoscope_length/2.0;

    Geometry::TransformationMatrix cyl_transform(endoscope_transform.rotMat(), cyl_frame_pos);
     // IMPORTANT: use row-major ordering since that is what VTKTransform expects (default for Eigen is col-major)
    Eigen::Matrix<Real, 4, 4, Eigen::RowMajor> transform_mat = cyl_transform.asMatrix();
    _vtk_transform->SetMatrix(transform_mat.data());

    // vtkCylinderSource creates a cylinder along the y-axis, but we expect the endoscope to be along the z-axis
    // hence we need to first rotate the cylinder provided by vtkCylinderSource by -90 deg about the x-axis
    _vtk_transform->PreMultiply();
    _vtk_transform->RotateX(-90);
}

} // namespace Graphics