#include "simobject/VirtuosoArm.hpp"
#include "config/simobject/VirtuosoArmConfig.hpp"
#include "graphics/vtk/VTKVirtuosoArmGraphicsObject.hpp"

#include "config/simulation/SimulationConfig.hpp"
#include "simulation/Simulation.hpp"

#include <vtkProperty.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkRenderer.h>
#include <vtkOpenGLRenderer.h>
#include <vtkActor.h>
#include <vtkPolyDataMapper.h>
#include <vtkCylinderSource.h>
#include <vtkTransform.h>

#include "common/types.hpp"

#include <memory>

int main()
{
    // create dummy simulation
    Config::SimulationConfig sim_config;
    Sim::Simulation dummy_sim(&sim_config);

    // create VirtuosoArm
    Config::VirtuosoArmConfig config("arm1", "default",
        Vec3r(0,0,0), Vec3r(0,0,0), Vec3r(0,0,0), false, false,
        1.56e-3, 1.14e-3, 1.5383e-2, 5e-3, 1.04e-3, 0.82e-3,
        0, 7e-3, 0, 10e-3,
        Sim::VirtuosoArm::ToolType::PALPATION, Sim::VirtuosoArm::CuttingModel::NONE, 15e-3, Config::ObjectRenderConfig());

    std::unique_ptr<Sim::VirtuosoArm> arm = config.createObject(&dummy_sim);
    arm->setup();

    const Sim::VirtuosoArm::OuterTubeFramesArray& ot_frames = arm->outerTubeFrames();
    const Sim::VirtuosoArm::InnerTubeFramesArray& it_frames = arm->innerTubeFrames();
    const Sim::VirtuosoArm::ToolTubeFramesArray& tt_frames = arm->toolTubeFrames();

    std::cout << "=== NO FORCE ===" << std::endl;
    std::cout << "Tool tube tip position: " << tt_frames.back().origin().transpose() << std::endl;
    std::cout << "Inner tube tip position: " << arm->actualTipPosition()[0] << ", " << arm->actualTipPosition()[1] << ", " << arm->actualTipPosition()[2] << std::endl;
    std::cout << "Outer tube tip position: " << ot_frames.back().origin()[0] << ", " << ot_frames.back().origin()[1] << ", " << ot_frames.back().origin()[2] << std::endl;

    int num_ips = ot_frames.size() + it_frames.size() + tt_frames.size();
    Mat3r tt_tip_compliance = arm->complianceMatrixAtIntegrationPoint(num_ips - 1);
    Mat3r it_tip_compliance = arm->complianceMatrixAtIntegrationPoint(num_ips - tt_frames.size() - 1);

    Mat3r approx_mid_tt_compliance = 0.5*it_tip_compliance + 0.5*tt_tip_compliance;
    int mid_tt_index = num_ips - 1 - tt_frames.size()/2;
    int mid_tt_arr_index = mid_tt_index - ot_frames.size() - it_frames.size();
    Mat3r mid_tt_compliance = arm->complianceMatrixAtIntegrationPoint(mid_tt_index);


    Vec3r applied_force(0.1, 0.1, 0);
    Vec3r cur_mid_tt_pos = tt_frames[mid_tt_arr_index].origin();
    Vec3r approx_predicted_mid_tt_pos = cur_mid_tt_pos + approx_mid_tt_compliance*applied_force;
    Vec3r predicted_mid_tt_pos = cur_mid_tt_pos + mid_tt_compliance*applied_force;

    // arm->setOuterTubeNodalForce(4, Vec3r(0,-10,0));
    // arm->setInnerTubeNodalForce(9, Vec3r(0,10,0));
    arm->setToolTubeNodalForce(mid_tt_arr_index, applied_force);
    // arm->setTipForce(Vec3r(0,50,0));
    arm->update();

    
    // const Sim::VirtuosoArm::OuterTubeFramesArray& ot_frames2 = arm->outerTubeFrames();
    std::cout << "\n=== APPLIED FORCE = (" << applied_force.transpose() << ") ===" << std::endl;
    std::cout << "Tool tube tip position: " << tt_frames.back().origin().transpose() << std::endl;
    std::cout << "Inner tube tip position: " << arm->actualTipPosition()[0] << ", " << arm->actualTipPosition()[1] << ", " << arm->actualTipPosition()[2] << std::endl;
    std::cout << "Outer tube tip position: " << ot_frames.back().origin()[0] << ", " << ot_frames.back().origin()[1] << ", " << ot_frames.back().origin()[2] << std::endl;

    std::cout << "Predicted position (approx compliance): " << approx_predicted_mid_tt_pos.transpose() << std::endl;
    std::cout << "Predicted position (real compliance): " << predicted_mid_tt_pos.transpose() << std::endl;
    std::cout << "Actual position: " << tt_frames[mid_tt_arr_index].origin().transpose() << std::endl;
    std::cout << "Approx Compliance Matrix:\n" << approx_mid_tt_compliance << std::endl;
    std::cout << "Real Compliance matrix:\n" << mid_tt_compliance << std::endl;

    


    ///////////////////////////////////////////////////////////////////////

    // visualize Virtuoso arm with VTK
    Graphics::VTKVirtuosoArmGraphicsObject arm_graphics_obj("arm", arm.get(), Config::ObjectRenderConfig());

    vtkNew<vtkOpenGLRenderer> renderer;
    renderer->SetBackground(1.0, 1.0, 1.0);
    // arm_graphics_obj.actor()->GetProperty()->SetOpacity(0.2);
    renderer->AddActor(arm_graphics_obj.actor());

    // draw collision geometry
    // for (unsigned i = 0; i < ot_frames.size()-1; i++)
    // {
    //     Vec3r start = ot_frames[i].origin();
    //     Vec3r end = ot_frames[i+1].origin();
    //     vtkNew<vtkCylinderSource> cyl_source;
    //     cyl_source->SetHeight( (end-start).norm() );
    //     cyl_source->SetRadius( arm->outerTubeOuterDiameter()/2.0 );
    //     cyl_source->SetResolution(20);
    //     cyl_source->CapsuleCapOn();

    //     vtkNew<vtkPolyDataMapper> mapper;
    //     mapper->SetInputConnection(cyl_source->GetOutputPort());

    //     vtkNew<vtkActor> actor;
    //     actor->SetMapper(mapper);
    //     actor->GetProperty()->SetColor(1.0, 1.0, 0.0);
    //     actor->GetProperty()->SetOpacity(0.4);
    //     renderer->AddActor(actor);

    //     // IMPORTANT: use row-major ordering since that is what VTKTransform expects (default for Eigen is col-major)
    //     Eigen::Matrix<Real, 4, 4, Eigen::RowMajor> cyl_transform_mat = ot_frames[i].transform().asMatrix();
    //     cyl_transform_mat.block<3,1>(0,3) = (start+end)/2.0;
    //     vtkNew<vtkTransform> vtk_transform;
    //     vtk_transform->SetMatrix(cyl_transform_mat.data());

    //     // vtkCylinderSource creates a cylinder along the y-axis, but we expect the cylinder to be along the z-axis
    //     // hence we need to first rotate the cylinder provided by vtkCylinderSource by -90 deg about the x-axis
    //     vtk_transform->PreMultiply();
    //     vtk_transform->RotateX(-90);

    //     actor->SetUserTransform(vtk_transform);
    // }
    // for (unsigned i = 0; i < it_frames.size()-1; i++)
    // {
    //     Vec3r start = it_frames[i].origin();
    //     Vec3r end = it_frames[i+1].origin();
    //     vtkNew<vtkCylinderSource> cyl_source;
    //     cyl_source->SetHeight( (end-start).norm() );
    //     cyl_source->SetRadius( arm->innerTubeOuterDiameter()/2.0 );
    //     cyl_source->SetResolution(20);
    //     cyl_source->CapsuleCapOn();

    //     vtkNew<vtkPolyDataMapper> mapper;
    //     mapper->SetInputConnection(cyl_source->GetOutputPort());

    //     vtkNew<vtkActor> actor;
    //     actor->SetMapper(mapper);
    //     actor->GetProperty()->SetColor(1.0, 1.0, 0.0);
    //     actor->GetProperty()->SetOpacity(0.4);
    //     renderer->AddActor(actor);

    //     // IMPORTANT: use row-major ordering since that is what VTKTransform expects (default for Eigen is col-major)
    //     Eigen::Matrix<Real, 4, 4, Eigen::RowMajor> cyl_transform_mat = it_frames[i].transform().asMatrix();
    //     cyl_transform_mat.block<3,1>(0,3) = (start+end)/2.0;
    //     vtkNew<vtkTransform> vtk_transform;
    //     vtk_transform->SetMatrix(cyl_transform_mat.data());

    //     // vtkCylinderSource creates a cylinder along the y-axis, but we expect the cylinder to be along the z-axis
    //     // hence we need to first rotate the cylinder provided by vtkCylinderSource by -90 deg about the x-axis
    //     vtk_transform->PreMultiply();
    //     vtk_transform->RotateX(-90);

    //     actor->SetUserTransform(vtk_transform);
    // }

    

    vtkNew<vtkRenderWindow> render_window;
    render_window->AddRenderer(renderer);
    render_window->SetSize(600,600);

    vtkNew<vtkRenderWindowInteractor> interactor;
    vtkNew<vtkInteractorStyleTrackballCamera> style;
    interactor->SetInteractorStyle(style);
    interactor->SetRenderWindow(render_window);
    render_window->Render();

    interactor->Start();
    
    return EXIT_SUCCESS;

}