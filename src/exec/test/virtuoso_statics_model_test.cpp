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

    /** Cubic fit of compliances for tool tube */
    // get compliances at evaluation points
    int tt_ip3 = num_ips - 1;
    int tt_ip2 = num_ips - 1 - tt_frames.size()/2;
    int tt_ip1 = num_ips - 1 - 3*tt_frames.size()/4;
    int tt_ip0 = num_ips - tt_frames.size();
    Real s_ip3 = Real(tt_ip3 - ( ot_frames.size() + it_frames.size() )) / (tt_frames.size()-1);
    Real s_ip2 = Real(tt_ip2 - ( ot_frames.size() + it_frames.size() )) / (tt_frames.size()-1);
    Real s_ip1 = Real(tt_ip1 - ( ot_frames.size() + it_frames.size() )) / (tt_frames.size()-1);
    Real s_ip0 = Real(tt_ip0 - ( ot_frames.size() + it_frames.size() )) / (tt_frames.size()-1);
    Mat3r tt_compliance3 = arm->complianceMatrixAtIntegrationPoint(tt_ip3);
    Mat3r tt_compliance2 = arm->complianceMatrixAtIntegrationPoint(tt_ip2);
    Mat3r tt_compliance1 = arm->complianceMatrixAtIntegrationPoint(tt_ip1);
    Mat3r tt_compliance0 = arm->complianceMatrixAtIntegrationPoint(tt_ip0);
    // solve 4x4 system for coefficients
    Mat4r vdm;
    // vdm << 1, 1, 1, 1,
    //        s_ip0, s_ip1, s_ip2, s_ip3,
    //        s_ip0*s_ip0, s_ip1*s_ip1, s_ip2*s_ip2, s_ip3*s_ip3,
    //        s_ip0*s_ip0*s_ip0, s_ip1*s_ip1*s_ip1, s_ip2*s_ip2*s_ip2, s_ip3*s_ip3*s_ip3;
    vdm << 1, s_ip0, s_ip0*s_ip0, s_ip0*s_ip0*s_ip0,
           1, s_ip1, s_ip1*s_ip1, s_ip1*s_ip1*s_ip1,
           1, s_ip2, s_ip2*s_ip2, s_ip2*s_ip2*s_ip2,
           1, s_ip3, s_ip3*s_ip3, s_ip3*s_ip3*s_ip3;

    Vec4r rhs( tt_compliance0(0,0), tt_compliance1(0,0), tt_compliance2(0,0), tt_compliance3(0,0));
    Vec4r a = vdm.colPivHouseholderQr().solve(rhs);

    int tt_ip_eval = num_ips - 1 - tt_frames.size()/2 + 2;
    Real s_ip_eval = Real(tt_ip_eval - ( ot_frames.size() + it_frames.size() )) / (tt_frames.size()-1);
    Real C00_int = a[0] + s_ip_eval * a[1] + s_ip_eval*s_ip_eval*a[2] + s_ip_eval*s_ip_eval*s_ip_eval*a[3];
    
    Mat3r C_ip_eval_interp = arm->interpolatedComplianceMatrix(tt_ip_eval, 0);

    Mat3r C_ip_eval = arm->complianceMatrixAtIntegrationPoint(tt_ip_eval);
    std::cout << "VDM:\n" << vdm << std::endl;
    std::cout << "RHS:" << rhs.transpose() << std::endl;
    std::cout << "a:" << a.transpose() << std::endl;
    std::cout << "C00 interpolated: " << C00_int << std::endl;
    std::cout << "Actual C:\n" << C_ip_eval << std::endl;
    std::cout << "Interpolated C:\n" << C_ip_eval_interp << std::endl;
    std::cout << "Inverse C:\n" << C_ip_eval_interp.inverse() << std::endl;

    Mat2r vdm_cheap;
    vdm_cheap << 1, s_ip0*s_ip0*s_ip0, 1, s_ip3*s_ip3*s_ip3;
    Vec2r rhs_cheap( tt_compliance0(0,0), tt_compliance3(0,0) );
    Vec2r a_cheap = vdm_cheap.colPivHouseholderQr().solve(rhs_cheap);
    std::cout << "cheap a: " << a_cheap.transpose() << std::endl;
    std::cout << "C00 cheaply interpolated: " << a_cheap[0] + a_cheap[1]*s_ip_eval*s_ip_eval*s_ip_eval << std::endl;
    std::cout << "C00 linearly interpolated: " << (1-s_ip_eval)*tt_compliance0(0,0) + s_ip_eval*tt_compliance3(0,0) << std::endl;


    Mat3r tt_tip_compliance = tt_compliance3;
    Mat3r it_tip_compliance = arm->complianceMatrixAtIntegrationPoint(num_ips - tt_frames.size() - 1);

    Mat3r approx_mid_tt_compliance = 0.5*it_tip_compliance + 0.5*tt_tip_compliance;
    int mid_tt_index = num_ips - 1 - tt_frames.size()/2;
    int mid_tt_arr_index = mid_tt_index - ot_frames.size() - it_frames.size();
    Mat3r mid_tt_compliance = arm->complianceMatrixAtIntegrationPoint(mid_tt_index);


    Vec3r applied_force(4, 4, 0);
    Vec3r cur_mid_tt_pos = tt_frames[mid_tt_arr_index].origin();
    Vec3r cur_mid_tt_tan = tt_frames[mid_tt_arr_index].transform().rotMat().col(2);
    Vec3r approx_predicted_mid_tt_pos = cur_mid_tt_pos + approx_mid_tt_compliance*applied_force;
    Vec3r predicted_mid_tt_pos = cur_mid_tt_pos + mid_tt_compliance*applied_force;
    Vec3r mid_tt_pos_diff = predicted_mid_tt_pos - cur_mid_tt_pos;
    Vec3r F_req = mid_tt_compliance.colPivHouseholderQr().solve(mid_tt_pos_diff);

    Eigen::JacobiSVD<Mat3r> svd(mid_tt_compliance, Eigen::ComputeFullU | Eigen::ComputeFullV);
    std::cout << "Compliance SVD values: " << svd.singularValues().transpose() << std::endl;
    std::cout << "Compliance SVD U:\n" << svd.matrixU() << std::endl;
    std::cout << "Compliance SVD V:\n" << svd.matrixV() << std::endl;

    Vec3r pred_diff = mid_tt_compliance*applied_force;

    std::cout << "Tool tube mid positional diff: " << mid_tt_pos_diff.transpose() << std::endl;
    std::cout << "Tool tube mid tangent: " << cur_mid_tt_tan.transpose() << std::endl;
    std::cout << " U3 dot pred diff: " << svd.matrixU().col(2).dot(pred_diff) << std::endl;
    std::cout << "Required force: " << F_req.transpose() << std::endl;

    Vec3r pos_diff_des(1e-3, 1e-3, 0);
    Vec3r pos_diff_des_non_stiff = pos_diff_des - pos_diff_des.dot(svd.matrixU().col(2)) * svd.matrixU().col(2);
    Vec3r stiff_force = mid_tt_compliance.colPivHouseholderQr().solve(pos_diff_des);
    Vec3r non_stiff_force =  mid_tt_compliance.colPivHouseholderQr().solve(pos_diff_des_non_stiff);
    std::cout << "Required force for (1 mm, 1 mm, 0): " <<  stiff_force.transpose() << std::endl;
    std::cout << "Required force for (1 mm, 1 mm, 0) (non-stiff): " << non_stiff_force.transpose() << std::endl;

    std::cout << "Original position: " << cur_mid_tt_pos.transpose() << std::endl;

    // arm->setOuterTubeNodalForce(4, Vec3r(0,-10,0));
    // arm->setInnerTubeNodalForce(9, Vec3r(0,10,0));
    // arm->setToolTubeNodalForce(mid_tt_arr_index, applied_force);
    arm->setToolTubeNodalForce(mid_tt_arr_index, non_stiff_force);
    // arm->setTipForce(Vec3r(0,50,0));
    arm->update();

    std::cout << "New position: " << tt_frames[mid_tt_arr_index].origin().transpose() << std::endl;
    std::cout << "Diff: " << (tt_frames[mid_tt_arr_index].origin() - cur_mid_tt_pos).transpose() << std::endl;
    

    
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

    std::cout << "\n\nIT tip compliance matrix:\n" << it_tip_compliance << std::endl;
    std::cout << "\n\nTT tip compliance matrix:\n" << tt_tip_compliance << std::endl;

    


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