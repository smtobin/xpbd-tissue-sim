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
        0, 7e-3, 0, 20e-3,
        Sim::VirtuosoArm::ToolType::NONE, Sim::VirtuosoArm::CuttingModel::NONE, 15e-3, Config::ObjectRenderConfig());

    std::unique_ptr<Sim::VirtuosoArm> arm = config.createObject(&dummy_sim);
    arm->setup();

    const Sim::VirtuosoArm::OuterTubeFramesArray& ot_frames = arm->outerTubeFrames();
    const Sim::VirtuosoArm::InnerTubeFramesArray& it_frames = arm->innerTubeFrames();

    std::cout << "=== NO FORCE ===" << std::endl;
    std::cout << "Tool tube tip position: " << it_frames.back().origin().transpose() << std::endl;
    std::cout << "Inner tube tip position: " << arm->actualTipPosition()[0] << ", " << arm->actualTipPosition()[1] << ", " << arm->actualTipPosition()[2] << std::endl;
    std::cout << "Outer tube tip position: " << ot_frames.back().origin()[0] << ", " << ot_frames.back().origin()[1] << ", " << ot_frames.back().origin()[2] << std::endl;

    int num_ips = ot_frames.size() + it_frames.size();

    Vec3r applied_force(0,10,0);
    // arm->setOuterTubeNodalForce(12, Vec3r(0,-20,0));
    arm->setInnerTubeNodalForce(9, applied_force);
    // arm->set
    // arm->seitipForce(Vec3r(0,50,0));
    arm->update();
    
    // const Sim::VirtuosoArm::OuterTubeFramesArray& ot_frames2 = arm->outerTubeFrames();
    std::cout << "\n=== APPLIED FORCE = (" << applied_force.transpose() << ") ===" << std::endl;
    std::cout << "Inner tube tip position: " << arm->actualTipPosition()[0] << ", " << arm->actualTipPosition()[1] << ", " << arm->actualTipPosition()[2] << std::endl;
    std::cout << "Outer tube tip position: " << ot_frames.back().origin()[0] << ", " << ot_frames.back().origin()[1] << ", " << ot_frames.back().origin()[2] << std::endl;

    


    ///////////////////////////////////////////////////////////////////////

    // visualize Virtuoso arm with VTK
    Graphics::VTKVirtuosoArmGraphicsObject arm_graphics_obj("arm", arm.get(), Config::ObjectRenderConfig());

    vtkNew<vtkOpenGLRenderer> renderer;
    renderer->SetBackground(1.0, 1.0, 1.0);
    // arm_graphics_obj.actor()->GetProperty()->SetOpacity(0.2);
    renderer->AddActor(arm_graphics_obj.actor());

    

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