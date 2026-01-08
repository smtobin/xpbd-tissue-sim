#include "fem/VoltageFEMSolver.hpp"
#include "utils/MeshUtils.hpp"

#include "graphics/vtk/VTKMeshGraphicsObject.hpp"

#include <vtkOpenGLRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkInteractorStyleTrackballCamera.h>

#include <chrono>

void visualizeMesh(const Geometry::RefinedTetMesh& refined_mesh)
{
    // visualize mesh with VTK
    Config::ObjectRenderConfig render_config(
        Config::ObjectRenderConfig::RenderType::PBR,
        std::nullopt, std::nullopt, std::nullopt,
        0.0, 0.3, 1.0,
        Vec3r(1.0, 0.0, 0.0),
        false,
        true,
        true,
        false
    );
    Graphics::VTKMeshGraphicsObject mesh_graphics_obj("mesh1", &refined_mesh, render_config);

    vtkNew<vtkOpenGLRenderer> renderer;
    renderer->SetBackground(0.0, 1.0, 1.0);
    // arm_graphics_obj.actor()->GetProperty()->SetOpacity(0.2);
    renderer->AddActor(mesh_graphics_obj.facesActor());
    renderer->AddActor(mesh_graphics_obj.edgesActor());

    vtkNew<vtkRenderWindow> render_window;
    render_window->AddRenderer(renderer);
    render_window->SetSize(600,600);

    vtkNew<vtkRenderWindowInteractor> interactor;
    vtkNew<vtkInteractorStyleTrackballCamera> style;
    interactor->SetInteractorStyle(style);
    interactor->SetRenderWindow(render_window);
    render_window->Render();

    interactor->Start();
}

void voltageSolveAndGradientTest(FEM::VoltageFEMSolver& voltage_solver, const Geometry::RefinedTetMesh& refined_mesh)
{
    auto t1 = std::chrono::high_resolution_clock::now();
    voltage_solver.step(1e-3);
    auto t2 = std::chrono::high_resolution_clock::now();
    voltage_solver.step(1e-3);
    auto t3 = std::chrono::high_resolution_clock::now();
    const std::vector<Real>& voltage = voltage_solver.voltage();

    // calculate the max voltage gradient in the mesh
    Real max_gradient_mag = 0;
    int max_gradient_elem = -1;
    for (const auto& elem_index : refined_mesh.elements().validIndices())
    {
        Vec3r voltage_gradient = voltage_solver.elementVoltageGradient(elem_index);
        Real voltage_gradient_mag = voltage_gradient.norm();
        if (voltage_gradient_mag > max_gradient_mag)
        {
            max_gradient_mag = voltage_gradient_mag;
            max_gradient_elem = elem_index;
        }
    }

    const Vec4i& max_gradient_elem_vertices = refined_mesh.element(max_gradient_elem);
    std::cout << "Max voltage gradient element vertices: " << max_gradient_elem_vertices.transpose() << std::endl;
    for (int i = 0; i < 4; i++)
    {
        for (int j = i+1; j < 4; j++)
        {
            int v1 = max_gradient_elem_vertices[i];
            int v2 = max_gradient_elem_vertices[j];
            Real edge_dist = (refined_mesh.vertex(v1) - refined_mesh.vertex(v2)).norm();
            std::cout << " Edge " << i << j << " distance: " << edge_dist*1000 << " mm" << std::endl;
        }
    }
    
    std::cout << "----------------------------------------------------------------------" << std::endl;
    std::cout << "Max voltage gradient magnitude: " << max_gradient_mag << std::endl;

    // calculate approximate time to 100 C
    Real sigma = 0.5; // electrical conductivity [S/m]
    Real density = 1000; // density [kg/m^3]
    Real c = 4000; // specific heat [J/kg-K]
    Real dT_dt = sigma * max_gradient_mag * max_gradient_mag / (density * c);
    Real t_to_100 = 80 / dT_dt; // ~80 deg to get from room temp to 100 C
    std::cout << "Approximate time to 100 C: " << t_to_100*1000 << " ms" << std::endl;
    std::cout << "----------------------------------------------------------------------" << std::endl;


    

    double elapsed_ms1 = std::chrono::duration_cast<std::chrono::nanoseconds>(t2 - t1).count() / 1.0e6;
    double elapsed_ms2 = std::chrono::duration_cast<std::chrono::nanoseconds>(t3 - t2).count() / 1.0e6;
    // std::cout << "Solution:\n" << Eigen::Map<const VecXr>(voltage.data(), voltage.size()).transpose() << std::endl;
    std::cout << "1st Assembly and solve took " << elapsed_ms1 << " ms " << std::endl;
    std::cout << "2nd Assembly and solve took " << elapsed_ms2 << " ms " << std::endl;
}

int main()
{
    gmsh::initialize();

    // load mesh as a RefinedTetMesh
    Geometry::TetMesh mesh = MeshUtils::loadTetMeshFromGmshFile("../resource/demos/trachea_virtuoso/cao_04_29_25_model1_tumor_d.msh");
    Geometry::AABB bbox = mesh.boundingBox();
    std::cout << "Bounding box size: " << bbox.size().transpose() << std::endl;
    mesh.resize(bbox.size()/1000);
    Geometry::RefinedTetMesh refined_mesh(mesh);
    refined_mesh.setCurrentStateAsUndeformedState();

    std::cout << "Number of elements before refinement: " << refined_mesh.numElements() << std::endl;

    // get ground faces
    std::set<int> fixed_vertices;
    std::vector<int> fixed_faces;
    std::string fixed_faces_filename = "../resource/demos/trachea_virtuoso/cao_04_29_25_model1_tumor_d_fixed_faces.txt";
    MeshUtils::verticesAndFacesFromFixedFacesFile(fixed_faces_filename, fixed_vertices, fixed_faces);
    
    FEM::VoltageFEMSolver voltage_solver(&refined_mesh, 100);

    // set voltage essential boundary where it is grounded
    for (const auto& vert : fixed_vertices)
    {
        voltage_solver.setVoltageAtBoundary(vert, 0, true);
    }

    /** Refine in a 1-ring around a vertex on the surface */
    // start with an arbitrary element on the surface
    int surface_elem = 453;
    const Vec4i& surface_elem_vertices = refined_mesh.element(surface_elem);
    // get a vertex in that element that is on the surface
    int surface_vertex = -1;
    for (const auto& v : surface_elem_vertices)
    {
        if (refined_mesh.vertexOnSurface(v))
        {
            surface_vertex = v;
            break;
        }
    }
    assert(surface_vertex >= 0);

    // set voltage at the surface vertex
    voltage_solver.setVoltageAtBoundary(surface_vertex, 100, false);

    std::cout << "Center surface vertex: " << surface_vertex << std::endl;

    std::cout << "\n============================================" << std::endl;
    std::cout << "Refinement" << std::endl;
    std::cout << "============================================" << std::endl;

    std::cout << "\n=== 0 Levels of Local Refinement ===" << std::endl;
    voltageSolveAndGradientTest(voltage_solver, refined_mesh);

    // get all attached elements to that vertex and refine them
    int num_refinements = 4;
    for (int i = 0; i < num_refinements; i++)
    {
        auto attached_elements = refined_mesh.vertexAttachedElements(surface_vertex);   // make a copy
        for (const auto& elem_index : attached_elements)
        {
            refined_mesh.refineElement(elem_index, 1, false);
        }
        std::cout << "\n=== " << i+1 << " Levels of Local Refinement ===" << std::endl;
        std::cout << "Number of elements after refinement: " << refined_mesh.numElements() << std::endl;
        voltageSolveAndGradientTest(voltage_solver, refined_mesh);
    }

    // now coarsen and check that we get the same thing

    std::cout << "\n\n============================================" << std::endl;
    std::cout << "Coarsening" << std::endl;
    std::cout << "============================================" << std::endl;

    auto attached_elements = refined_mesh.vertexAttachedElements(surface_vertex);
    for (int i = 0; i < num_refinements; i++)
    {
        auto attached_elements = refined_mesh.vertexAttachedElements(surface_vertex);   // make a copy
        for (const auto& elem_index : attached_elements)
        {
            refined_mesh.coarsenElement(elem_index, 1, false);
        }
        std::cout << "\n=== " << num_refinements - (i+1) << " Levels of Local Refinement ===" << std::endl;
        std::cout << "Number of elements after coarsening: " << refined_mesh.numElements() << std::endl;
        voltageSolveAndGradientTest(voltage_solver, refined_mesh);
    }
    

    visualizeMesh(refined_mesh);
    
    
}