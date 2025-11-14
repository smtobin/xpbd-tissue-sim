// #include "fem/FEMTetMesh.hpp"
#include "geometry/RefinedTetMesh.hpp"
#include "utils/MeshUtils.hpp"

#include "graphics/vtk/VTKMeshGraphicsObject.hpp"

#include <vtkOpenGLRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkInteractorStyleTrackballCamera.h>

int main()
{
    gmsh::initialize();

    {
        Geometry::TetMesh mesh = MeshUtils::loadTetMeshFromGmshFile("../resource/general/single.msh");

        Geometry::RefinedTetMesh refined_mesh(mesh);
        // refined_mesh.setCurrentStateAsUndeformedState();

        auto t1 = std::chrono::high_resolution_clock::now();
        refined_mesh.refineElement(0, 1);
        refined_mesh.refineElement(1, 3);
        // refined_mesh.refineElement(2, 2);
        // refined_mesh.refineElement(3, 2);
        // refined_mesh.refineElement(4, 2);
        // refined_mesh.refineElement(5, 2);
        // refined_mesh.refineElement(6, 2);
        // refined_mesh.refineElement(7, 2);
        // refined_mesh.refineElement(8, 2);
        refined_mesh.coarsenElement(25, 2);
        refined_mesh.refineElement(80, 2);
        auto t2 = std::chrono::high_resolution_clock::now();

        std::cout << "Num vertices: " << refined_mesh.numVertices() << std::endl;
        std::cout << "Num faces: " << refined_mesh.numFaces() << std::endl;
        std::cout << "Num elements: " << refined_mesh.numElements() << std::endl;
        double elapsed_ms = std::chrono::duration_cast<std::chrono::nanoseconds>(t2 - t1).count() / 1.0e6;
        std::cout << "Refining took " << elapsed_ms << " ms" << std::endl;

        // count duplicate vertices
        int num_duplicate_verts = 0;
        for (const auto& v1 : refined_mesh.vertices().validIndices())
        {
            for (const auto& v2 : refined_mesh.vertices().validIndices())
            {
                if (v2 <= v1)
                    continue;

                if ( (refined_mesh.vertex(v1) - refined_mesh.vertex(v2)).norm() < 1e-10)
                    num_duplicate_verts++;
            }
        }

        std::cout << "Number of duplicate vertices: " << num_duplicate_verts << std::endl;

        std::cout << "Number of hanging vertices: " << refined_mesh.hangingVertices().size() << std::endl;

        std::cout << "Number of hanging vertices (manual computation): " << refined_mesh.verifyHangingVertices().size() << std::endl;

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
    
    
}