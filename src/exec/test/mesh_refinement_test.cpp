// #include "fem/FEMTetMesh.hpp"
#include "geometry/RefinedTetMesh.hpp"
#include "utils/MeshUtils.hpp"

#include "graphics/vtk/VTKMeshGraphicsObject.hpp"

#include <vtkOpenGLRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkInteractorStyleTrackballCamera.h>

bool testCorrectness(Geometry::RefinedTetMesh& refined_mesh)
{
    bool correct = true;

    /** Check 1: duplicate vertices */
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

    if (num_duplicate_verts > 0)
    {
        std::cout << "   " << num_duplicate_verts << " duplicate vertices detected!" << std::endl;
        correct = false;
    }

    /** Check 2: Hanging vertices */

    auto hanging_verts = refined_mesh.hangingVertices();
    auto verified_hanging_verts = refined_mesh.verifyHangingVertices();
    if (refined_mesh.hangingVertices().size() != verified_hanging_verts.size())
    {
        std::cout << "   Mismatch in number of hanging vertices!" << std::endl;
        std::cout << "     - Number of hanging vertices: " << refined_mesh.hangingVertices().size() << std::endl;
        std::cout << "     - Number of hanging vertices (manual computation): " << refined_mesh.verifyHangingVertices().size() << std::endl;
        correct = false;
    }

    for (const auto& [v,edge] : hanging_verts)
    {
        if (verified_hanging_verts.count(v) == 0)
        {
            std::cout << "   Vertex " << v << " is in hanging_verts but not verified_hanging_verts!" << std::endl;
            correct = false;
        }
    }


    for (const auto& v : verified_hanging_verts)
    {
        if (hanging_verts.count(v) == 0)
        {
            std::cout << "   Vertex " << v << " is in verified_hanging_verts but not hanging_verts!" << std::endl;
            correct = false;
        }
    }


    /** Check 3: Adjacent vertices lists */

    std::vector<std::unordered_set<int>> vertex_adjacent_vertices(refined_mesh.vertices().totalSize());
    for (const auto& ind : refined_mesh.vertices().validIndices())
    {
        vertex_adjacent_vertices[ind] = refined_mesh.vertexAdjacentVertices(ind);
    }

    std::vector<Real> vertex_volumes(refined_mesh.vertices().totalSize());
    for (const auto& ind : refined_mesh.vertices().validIndices())
    {
        vertex_volumes[ind] = refined_mesh.vertexRestVolume(ind);
    }
    
    refined_mesh.setCurrentStateAsUndeformedState(); // this will recompute the vertex adjacency information
    for (const auto& ind : refined_mesh.vertices().validIndices())
    {
        const std::unordered_set<int>& new_adj_verts = refined_mesh.vertexAdjacentVertices(ind);
        if (new_adj_verts.size() != vertex_adjacent_vertices[ind].size())
        {
            std::cout << "   Different number of adjacent verts for vertex " << ind << std::endl;
            std::cout << "     - Computed adjacent verts: (";
            for (const auto& v : vertex_adjacent_vertices[ind])
                std::cout << v << ", ";
            std::cout << ")\n     - Correct adjacent verts: (";
            for (const auto& v : new_adj_verts)
                std::cout << v << ", ";
            std::cout << ")" << std::endl;
            std::cout << "     - Vertex " << ind << ": " << refined_mesh.vertex(ind).transpose() << std::endl;

            correct = false;
        }
    }

    /** Check 4: Vertex rest volumes */
    // we already set the current state as undeformed state, so we have already recomputed the vertex rest volumes
    for (const auto& ind : refined_mesh.vertices().validIndices())
    {
        Real new_vertex_volume = refined_mesh.vertexRestVolume(ind);
        if (std::abs(new_vertex_volume - vertex_volumes[ind]) > 1e-14)
        {
            std::cout << "   Vertex " << ind << " has a different vertex volume! New: " << new_vertex_volume << "  Old: " << vertex_volumes[ind] << std::endl;
        }
    }

    return correct;
}

/** Test basic refinement */
std::pair<bool, Geometry::RefinedTetMesh> test0()
{
    Geometry::TetMesh mesh = MeshUtils::loadTetMeshFromGmshFile("../resource/general/single.msh");
    Geometry::RefinedTetMesh refined_mesh(mesh);
    refined_mesh.setCurrentStateAsUndeformedState();

    refined_mesh.refineElement(0, 1, true);
    refined_mesh.refineElement(1, 2, false);

    return std::make_pair(testCorrectness(refined_mesh), refined_mesh);
}

/** Test refinement past level 2 */
std::pair<bool, Geometry::RefinedTetMesh> test1()
{
    Geometry::TetMesh mesh = MeshUtils::loadTetMeshFromGmshFile("../resource/general/single.msh");
    Geometry::RefinedTetMesh refined_mesh(mesh);
    refined_mesh.setCurrentStateAsUndeformedState();

    refined_mesh.refineElement(0, 1, true);
    refined_mesh.refineElement(1, 3, false);

    return std::make_pair(testCorrectness(refined_mesh), refined_mesh);
}

/** Test refinement after removing elements */
std::pair<bool, Geometry::RefinedTetMesh> test2()
{
    Geometry::TetMesh mesh = MeshUtils::loadTetMeshFromGmshFile("../resource/general/single.msh");
    Geometry::RefinedTetMesh refined_mesh(mesh);
    refined_mesh.setCurrentStateAsUndeformedState();

    refined_mesh.refineElement(0, 1, true);
    refined_mesh.removeElement(1);
    refined_mesh.removeElement(2);
    refined_mesh.removeElement(6);
    refined_mesh.removeElement(4);
    refined_mesh.refineElement(5, 3);

    return std::make_pair(testCorrectness(refined_mesh), refined_mesh);
}

/** Test refinement + coarsening + refinement after removing elements */
std::pair<bool, Geometry::RefinedTetMesh> test3()
{
    Geometry::TetMesh mesh = MeshUtils::loadTetMeshFromGmshFile("../resource/general/single.msh");
    Geometry::RefinedTetMesh refined_mesh(mesh);
    refined_mesh.setCurrentStateAsUndeformedState();

    refined_mesh.refineElement(0, 1, true);
    refined_mesh.removeElement(1);
    refined_mesh.removeElement(2);
    refined_mesh.removeElement(6);
    refined_mesh.removeElement(4);
    refined_mesh.refineElement(5, 2);
    refined_mesh.coarsenElement(20, 1);
    refined_mesh.refineElement(3, 3);

    return std::make_pair(testCorrectness(refined_mesh), refined_mesh);
}

/** Test uniform level=1 refinement on a mesh with multiple initial tets */
std::pair<bool, Geometry::RefinedTetMesh> test4()
{
    Geometry::TetMesh mesh = MeshUtils::loadTetMeshFromGmshFile("../resource/cube/cube2.msh");
    // Geometry::TetMesh mesh = MeshUtils::loadTetMeshFromGmshFile("../resource/demos/trachea_virtuoso/cao_04_29_25_model1_tumor_d.msh");
    Geometry::RefinedTetMesh refined_mesh(mesh);
    refined_mesh.setCurrentStateAsUndeformedState();

    int num_initial_faces = refined_mesh.numFaces();
    for (int i = 0; i < num_initial_faces; i++)
    {
        int elem_to_refine = refined_mesh.elementWithFace(i);
        refined_mesh.refineElement(elem_to_refine, 1, true);
    }

    // std::cout << "Element with face 1: " << refined_mesh.element(refined_mesh.elementWithFace(1)).transpose() << std::endl;
    // std::cout << "Element with face 4: " << refined_mesh.element(refined_mesh.elementWithFace(4)).transpose() << std::endl;

    refined_mesh.refineElement(refined_mesh.elementWithFace(1), 2, true);
    refined_mesh.refineElement(refined_mesh.elementWithFace(4), 2, true);

    return std::make_pair(testCorrectness(refined_mesh), refined_mesh);
}

/** Test level=2 refinement on a mesh with multiple initial tets */
std::pair<bool, Geometry::RefinedTetMesh> test5()
{
    Geometry::TetMesh mesh = MeshUtils::loadTetMeshFromGmshFile("../resource/general/double.msh");
    // Geometry::TetMesh mesh = MeshUtils::loadTetMeshFromGmshFile("../resource/demos/trachea_virtuoso/cao_04_29_25_model1_tumor_d.msh");
    Geometry::RefinedTetMesh refined_mesh(mesh);
    refined_mesh.setCurrentStateAsUndeformedState();

    // std::cout << "Element with face 1: " << refined_mesh.element(refined_mesh.elementWithFace(1)).transpose() << std::endl;
    // std::cout << "Element with face 4: " << refined_mesh.element(refined_mesh.elementWithFace(4)).transpose() << std::endl;

    refined_mesh.refineElement(refined_mesh.elementWithFace(1), 2, true);
    // refined_mesh.refineElement(refined_mesh.elementWithFace(4), 2, true);

    return std::make_pair(testCorrectness(refined_mesh), refined_mesh);
}

/** Test refinement and coarsening on a mesh with multiple initial tets */
std::pair<bool, Geometry::RefinedTetMesh> test6()
{
    // Geometry::TetMesh mesh = MeshUtils::loadTetMeshFromGmshFile("../resource/cube/cube2.msh");
    Geometry::TetMesh mesh = MeshUtils::loadTetMeshFromGmshFile("../resource/demos/trachea_virtuoso/cao_04_29_25_model1_tumor_d.msh");
    Geometry::RefinedTetMesh refined_mesh(mesh);
    refined_mesh.setCurrentStateAsUndeformedState();

    std::vector<int> initially_refined_elements = {
        453, 199, 49, 176, 774, 1257, 764, 1258, 1403, 355, 245, 494, 1323, 1400, 503, 750, 1266
    };
    for (const auto& index : initially_refined_elements)
    {
        refined_mesh.refineElement(index, 2, true);
    }
    
    // coarsen 10 of the elements at random
    int num_coarsened = 0;
    while (num_coarsened < 10)
    {
        for (const auto& index : refined_mesh.elements().validIndices())
        {
            if (refined_mesh.elementRefinementLevel(index) > 0)
            {
                refined_mesh.coarsenElement(index, 1, false);
                num_coarsened++;
                break;
            }
        }
    }



    return std::make_pair(testCorrectness(refined_mesh), refined_mesh);
}

/** Test refinement and then removal */
std::pair<bool, Geometry::RefinedTetMesh> test7()
{
    // Geometry::TetMesh mesh = MeshUtils::loadTetMeshFromGmshFile("../resource/general/double.msh");
    // Geometry::TetMesh mesh = MeshUtils::loadTetMeshFromGmshFile("../resource/demos/trachea_virtuoso/cao_04_29_25_model1_tumor_d.msh");
    Geometry::TetMesh mesh = MeshUtils::loadTetMeshFromGmshFile("../resource/cube/cube2.msh");
    Geometry::RefinedTetMesh refined_mesh(mesh);
    refined_mesh.setCurrentStateAsUndeformedState();

    // std::vector<int> initially_refined_elements = {
    //     1086, 147, 740, 1085, 1010, 399, 717
    // };
    // for (const auto& index : initially_refined_elements)
    // {
    //     refined_mesh.refineElement(index, 1, true);
    //     break;
    // }
    // refined_mesh.refineElement(4, 1, true);
    refined_mesh.refineElement(0, 1, true);
    
    
    // search for elements with refinement level > 0 and remove them
    int num_removed = 0;
    while (num_removed < 1)
    {
        for (const auto& index : refined_mesh.elements().validIndices())
        {
            if (refined_mesh.elementRefinementLevel(index) > 0)
            {
                refined_mesh.removeElement(index);
                num_removed++;
                break;
            }
        }
    }


    return std::make_pair(testCorrectness(refined_mesh), refined_mesh);
}

/** Test refinement and then lots of removal */
std::pair<bool, Geometry::RefinedTetMesh> test8()
{
    Geometry::TetMesh mesh = MeshUtils::loadTetMeshFromGmshFile("../resource/general/double.msh");
    // Geometry::TetMesh mesh = MeshUtils::loadTetMeshFromGmshFile("../resource/demos/trachea_virtuoso/cao_04_29_25_model1_tumor_d.msh");
    // Geometry::TetMesh mesh = MeshUtils::loadTetMeshFromGmshFile("../resource/cube/cube2.msh");
    Geometry::RefinedTetMesh refined_mesh(mesh);
    refined_mesh.setCurrentStateAsUndeformedState();

    refined_mesh.refineElement(0, 2, true);
    std::vector<int> latest_added_elements = refined_mesh.latestAddedElements();
    // std::cout << "Added elements:\n" << std::endl;
    for (const auto& e : latest_added_elements)
    {
        std::cout << e << ", ";
    }
    std::cout << std::endl;
    
    
    // search for elements with refinement level > 0 and remove them
    int num_removed = 0;
    while (num_removed < 60)
    {
        for (const auto& index : latest_added_elements)
        {
            // std::cout << " Element " << index << " refinement level: " << refined_mesh.elementRefinementLevel(index) << std::endl;
            // std::cout << " Element " << index << " on surface: " << refined_mesh.elementOnSurface(index) << std::endl;
            if (refined_mesh.elementRefinementLevel(index) > 0 && refined_mesh.elementOnSurface(index))
            {
                // std::cout << "Removing element " << index << "..." << std::endl;
                refined_mesh.removeElement(index);
                num_removed++;
                break;
            }
        }
    }


    return std::make_pair(testCorrectness(refined_mesh), refined_mesh);
}

std::pair<bool, Geometry::RefinedTetMesh> test9()
{
    // Geometry::TetMesh mesh = MeshUtils::loadTetMeshFromGmshFile("../resource/general/double.msh");
    Geometry::TetMesh mesh = MeshUtils::loadTetMeshFromGmshFile("../resource/demos/trachea_virtuoso/cao_04_29_25_model1_tumor_d.msh");
    // Geometry::TetMesh mesh = MeshUtils::loadTetMeshFromGmshFile("../resource/cube/cube2.msh");
    Geometry::RefinedTetMesh refined_mesh(mesh);
    refined_mesh.setCurrentStateAsUndeformedState();

    // std::vector<int> initially_refined_elements = {
    //     1086, 147, 740, 1085, 1010, 399, 717
    // };
    std::vector<int> initially_refined_elements = {
        1086, 147, 740, 1085, 1010, 399, 717
    };
    for (const auto& index : initially_refined_elements)
    {
        refined_mesh.refineElement(index, 1, true, false);
    }
    // std::vector<int> added_elements = refined_mesh.latestAddedElements();
    
    
    // search for elements with refinement level > 0 and remove them
    int num_removed = 0;
    while (num_removed < 2000)
    {
        for (const auto& index : refined_mesh.elements().validIndices())
        {
            if (refined_mesh.elementValid(index) && refined_mesh.elementRefinementLevel(index) > 0 && refined_mesh.elementOnSurface(index))
            {
                refined_mesh.removeElement(index);
                num_removed++;
                break;
            }
        }
    }

    std::vector<Geometry::Edge> boundary_edges = refined_mesh.boundaryEdges();
    std::cout << "Number of boundary edges: " << boundary_edges.size() << std::endl;


    return std::make_pair(testCorrectness(refined_mesh), refined_mesh);
}

std::pair<bool, Geometry::RefinedTetMesh> test10()
{
    Geometry::TetMesh mesh = MeshUtils::loadTetMeshFromGmshFile("../resource/general/double.msh");
    // Geometry::TetMesh mesh = MeshUtils::loadTetMeshFromGmshFile("../resource/demos/trachea_virtuoso/cao_04_29_25_model1_tumor_d.msh");
    // Geometry::TetMesh mesh = MeshUtils::loadTetMeshFromGmshFile("../resource/cube/cube2.msh");
    Geometry::RefinedTetMesh refined_mesh(mesh);
    refined_mesh.setCurrentStateAsUndeformedState();

    refined_mesh.refineElement(0, 2, true);
    std::vector<int> latest_added_elements = refined_mesh.latestAddedElements();
    
    
    // search for elements with refinement level > 0 and remove them
    int num_removed = 0;
    while (num_removed < 1)
    {
        for (const auto& index : latest_added_elements)
        {
            if (refined_mesh.elementRefinementLevel(index) > 0 && refined_mesh.elementOnSurface(index))
            {
                refined_mesh.removeElement(index);
                num_removed++;
                break;
            }
        }
    }
    std::vector<int> latest_added_elements2 = refined_mesh.latestAddedElements();

    // now try and coarsen elements
    for (const auto& index : latest_added_elements)
    {
        if (refined_mesh.elementValid(index) && refined_mesh.elementRefinementLevel(index) > 0 && refined_mesh.elementOnSurface(index))
        {
            refined_mesh.coarsenElement(index, 0, true);
        }
    }

    // now try and coarsen elements
    for (const auto& index : latest_added_elements2)
    {
        if (refined_mesh.elementValid(index) && refined_mesh.elementRefinementLevel(index) > 0 && refined_mesh.elementOnSurface(index))
        {
            refined_mesh.coarsenElement(index, 0, true);
        }
    }


    return std::make_pair(testCorrectness(refined_mesh), refined_mesh);
}


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


int main()
{
    gmsh::initialize();

    using TestFuncType = std::function<std::pair<bool, Geometry::RefinedTetMesh> ()>;
    std::vector<TestFuncType> test_funcs = {test0, test1, test2, test3, test4, test5, test6, test7, test8, test9, test10};
    std::vector<bool> successes(test_funcs.size(), false);
    std::vector<Geometry::RefinedTetMesh> refined_meshes;

    unsigned visualize_index = -1;//test_funcs.size()-1;

    std::cout << "\n=== Running tests ===" << std::endl;
    for (unsigned i = 0; i < test_funcs.size(); i++)
    {
        std::cout << " \nRunning test " << i << "..." << std::endl;
        auto [success, refined_mesh] = test_funcs[i]();
        successes[i] = success;
        refined_meshes.push_back(std::move(refined_mesh));
    }


    // print out which tests succeeded
    std::cout << "\n=== Test results ===" << std::endl;
    for (unsigned i = 0; i < successes.size(); i++)
    {
        std::string true_false = successes[i] ? "Pass" : "Fail";
        std::cout << " Test " << i << ": " << true_false << std::endl;
    }

    if (visualize_index > 0 && visualize_index < refined_meshes.size())
    {
        std::cout << "\n=== Visualized mesh stats ===" << std::endl;
        std::cout << " Num vertices: " << refined_meshes[visualize_index].numVertices() << std::endl;
        std::cout << " Num faces: " << refined_meshes[visualize_index].numFaces() << std::endl;
        std::cout << " Num elements: " << refined_meshes[visualize_index].numElements() << std::endl;

        auto hanging_verts = refined_meshes[visualize_index].hangingVertices();
        auto verified_hanging_verts = refined_meshes[visualize_index].verifyHangingVertices();
        std::cout << " Number of hanging vertices: " << hanging_verts.size() << std::endl;
        std::cout << " Number of hanging vertices (manual computation): " << verified_hanging_verts.size() << std::endl;

        visualizeMesh(refined_meshes[visualize_index]);
    }

    return 0;
    
}