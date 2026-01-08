#include "fem/VoltageFEMSolver.hpp"
#include "utils/MeshUtils.hpp"

#include <chrono>

int main()
{
    gmsh::initialize();

    // load mesh as a RefinedTetMesh
    Geometry::TetMesh mesh = MeshUtils::loadTetMeshFromGmshFile("../resource/demos/trachea_virtuoso/cao_04_29_25_model1_tumor_d.msh");
    Geometry::RefinedTetMesh refined_mesh(mesh);
    refined_mesh.setCurrentStateAsUndeformedState();

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

    std::vector<int> initially_refined_elements = {
        453, 199, 49, 176, 774, 1257, 764, 1258, 1403, 355, 245, 494, 1323, 1400, 503, 750, 1266
    };
    for (const auto& index : initially_refined_elements)
    {
        refined_mesh.refineElement(index, 2, true);
    }

    auto t1 = std::chrono::high_resolution_clock::now();
    voltage_solver.step(1e-3);
    auto t2 = std::chrono::high_resolution_clock::now();
    voltage_solver.step(1e-3);
    auto t3 = std::chrono::high_resolution_clock::now();
    const std::vector<Real>& voltage = voltage_solver.voltage();
    double elapsed_ms1 = std::chrono::duration_cast<std::chrono::nanoseconds>(t2 - t1).count() / 1.0e6;
    double elapsed_ms2 = std::chrono::duration_cast<std::chrono::nanoseconds>(t3 - t2).count() / 1.0e6;
    std::cout << "Solution:\n" << Eigen::Map<const VecXr>(voltage.data(), voltage.size()).transpose() << std::endl;
    std::cout << "1st Assembly and solve took " << elapsed_ms1 << " ms " << std::endl;
    std::cout << "2nd Assembly and solve took " << elapsed_ms2 << " ms " << std::endl;
    
    
}