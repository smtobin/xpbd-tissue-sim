#include "fem/LaplacianFEMSolver.hpp"
#include "utils/MeshUtils.hpp"

#include <chrono>

int main()
{
    gmsh::initialize();

    Geometry::TetMesh mesh = MeshUtils::loadTetMeshFromGmshFile("../resource/cube/cube4.msh");
    FEM::LaplacianFEMSolver laplace_solver(&mesh, 100);

    Geometry::AABB bbox = mesh.boundingBox();
    std::vector<int> min_z_verts = mesh.getVerticesWithZ(bbox.min[2]);
    std::vector<int> max_z_verts = mesh.getVerticesWithZ(bbox.max[2]);
    for (const auto& vert : min_z_verts)
    {
        laplace_solver.setEssentialBoundary(vert, 100);
    }
    for (const auto& vert : max_z_verts)
    {
        laplace_solver.setEssentialBoundary(vert, 0);
    }

    auto t1 = std::chrono::high_resolution_clock::now();
    VecXr x = laplace_solver.solve();
    auto t2 = std::chrono::high_resolution_clock::now();
    double elapsed_ms = std::chrono::duration_cast<std::chrono::nanoseconds>(t2 - t1).count() / 1.0e6;
    std::cout << "Solution:\n" << x << std::endl;
    std::cout << "Assembly and solve took " << elapsed_ms << " ms " << std::endl;
    
    
}