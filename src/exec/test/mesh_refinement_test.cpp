#include "fem/FEMTetMesh.hpp"
#include "utils/MeshUtils.hpp"

int main()
{
    gmsh::initialize();

    {
        Geometry::TetMesh mesh = MeshUtils::loadTetMeshFromGmshFile("../resource/general/single.msh");
        FEM::FEMTetMesh fem_mesh(&mesh);

        auto t1 = std::chrono::high_resolution_clock::now();
        fem_mesh.refineElement(0, 5);
        auto t2 = std::chrono::high_resolution_clock::now();

        std::cout << "Num vertices: " << mesh.numVertices() << std::endl;
        double elapsed_ms = std::chrono::duration_cast<std::chrono::nanoseconds>(t2 - t1).count() / 1.0e6;
        std::cout << "Refining took " << elapsed_ms << " ms" << std::endl;

        int num_duplicate_verts = 0;
        for (const auto& v1 : mesh.vertices().validIndices())
        {
            for (const auto& v2 : mesh.vertices().validIndices())
            {
                if (v2 <= v1)
                    continue;

                if ( (mesh.vertex(v1)-mesh.vertex(v2)).norm() < 1e-10)
                    num_duplicate_verts++;
            }
        }

        std::cout << "Number of duplicate vertices: " << num_duplicate_verts << std::endl;
    }
    
    
}