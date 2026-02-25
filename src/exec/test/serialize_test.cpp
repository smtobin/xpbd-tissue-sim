#include "geometry/RefinedTetMesh.hpp"
#include "utils/MeshUtils.hpp"

int main()
{
    gmsh::initialize();

    Geometry::TetMesh mesh = MeshUtils::loadTetMeshFromGmshFile("../resource/demos/trachea_virtuoso/cao_04_29_25_model1_tumor_d.msh");
    Geometry::RefinedTetMesh refined_mesh(mesh);
    refined_mesh.setCurrentStateAsUndeformedState();

    std::vector<int> initially_refined_elements = {
        1086, 147, 740, 1085, 1010, 399, 717
    };
    for (const auto& index : initially_refined_elements)
    {
        refined_mesh.refineElement(index, 1, true, false);
    }

    std::cout << "Num elements when saving: " << refined_mesh.numElements() << std::endl;

    std::vector<std::byte> bytes;
    pack(bytes, refined_mesh);

    int num_removed = 0;
    while (num_removed < 100)
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

    std::cout << "Num elements before reset: " << refined_mesh.numElements() << std::endl;

    const std::byte* cursor = bytes.data();
    unpack(cursor, refined_mesh);

    std::cout << "Num elements after reset: " << refined_mesh.numElements() << std::endl;
}