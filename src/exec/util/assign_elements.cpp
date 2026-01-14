#include "geometry/Mesh.hpp"
#include "geometry/embree/EmbreeScene.hpp"

#include "utils/MeshUtils.hpp"

#include <vector>
#include <set>
#include <map>
#include <fstream>
#include <iostream>

/** A simple utility for generating an "element classes" file, which is simply a vector of integers.
 * The i'th integer in the vector corresponds to the class of the i'th element.
 * 
 * Classes are determined by finding overlapping elements with "submeshes".
 * E.g. 
 *   If we have a trachea mesh with a tumor, and we want to distinguish between the trachea and the tumor, we can give this script
 *   the tumor as a separate mesh, and the script will find which elements in the combined mesh overlap with those in the separate tumor mesh,
 *   and generate a file with 0s for element indices corresponding to the trachea, and 1s for element indices corresponding to the tumor.
 * 
 * Usage:
 * 
 * ./AssignElements <combined-mesh-filename> <separate-mesh-1> <class-1> <separate-mesh-2> <class-2> ...
 * 
 * where 
 *  - <combined-mesh-filename> is the path to the "combined" mesh used in the simulation that will have different material properties
 *  - <separate-mesh-i> is the i'th separate mesh (usually generated from CT segmentations)
 *  - <class-i> is an integer corresponding to the class to be associated with elements that overlap with separate-mesh-i.
 */

int main(int argc, char* argv[])
{
    // extract command line arguments
    bool invalid_args = false;
    if (argc%2 != 0 || argc < 4)
        invalid_args = true;

    if (invalid_args)
    {
        std::cout << "INVALID ARGUMENTS!\nUsage:\n" << "  ./AssignElements <combined-mesh-filename> <separate-mesh-1> <class-1> <separate-mesh-2> <class-2> ..." << std::endl;
        return EXIT_FAILURE;
    }

    std::string combined_mesh_filename = argv[1];
    std::map<std::string, int> separate_meshes;
    try
    {
        for (int i = 2; i < argc; i+=2)
        {
            std::string filename = argv[i];
            int cl = atoi(argv[i+1]);
            separate_meshes[filename] = cl;
        }
    }
    catch(const std::exception& e)
    {
        invalid_args = true;
    }
    
    

    if (invalid_args)
    {
        std::cout << "INVALID ARGUMENTS!\nUsage:\n" << "./AssignElements <combined-mesh-filename> <separate-mesh-1> <class-1> <separate-mesh-2> <class-2> ..." << std::endl;
        return EXIT_FAILURE;
    }
    

    gmsh::initialize();

    // std::string combined_mesh_filename = "../resource/demos/trachea_virtuoso/cao_04_29_25_model1_decimated3_r.msh";
    // std::vector<std::string> class_mesh_filenames = {
    //     "../resource/demos/trachea_virtuoso/cao_04_29_25_model1_tumor.msh",
    //     "../resource/demos/trachea_virtuoso/cao_04_29_25_model1_trachea.msh"
    // };

    // create Embree scene
    Geometry::EmbreeScene embree_scene;

    Geometry::TetMesh combined_mesh = MeshUtils::loadTetMeshFromGmshFile(combined_mesh_filename);
    Vec3r combined_mesh_cm = combined_mesh.massCenter();

    // load combined mesh from file
    Config::MeshObjectConfig config(combined_mesh_filename, std::nullopt, std::nullopt, std::nullopt, false, false, false, Vec4r(0,0,0,0));
    Config::ObjectConfig obj_config("combined", "default", combined_mesh_cm, Vec3r::Zero(), Vec3r::Zero(), false, false, Config::ObjectRenderConfig());

    
    Sim::TetMeshObject combined_mesh_obj(&config, &obj_config);
    combined_mesh_obj.loadAndConfigureMesh();
    std::cout << "bounding box: " << combined_mesh_obj.mesh()->boundingBox().min.transpose() << " to " << combined_mesh_obj.mesh()->boundingBox().max.transpose() << std::endl;

    // add the combined mesh to the Embree scene
    // this will create a scene for the tetrahedra that we can do point queries on
    embree_scene.addObject(&combined_mesh_obj);
    embree_scene.update();

    std::vector<int> elem_classes(combined_mesh.numElements(), 0);
    for (const auto& [mesh_filename, class_int] : separate_meshes)
    // for (unsigned i = 0; i < class_mesh_filenames.size(); i++)
    {
        // load the class mesh from file - this will convert any .obj or .stl to .msh
        Geometry::TetMesh class_mesh = MeshUtils::loadTetMeshFromGmshFile(mesh_filename);

        std::cout << "bounding box: " << class_mesh.boundingBox().min.transpose() << " to " << class_mesh.boundingBox().max.transpose() << std::endl;

        int num_hits = 0;

        // go through each element of the combined mesh and see if its centroid is inside any tetrahedra of the class mesh
        // just brute force it for now
        for (int e = 0; e < combined_mesh.numElements(); e++)
        {
            const Eigen::Vector4i& element = combined_mesh.element(e);
            Vec3r c = (combined_mesh.vertex(element[0]) + combined_mesh.vertex(element[1]) + combined_mesh.vertex(element[2]) + combined_mesh.vertex(element[3]))/4.0;
            for (int class_e = 0; class_e < class_mesh.numElements(); class_e++)
            {
                const Eigen::Vector4i& class_element = class_mesh.element(class_e);
                const Vec3r& v1 = class_mesh.vertex(class_element[0]);
                const Vec3r& v2 = class_mesh.vertex(class_element[1]);
                const Vec3r& v3 = class_mesh.vertex(class_element[2]);
                const Vec3r& v4 = class_mesh.vertex(class_element[3]);

                float p[3] = {(float)c[0], (float)c[1], (float)c[2]};
                float f1[3] = {(float)v1[0], (float)v1[1], (float)v1[2]};
                float f2[3] = {(float)v2[0], (float)v2[1], (float)v2[2]};
                float f3[3] = {(float)v3[0], (float)v3[1], (float)v3[2]};
                float f4[3] = {(float)v4[0], (float)v4[1], (float)v4[2]};
                bool in_tet = Geometry::EmbreeTetMeshGeometry::isPointInTetrahedron(p, f1, f2, f3, f4);
                if (in_tet)
                {
                    num_hits++;
                    elem_classes[e] = class_int;
                    break;
                }
            }
        }

        std::cout << mesh_filename << " overlapped with " << num_hits << " tetrahedra!" << std::endl;
    }

    std::string out_filename = combined_mesh_filename.substr(0,combined_mesh_filename.length()-4) + "_element_classes.txt";
    std::ofstream out(out_filename);
    for (const auto& elem_class : elem_classes)
    {
        out << elem_class << "\n";
    }
    out.close();
}