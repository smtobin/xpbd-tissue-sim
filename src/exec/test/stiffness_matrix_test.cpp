#include "utils/MeshUtils.hpp"
#include "config/simobject/XPBDMeshObjectConfig.hpp"
#include "simobject/XPBDObjectFactory.hpp"
#include "simobject/XPBDMeshObject.hpp"

#include "config/simulation/SimulationConfig.hpp"
#include "simulation/Simulation.hpp"

#include <fstream>
#include <string>
#include <iomanip>

int main()
{
    gmsh::initialize();

    Real x_size = 1.0;
    Real y_size = 1.0;
    Real z_size = 1.0;
    int num_subdivisions = 4;

    const std::string filename = std::to_string(x_size) + "x" + std::to_string(y_size) + "x" + std::to_string(z_size) + std::to_string(2) + ".msh";
    MeshUtils::createBeamMsh(filename, y_size, x_size, z_size, num_subdivisions);

    Config::ElasticMaterialConfig mat_config("material", 1000, 4e4, 0.45, 0.5, 0.2);
    ElasticMaterial mat(&mat_config);


    Config::SimulationConfig sim_config;
    Sim::Simulation sim(&sim_config);
    sim.addMaterial(mat);

    const std::string single_tet_filename = "../resource/general/single.msh";
    const std::string bunny_filename = "../resource/general/stanford_bunny_medpoly.msh";
    const std::string cube_filename = "../resource/cube/cube2.msh";
    std::vector<std::string> materials = {"material"};
    Config::FirstOrderXPBDMeshObjectConfig config(
        "test", Vec3r(0,0,0.50), Vec3r(0,0,0), Vec3r(0,0,0), false, false,
        filename, 1, std::nullopt,
        false, true, true, Vec4r(1,1,1,1),
        materials, std::nullopt,
        false, 10, 5, XPBDObjectSolverTypeEnum::GAUSS_SEIDEL,
        XPBDMeshObjectConstraintConfigurationEnum::STABLE_NEOHOOKEAN_COMBINED,
        XPBDSolverResidualPolicyEnum::NEVER,
        100000, false,
        Config::ObjectRenderConfig()
    );


    
    std::unique_ptr<Sim::FirstOrderXPBDMeshObject_Base> xpbd_mesh_obj = config.createObject(&sim);
    xpbd_mesh_obj->setup();

    Geometry::AABB bbox = xpbd_mesh_obj->boundingBox();
    std::vector<int> bottom_vertices = xpbd_mesh_obj->mesh()->getVerticesWithZ(bbox.min[2]);
    // std::vector<int> bottom_vertices = xpbd_mesh_obj->mesh()->getVerticesWithY(bbox.min[1]);
    for (const auto& v : bottom_vertices)
    {
        xpbd_mesh_obj->fixVertex(v);
    }
    // int v1 = xpbd_mesh_obj->mesh()->getClosestVertex(bbox.min);
    // int v2 = xpbd_mesh_obj->mesh()->getClosestVertex(Vec3r(bbox.min[0], bbox.max[1], bbox.min[2]));
    // int v3 = xpbd_mesh_obj->mesh()->getClosestVertex(Vec3r(bbox.max[0], bbox.max[1], bbox.min[2]));
    // int v4 = xpbd_mesh_obj->mesh()->getClosestVertex(Vec3r(bbox.max[0], bbox.min[1], bbox.min[2]));
    // xpbd_mesh_obj->fixVertex(v1);
    // xpbd_mesh_obj->fixVertex(v2);
    // xpbd_mesh_obj->fixVertex(v3);
    // xpbd_mesh_obj->fixVertex(v4);

    // perturb all the vertices a little bit
    // for (int i = 0; i < xpbd_mesh_obj->mesh()->numVertices(); i++)
    // {
    //     xpbd_mesh_obj->mesh()->displaceVertex(i, Vec3r::Random()/1000 + Vec3r::Random()/100000);
    // }

    for (int i = 0; i < 1500; i++)
    {
        xpbd_mesh_obj->update();

        if (i%10 == 0)
        {
            std::cout << "Computing stiffness mat for step " << i/10 << std::endl;
            MatXr stiffness_mat = xpbd_mesh_obj->stiffnessMatrix();

            std::stringstream sfilename_ss;
            sfilename_ss << std::setw(6) << std::setfill('0') << "stiffness" << i/10 << ".txt";
            std::ofstream stiffness_ss(sfilename_ss.str());
            stiffness_ss << stiffness_mat;
            stiffness_ss.close();

            std::stringstream vfilename_ss;
            vfilename_ss << std::setw(6) << std::setfill('0') << "vertices" << i/10 << ".txt";
            std::ofstream vertices_ss(vfilename_ss.str());
            vertices_ss << xpbd_mesh_obj->mesh()->vertices().transpose();
            vertices_ss.close();
        }
    }
    
    

    // std::cout << "\n\nStiffness matrix:\n" << stiffness_mat << std::endl;

    // Eigen::SelfAdjointEigenSolver<MatXr> eig;
    // eig.compute(stiffness_mat);
    // VecXr eigenvalues = eig.eigenvalues();
    // int num_le_zero = 0;
    // for (int i = 0; i < eigenvalues.size(); i++)
    // {
    //     if (eigenvalues[i] < 1e-6)
    //     {
    //         num_le_zero++;
    //     }
    // }
    // std::cout << "\nEigenvalues:\n" << eig.eigenvalues() << std::endl;

    // std::cout << "Num eigenvalues <= 0: " << num_le_zero << std::endl;
    // std::cout << "\nEigenvectors:\n" << eig.eigenvectors() << std::endl;

    // Eigen::LLT<MatXr> llt(new_stiffness_mat); // compute the Cholesky decomposition of A
    // if(llt.info() == Eigen::NumericalIssue)
    // {
    //     throw std::runtime_error("Possibly non semi-positive definitie matrix!");
    // }   

    

    std::ofstream elements_ss("elements.txt");
    elements_ss << xpbd_mesh_obj->tetMesh()->elements().transpose();
    elements_ss.close();
    
    std::ofstream surface_faces_ss("surface_faces.txt");
    surface_faces_ss << xpbd_mesh_obj->mesh()->faces().transpose();
    surface_faces_ss.close();
}