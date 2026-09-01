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

    Config::ElasticMaterialConfig mat_config("material", 1000, 4e3, 0.45, 0.5, 0.2);
    Config::MaterialClassConfig mat_class_config("test", mat_config, Config::ObjectRenderConfig());
    Sim::MaterialClass mat(&mat_class_config);

    // test the individual constraint Hessians
    Vec3r p1(0,0,0);
    Vec3r p2(1,0,0);
    Vec3r p3(0,1,0);
    Vec3r p4(0,0,1);
    TombstoneVector<Vec3r> vertices_vec;
    int v1 = vertices_vec.push_back(p1);
    int v2 = vertices_vec.push_back(p2);
    int v3 = vertices_vec.push_back(p3);
    int v4 = vertices_vec.push_back(p4);
    Solver::HydrostaticConstraint hyd(v1, &vertices_vec, 1, v2, &vertices_vec, 1, v3, &vertices_vec, 1, v4, &vertices_vec, 1, mat.material());
    Solver::DeviatoricConstraint dev(v1, &vertices_vec, 1, v2, &vertices_vec, 1, v3, &vertices_vec, 1, v4, &vertices_vec, 1, mat.material());

    vertices_vec[0][0] = -3; vertices_vec[0][1] = 2.5;
    vertices_vec[1][0] = -10; vertices_vec[1][1] = 0.1; vertices_vec[1][2] = 0.1;

    typename Solver::HydrostaticConstraint::HessianMatType hyd_hessian_mat = hyd.hessian();
    typename Solver::DeviatoricConstraint::HessianMatType dev_hessian_mat = dev.hessian();

    std::cout << "\nHydrostatic Constraint Hessian:\n" << hyd_hessian_mat << std::endl;
    std::cout << "\nDeviatoric constraint Hessian:\n" << dev_hessian_mat << std::endl;


    Config::SimulationConfig sim_config;
    Sim::Simulation sim(&sim_config);
    sim.addMaterial(mat);

    const std::string single_tet_filename = "../resource/general/single.msh";
    const std::string bunny_filename = "../resource/general/stanford_bunny_medpoly.msh";
    const std::string cube_filename = "../resource/cube/cube2.msh";
    // std::vector<std::string> materials = {"material"};
    Config::FirstOrderXPBDMeshObjectConfig config(
        "test", "material", Vec3r(0,0,0.50), Vec3r(0,0,0), Vec3r(0,0,0), false, false,
        single_tet_filename, 1, std::nullopt, std::nullopt,
        false, true, true, Vec4r(1,1,1,1),
        {}, std::nullopt, std::nullopt,
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

    for (int i = 0; i < xpbd_mesh_obj->mesh()->numVertices(); i++)
    {
        xpbd_mesh_obj->mesh()->displaceVertex(i, Vec3r::Random());
        // std::cout << "Vertex " << i << ": " << v.transpose() << std::endl;
    }

    auto t1 = std::chrono::high_resolution_clock::now();
    MatXr stiffness_mat = xpbd_mesh_obj->stiffnessMatrix();
    auto t2 = std::chrono::high_resolution_clock::now();

    double new_ms = std::chrono::duration_cast<std::chrono::nanoseconds>(t2 - t1).count() / 1.0e6;

    std::cout << "\n\nStiffness matrix:\n" << stiffness_mat << std::endl;

    for (int i = 0; i < 10000; ++i)
    {
        VecXr x = VecXr::Random(stiffness_mat.rows());

        Real q = x.dot(stiffness_mat * x);

        if (q < 0)
        {
            std::cout << "NEGATIVE QUADRATIC FORM: "
                    << q << std::endl;
            break;
        }
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

    // for (int i = 0; i < 1500; i++)
    // {
    //     xpbd_mesh_obj->update();

    //     if (i%10 == 0)
    //     {
    //         std::cout << "Computing stiffness mat for step " << i/10 << std::endl;
    //         MatXr stiffness_mat = xpbd_mesh_obj->stiffnessMatrix();

    //         std::stringstream sfilename_ss;
    //         sfilename_ss << std::setw(6) << std::setfill('0') << "stiffness" << i/10 << ".txt";
    //         std::ofstream stiffness_ss(sfilename_ss.str());
    //         stiffness_ss << stiffness_mat;
    //         stiffness_ss.close();

    //         std::stringstream vfilename_ss;
    //         vfilename_ss << std::setw(6) << std::setfill('0') << "vertices" << i/10 << ".txt";
    //         std::ofstream vertices_ss(vfilename_ss.str());
    //         vertices_ss << xpbd_mesh_obj->mesh()->vertices().transpose();
    //         vertices_ss.close();
    //     }
    // }

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
    for (const auto& e : xpbd_mesh_obj->tetMesh()->elements())
        elements_ss << e.transpose()  << "\n";
    elements_ss.close();
    
    std::ofstream surface_faces_ss("surface_faces.txt");
    for (const auto& f : xpbd_mesh_obj->mesh()->faces())
        surface_faces_ss << f.transpose() << "\n";
    surface_faces_ss.close();
}