#include "config/simobject/ObjectConfig.hpp"
#include "config/simobject/MeshObjectConfig.hpp"
#include "config/simobject/XPBDMeshObjectConfig.hpp"
#include "config/simulation/SimulationConfig.hpp"
#include "simulation/Simulation.hpp"

#include "geometry/TetMesh.hpp"
#include "geometry/embree/EmbreeScene.hpp"

#include "simobject/MeshObject.hpp"
#include "simobject/VirtuosoRobot.hpp"
#include "simobject/VirtuosoArm.hpp"
#include "simobject/RigidMeshObject.hpp"
#include "simobject/XPBDMeshObjectBase.hpp"
#include "simobject/RigidPrimitives.hpp"

#include "utils/MeshUtils.hpp"

#include <embree4/rtcore.h>

#include <iostream>
#include <limits>
#include <set>
#include <chrono>

/* -------------------------------------------------------------------------- */

int main()
{
    gmsh::initialize();
    // create dummy simulation
    Config::SimulationConfig sim_config;
    Sim::Simulation dummy_sim(&sim_config);

    // load mesh
    // Geometry::TetMesh tet_mesh = MeshUtils::loadTetMeshFromGmshFile("../resource/demos/trachea_virtuoso/tracheal_tumor_v2_refined.msh");
    Config::MeshObjectConfig mesh_config("../resource/cube/cube2.msh", 2, std::nullopt, std::nullopt,
        false, false, true, Vec4r(0,0,0,0));
    Config::ObjectConfig object_config("test", "default", Vec3r(0,0,5), Vec3r(0,0,0), Vec3r(0,0,0), true, false, Config::ObjectRenderConfig());
    Config::XPBDMeshObjectConfig xpbd_config(object_config, mesh_config);
    auto mesh_obj = xpbd_config.createObject(&dummy_sim);

    // Sim::TetMeshObject mesh_obj(&mesh_config, &object_config);
    mesh_obj->setup();
    // Geometry::TetMesh tet_mesh = MeshUtils::loadTetMeshFromGmshFile("../resource/general/single.msh");
    // Geometry::TetMesh tet_mesh = MeshUtils::loadTetMeshFromGmshFile("../resource/cube/cube16.msh");


    Config::VirtuosoArmConfig arm1_config;
    auto arm1 = arm1_config.createObject(&dummy_sim);
    arm1->setup();

    Config::VirtuosoArmConfig arm2_config;
    auto arm2 = arm2_config.createObject(&dummy_sim);
    arm2->setup();

    // create the EmbreeScene object to interface with Embree
    Geometry::EmbreeScene embree_scene;

    // add object(s) to EmbreeScene
    embree_scene.addObject(arm1.get());
    embree_scene.addObject(arm2.get());
    // embree_scene.addObject(mesh_obj.get());
    
    

    // translate object and update the EmbreeScene
    Geometry::Mesh::vertices_vec_type initial_vertices = mesh_obj->mesh()->vertices();

    Geometry::AABB initial_bbox = mesh_obj->mesh()->boundingBox();
    std::cout << "Initial mesh bounding box:\n(" << initial_bbox.min.transpose() << ") to (" << initial_bbox.max.transpose() << ")" << std::endl;
    
    mesh_obj->refinedTetMesh()->refineElement(0,2);

    // move the mesh
    Vec3r translation(0,0,-5);
    mesh_obj->mesh()->moveTogether(translation);

    // move the arm
    arm1->setBasePosition(Vec3r(0,0,2));
    arm1->update();

    embree_scene.update();

    Geometry::AABB bbox = mesh_obj->mesh()->boundingBox();
    std::cout << "Mesh bounding box:\n(" << bbox.min.transpose() << ") to (" << bbox.max.transpose() << ")" << std::endl;

    // point-in-tetrahedron query
    const Vec3r query_point(0.0,0.0,0.0);
    std::set<Geometry::EmbreePQHit> result = embree_scene.pointInTetrahedraQuery(query_point, 0.0, mesh_obj.get());

    std::cout << "\n=== Results for point-in-tet query for query point (" << query_point[0] << ", " << query_point[1] << ", " << query_point[2] << ") ===" << std::endl;
    for (const auto& hit : result)
    {
        std::cout << " Hit! Details: " << std::endl;
        std::cout << "   Tet index: " << hit.prim_index << std::endl;

        const Eigen::Vector4i& elem = mesh_obj->tetMesh()->element(hit.prim_index);
        const Vec3r& v1 = mesh_obj->tetMesh()->vertex(elem[0]);
        const Vec3r& v2 = mesh_obj->tetMesh()->vertex(elem[1]);
        const Vec3r& v3 = mesh_obj->tetMesh()->vertex(elem[2]);
        const Vec3r& v4 = mesh_obj->tetMesh()->vertex(elem[3]);

        std::cout << "Elem v1: " << v1[0] << ", " << v1[1] << ", " << v1[2] << std::endl;
        std::cout << "Elem v2: " << v2[0] << ", " << v2[1] << ", " << v2[2] << std::endl;
        std::cout << "Elem v3: " << v3[0] << ", " << v3[1] << ", " << v3[2] << std::endl;
        std::cout << "Elem v4: " << v4[0] << ", " << v4[1] << ", " << v4[2] << std::endl;
    }

    // closest point query
    const Vec3r cp_query_point(0.7, 0.1, 0.1);
    Geometry::EmbreePQHit cp_result = embree_scene.closestPointTetMesh(cp_query_point, mesh_obj.get());
    
    std::cout << "\n=== Results for closest-point query for query point (" << cp_query_point[0] << ", " << cp_query_point[1] << ", " << cp_query_point[2] << ") ===" << std::endl;
    if (cp_result.obj)
    {
        const Eigen::Vector3i& face = mesh_obj->tetMesh()->face(cp_result.prim_index);
        const Vec3r& v1 = mesh_obj->tetMesh()->vertex(face[0]);
        const Vec3r& v2 = mesh_obj->tetMesh()->vertex(face[1]);
        const Vec3r& v3 = mesh_obj->tetMesh()->vertex(face[2]);

        std::cout << "Face index: " << cp_result.prim_index << std::endl;
        std::cout << "Face v1: " << v1[0] << ", " << v1[1] << ", " << v1[2] << std::endl;
        std::cout << "Face v2: " << v2[0] << ", " << v2[1] << ", " << v2[2] << std::endl;
        std::cout << "Face v3: " << v3[0] << ", " << v3[1] << ", " << v3[2] << std::endl;

        std::cout << "closest point: " << cp_result.hit_point[0] << ", " << cp_result.hit_point[1] << ", " << cp_result.hit_point[2] << std::endl;
    }

    // closest point query on undeformed mesh
    const Vec3r cp_query_point2 = cp_query_point - translation;
    Geometry::EmbreePQHit cp_result2 = embree_scene.closestPointUndeformedTetMesh(cp_query_point2, mesh_obj.get());
    std::cout << "\n=== Results for closest-point query (undeformed mesh) for query point (" << cp_query_point2[0] << ", " << cp_query_point2[1] << ", " << cp_query_point2[2] << ") ===" << std::endl;
    std::cout << "Face index: " << cp_result2.prim_index << std::endl;
    if (cp_result2.obj)
    {
        const Eigen::Vector3i& face2 = mesh_obj->tetMesh()->face(cp_result2.prim_index);
        const Vec3r& v12 = initial_vertices[face2[0]];
        const Vec3r& v22 = initial_vertices[face2[1]];
        const Vec3r& v32 = initial_vertices[face2[2]];

        
        std::cout << "Face v1: " << v12[0] << ", " << v12[1] << ", " << v12[2] << std::endl;
        std::cout << "Face v2: " << v22[0] << ", " << v22[1] << ", " << v22[2] << std::endl;
        std::cout << "Face v3: " << v32[0] << ", " << v32[1] << ", " << v32[2] << std::endl;

        std::cout << "closest point: " << cp_result2.hit_point[0] << ", " << cp_result2.hit_point[1] << ", " << cp_result2.hit_point[2] << std::endl;
    }
    // ray-tracing
    const Vec3r ray_origin(100, 0.38, 0.31);
    const Vec3r ray_dir(-1, 0, 0);
    Geometry::EmbreeRayHit rt_result = embree_scene.castRay(ray_origin, ray_dir);
    std::cout << "\n=== Results for ray query for ray (" << ray_origin[0] << ", " << ray_origin[1] << ", " << ray_origin[2] << ") with dir (" << ray_dir[0] << ", " << ray_dir[1] << ", " << ray_dir[2] << ") ===" << std::endl;
    
    std::visit([&](auto* obj) {
        if (!obj)
        {
            std::cout << "  No hit detected!" << std::endl;
            return;
        }
        
        using ObjectType = std::remove_pointer_t<std::decay_t<decltype(obj)>>;
        if constexpr(std::is_base_of_v<Sim::MeshObject, ObjectType>)
        {
            const Eigen::Vector3i& face = obj->mesh()->face(rt_result.prim_index);
            const Vec3r& v1 = obj->mesh()->vertex(face[0]);
            const Vec3r& v2 = obj->mesh()->vertex(face[1]);
            const Vec3r& v3 = obj->mesh()->vertex(face[2]);

            std::cout << "Face v1: " << v1[0] << ", " << v1[1] << ", " << v1[2] << std::endl;
            std::cout << "Face v2: " << v2[0] << ", " << v2[1] << ", " << v2[2] << std::endl;
            std::cout << "Face v3: " << v3[0] << ", " << v3[1] << ", " << v3[2] << std::endl;

            std::cout << "hit point: " << rt_result.hit_point[0] << ", " << rt_result.hit_point[1] << ", " << rt_result.hit_point[2] << std::endl;
        }
    }, rt_result.obj);

    // ray-tracing w/ collision w/ Virtuoso arm
    const Vec3r ray_origin2(100, 0, 2.001);
    const Vec3r ray_dir2(-1, 0, 0);
    Geometry::EmbreeRayHit rt_result2 = embree_scene.castRay(ray_origin2, ray_dir2);
    std::cout << "\n=== Results for ray query for ray (" << ray_origin2.transpose() << ") with dir (" << ray_dir2.transpose() << ") ===" << std::endl;
    std::visit([&](auto* obj) {
        if (!obj)
        {
            std::cout << "  No hit detected!" << std::endl;
            return;
        }
        
        std::cout << "hit point: (" << rt_result2.hit_point.transpose() << ")" << std::endl;
    }, rt_result2.obj);
    
    

    return 0;
}