#include "geometry/embree/EmbreeScene.hpp"
#include "geometry/embree/EmbreeQueryStructs.hpp"

#include <math.h>

#include "simobject/XPBDMeshObject.hpp"

namespace Geometry
{

EmbreeScene::EmbreeScene()
    : _device(nullptr), _collision_scene(nullptr), _ray_scene(nullptr)
{
    _device = rtcNewDevice(NULL);

    if (!_device)
    {
        std::cout << "Error " << rtcGetDeviceError(NULL) << ": cannot create device" << std::endl;
        assert(0);
    }

    _ray_scene = rtcNewScene(_device);
    rtcSetSceneFlags(_ray_scene, RTC_SCENE_FLAG_DYNAMIC);
}

EmbreeScene::~EmbreeScene()
{
    if (_collision_scene)
        rtcReleaseScene(_collision_scene);

    if (_ray_scene)
        rtcReleaseScene(_ray_scene);

    rtcReleaseDevice(_device);
    
}

void EmbreeScene::_setupEmbreeForSurfaceMesh(EmbreeMeshGeometry& mesh_geom)
{
    /** Ray-casting scene */

    // create Embree geometry (using triangle primitive type)
    RTCGeometry rtc_geom = rtcNewGeometry(_device, RTC_GEOMETRY_TYPE_TRIANGLE);
    // update geometry buffers (copy vertices and faces into buffers)
    mesh_geom.updateSurfaceMeshGeometryBuffers(rtc_geom);
    // attach the geometry, and store its ID in the EmbreeMeshGeometry object
    mesh_geom.setMeshGeomID( rtcAttachGeometry(_ray_scene, rtc_geom) );

    // commit geometry to scene
    rtcCommitGeometry(rtc_geom);
    rtcCommitScene(_ray_scene);     // this will build BVH
    rtcReleaseGeometry(rtc_geom);


    /**  Undeformed scene */

    // create a new scene for the mesh exclusively for undeformed mesh queries (this scene is static)
    RTCScene undeformed_mesh_scene = rtcNewScene(_device);
    mesh_geom.setUndeformedScene(undeformed_mesh_scene);

    // add Embree user geometry to static scene for undeformed mesh
    RTCGeometry rtc_undeformed_geom = rtcNewGeometry(_device, RTC_GEOMETRY_TYPE_TRIANGLE);
    // update geometry buffers (copy vertices and faces into buffers)
    mesh_geom.updateSurfaceMeshGeometryBuffers(rtc_undeformed_geom);
    // attach the geometry, and store its ID in the EmbreeMeshGeometry object
    mesh_geom.setUndeformedMeshGeomID( rtcAttachGeometry(mesh_geom.undeformedScene(), rtc_undeformed_geom) );

    // set BVH build quality to REFIT - this will update the BVH rather than do a complete rebuild
    rtcSetGeometryBuildQuality(rtc_geom, RTC_BUILD_QUALITY_REFIT);

    // set BVH build quality to MEDIUM for the static scene
    rtcSetGeometryBuildQuality(rtc_undeformed_geom, RTC_BUILD_QUALITY_MEDIUM);

    rtcCommitGeometry(rtc_undeformed_geom);
    rtcCommitScene(undeformed_mesh_scene);
    rtcReleaseGeometry(rtc_undeformed_geom);
}


void EmbreeScene::_setupEmbreeForTetMesh(EmbreeTetMeshGeometry& tet_mesh_geom)
{

    /** Setup for surface mesh part of the volume mesh */
    _setupEmbreeForSurfaceMesh(tet_mesh_geom);



    /** Point-in-query scene */

    // create a new scene for the TetMesh exclusively for point-in-tetrahedra queries
    RTCScene tet_mesh_scene = rtcNewScene(_device);
    rtcSetSceneFlags(tet_mesh_scene, RTC_SCENE_FLAG_DYNAMIC);
    tet_mesh_geom.setTetScene(tet_mesh_scene);
    
    // create a user-geometry type
    RTCGeometry rtc_tet_mesh_geom = rtcNewGeometry(_device, RTC_GEOMETRY_TYPE_USER);
    tet_mesh_geom.setTetMeshGeomID( rtcAttachGeometry(tet_mesh_scene, rtc_tet_mesh_geom) );

    rtcSetGeometryBuildQuality(rtc_tet_mesh_geom, RTC_BUILD_QUALITY_REFIT);

    // set custom user data
    rtcSetGeometryUserPrimitiveCount(rtc_tet_mesh_geom, tet_mesh_geom.tetMesh()->numElements());
    rtcSetGeometryUserData(rtc_tet_mesh_geom, &tet_mesh_geom);

    // set custom callbacks
    rtcSetGeometryBoundsFunction(rtc_tet_mesh_geom, EmbreeTetMeshGeometry::boundsFuncTetrahedra, &tet_mesh_geom);
    rtcSetGeometryIntersectFunction(rtc_tet_mesh_geom, EmbreeTetMeshGeometry::intersectFuncTetrahedra);
    rtcSetGeometryPointQueryFunction(rtc_tet_mesh_geom, EmbreeTetMeshGeometry::pointQueryFuncTetrahedra);

    // commit geometry and scene
    rtcCommitGeometry(rtc_tet_mesh_geom);
    rtcCommitScene(tet_mesh_scene);
    rtcReleaseGeometry(rtc_tet_mesh_geom);
}

void EmbreeScene::addObject(const Sim::MeshObject* obj_ptr)
{
    // make sure that object has not already been added to Embree scene
    if (_mesh_to_embree_geom.count(obj_ptr) > 0)
        assert(0 && "Object has already been added to Embree scene!");

    // create new EmbreeMeshGeometry for the object
    _embree_mesh_geoms.emplace_back(obj_ptr->mesh());
    EmbreeMeshGeometry& geom = _embree_mesh_geoms.back();

    _mesh_to_embree_geom[obj_ptr] = &geom;

    // set up Embree scenes and geometries for the surface mesh
    _setupEmbreeForSurfaceMesh(geom);
    _geomID_to_mesh_obj[geom.meshGeomID()] = obj_ptr;
    
}

void EmbreeScene::addObject(const Sim::TetMeshObject* obj_ptr)
{
    // make sure that object has not already been added to Embree scene
    if (_tet_mesh_to_embree_geom.count(obj_ptr) > 0)
        assert(0 && "Object has already been added to Embree scene!");
    
    // create new EmbreeTetMeshGeometry for the object
    _embree_tet_mesh_geoms.emplace_back(obj_ptr->tetMesh());
    EmbreeTetMeshGeometry& geom = _embree_tet_mesh_geoms.back();

    // store the new user geometry by its pointer in the maps
    _tet_mesh_to_embree_geom[obj_ptr] = &geom;
    _mesh_to_embree_geom[obj_ptr] = &geom;

    // set up Embree scenes and geometries for the tet mesh object
    _setupEmbreeForTetMesh(geom);
    _geomID_to_mesh_obj[geom.meshGeomID()] = obj_ptr;
    
}


void EmbreeScene::update()
{
    // update all surface meshes
    // (just the ray-scene)
    for (auto& geom : _embree_mesh_geoms)
    {
        RTCGeometry rtc_geom = rtcGetGeometry(_ray_scene, geom.meshGeomID());
        geom.updateSurfaceMeshGeometryBuffers(rtc_geom);
        rtcCommitGeometry(rtc_geom);
    }

    // update all tet meshes
    // (the ray-scene and the point-in-tet scene)
    for (auto& geom : _embree_tet_mesh_geoms)
    {
        // update the ray casting scene
        RTCGeometry rtc_mesh_geom = rtcGetGeometry(_ray_scene, geom.meshGeomID());
        geom.updateSurfaceMeshGeometryBuffers(rtc_mesh_geom);
        rtcCommitGeometry(rtc_mesh_geom);

        // update the point-in-tet query scene
        RTCGeometry rtc_tet_mesh_geom = rtcGetGeometry(geom.tetScene(), geom.tetMeshGeomID());
        rtcCommitGeometry(rtc_tet_mesh_geom);
        rtcCommitScene(geom.tetScene());
    }

    // commit the ray scene once we've updated all the objects
    rtcCommitScene(_ray_scene);
}

void EmbreeScene::updateObject(const Sim::MeshObject* /*mesh_obj*/)
{
    // don't need to do anything here since there are no dynamic scenes that EmbreeMeshGeometry owns 
}

void EmbreeScene::updateObject(const Sim::TetMeshObject* tet_mesh_obj)
{
    // update the point-in-tet query scene
    EmbreeTetMeshGeometry* geom = _tet_mesh_to_embree_geom[tet_mesh_obj];
    RTCGeometry rtc_tet_mesh_geom = rtcGetGeometry(geom->tetScene(), geom->tetMeshGeomID());
    rtcCommitGeometry(rtc_tet_mesh_geom);
    rtcCommitScene(geom->tetScene());
}

void EmbreeScene::updateRayScene()
{
    // update buffers for surface meshes
    for (auto& geom : _embree_mesh_geoms)
    {
        RTCGeometry rtc_geom = rtcGetGeometry(_ray_scene, geom.meshGeomID());
        geom.updateSurfaceMeshGeometryBuffers(rtc_geom);
        rtcCommitGeometry(rtc_geom);
    }

    // update buffers for tet meshes (just the surface part)
    for (auto& geom : _embree_tet_mesh_geoms)
    {
        // only update the ray scene geometry (not the tetrahedral mesh geometry)
        RTCGeometry rtc_mesh_geom = rtcGetGeometry(_ray_scene, geom.meshGeomID());
        geom.updateSurfaceMeshGeometryBuffers(rtc_mesh_geom);
        rtcCommitGeometry(rtc_mesh_geom);
    }

    // commit the ray scene (rebuild the BVH) after we've updated all the buffers
    rtcCommitScene(_ray_scene);
}

EmbreeHit EmbreeScene::castRay(const Vec3r& ray_origin, const Vec3r& ray_dir) const
{
    RTCRayHit rayhit;
    rayhit.ray.org_x = ray_origin[0];
    rayhit.ray.org_y = ray_origin[1];
    rayhit.ray.org_z = ray_origin[2];

    rayhit.ray.dir_x = ray_dir[0];
    rayhit.ray.dir_y = ray_dir[1];
    rayhit.ray.dir_z = ray_dir[2];

    rayhit.ray.tnear = 0;
    rayhit.ray.tfar = std::numeric_limits<float>::infinity();
    rayhit.ray.flags = 0;
    rayhit.ray.time = 0;
    rayhit.ray.mask = -1;

    rayhit.hit.geomID = RTC_INVALID_GEOMETRY_ID;
    rayhit.hit.instID[0] = RTC_INVALID_GEOMETRY_ID;

    rtcIntersect1(_ray_scene, &rayhit, nullptr);

    // check if we have a hit
    if (rayhit.hit.geomID != RTC_INVALID_GEOMETRY_ID)
    {
        const Sim::MeshObject* obj = _geomID_to_mesh_obj.at(rayhit.hit.geomID);
        float t = rayhit.ray.tfar;
        

        EmbreeHit hit;
        hit.obj = obj;
        hit.prim_index = rayhit.hit.primID;
        // hit.hit_point = v1*rayhit.hit.u + v2*rayhit.hit.v + v3*(1 - rayhit.hit.u - rayhit.hit.v);
        hit.hit_point = ray_origin + t*ray_dir;
        return hit;
    }
    else
    {
        EmbreeHit hit;
        hit.obj = nullptr;
        return hit;
    }
}
    
EmbreeHit EmbreeScene::closestPointSurfaceMesh(const Vec3r& point, const Sim::MeshObject* obj_ptr) const
{
    const EmbreeMeshGeometry* geom = _mesh_to_embree_geom.at(obj_ptr);
    return _closestPointQuery(point, obj_ptr, geom);
}

EmbreeHit EmbreeScene::closestPointTetMesh(const Vec3r& point, const Sim::TetMeshObject* obj_ptr) const
{
    const EmbreeTetMeshGeometry* geom = _tet_mesh_to_embree_geom.at(obj_ptr);
    return _closestPointQuery(point, obj_ptr, geom);
}

EmbreeHit EmbreeScene::closestPointUndeformedTetMesh(const Vec3r& point, const Sim::TetMeshObject* obj_ptr) const
{
    const EmbreeTetMeshGeometry* geom = _tet_mesh_to_embree_geom.at(obj_ptr);
    return _closestPointQueryUndeformed(point, obj_ptr, geom);
}

EmbreeHit EmbreeScene::_closestPointQuery(const Vec3r& point, const Sim::MeshObject* obj_ptr, const EmbreeMeshGeometry* geom) const
{
    EmbreeClosestPointQueryUserData point_query_data;
    point_query_data.obj_ptr = obj_ptr;
    point_query_data.geom = geom;
    point_query_data.point = point;

    RTCPointQuery query;
    query.x = point[0];
    query.y = point[1];
    query.z = point[2];
    query.radius = std::numeric_limits<float>::infinity(); // the query radius will get refined as we go

    RTCPointQueryContext context;
    rtcInitPointQueryContext(&context);
    rtcPointQuery(_ray_scene, &query, &context, EmbreeMeshGeometry::pointQueryFuncTriangle, &point_query_data);

    return point_query_data.result;

}

EmbreeHit EmbreeScene::_closestPointQueryUndeformed(const Vec3r& point, const Sim::MeshObject* obj_ptr, const EmbreeMeshGeometry* geom) const
{
    EmbreeClosestPointQueryUserData point_query_data;
    point_query_data.obj_ptr = obj_ptr;
    point_query_data.geom = geom;
    point_query_data.point = point;

    RTCPointQuery query;
    query.x = point[0];
    query.y = point[1];
    query.z = point[2];
    query.radius = std::numeric_limits<float>::infinity(); // the query radius will get refined as we go

    RTCPointQueryContext context;
    rtcInitPointQueryContext(&context);
    rtcPointQuery(geom->undeformedScene(), &query, &context, EmbreeMeshGeometry::pointQueryFuncTriangleInitialVertices, &point_query_data);

    return point_query_data.result;

}

std::set<EmbreeHit> EmbreeScene::pointInTetrahedraQuery(const Vec3r& point, Real radius, const Sim::TetMeshObject* obj_ptr) const
{
    const EmbreeTetMeshGeometry* geom = _tet_mesh_to_embree_geom.at(obj_ptr);
    EmbreePointQueryUserData point_query_data;
    point_query_data.obj_ptr = obj_ptr;
    point_query_data.geom = geom;
    point_query_data.vertex_ind = -1;
    point_query_data.radius = radius;
    point_query_data.point = point;

    RTCPointQuery query;
    query.x = point[0];
    query.y = point[1];
    query.z = point[2];
    query.radius = radius;

    RTCPointQueryContext context;
    rtcInitPointQueryContext(&context);
    rtcPointQuery(geom->tetScene(), &query, &context, nullptr, &point_query_data);

    return point_query_data.result;
}

std::set<EmbreeHit> EmbreeScene::tetMeshSelfCollisionQuery(int vertex_index, const Sim::TetMeshObject* obj_ptr) const
{
    const EmbreeTetMeshGeometry* geom = _tet_mesh_to_embree_geom.at(obj_ptr);
    EmbreePointQueryUserData point_query_data;
    point_query_data.obj_ptr = obj_ptr;
    point_query_data.geom = geom;
    point_query_data.vertex_ind = vertex_index;
    
    const Vec3r& vertex = obj_ptr->mesh()->vertex(vertex_index);
    point_query_data.point = vertex;

    RTCPointQuery query;
    query.x = vertex[0];
    query.y = vertex[1];
    query.z = vertex[2];
    query.radius = 0.0f;

    RTCPointQueryContext context;
    rtcInitPointQueryContext(&context);
    rtcPointQuery(geom->tetScene(), &query, &context, nullptr, &point_query_data);

    return point_query_data.result;
}

std::vector<Vec3r> EmbreeScene::partialViewPointCloud(const Vec3r& origin, const Vec3r& view_dir, const Vec3r& up_dir, Real hfov_deg, Real vfov_deg, Real sample_density) const
{
    // calculate "right" direction from view direction and up direction
    const Vec3r right_dir = up_dir.cross(view_dir);
    Mat3r R_camera;
    R_camera.col(0) = right_dir;    // x-axis is "right" direction
    R_camera.col(1) = up_dir;       // y-axis is "up" direction
    R_camera.col(2) = view_dir;     // z-axis is "view" direction

    std::vector<Vec3r> hit_points;
    hit_points.reserve(hfov_deg * vfov_deg * sample_density * sample_density);

    // create rays by sampling spherical coordinates and transforming into local frame
    Real angle_increment = 1.0/sample_density;

    // std::cout << "hfov: " << hfov_deg << " vfov: " << vfov_deg << " angle_incr: " << angle_increment<< std::endl;
    for (Real h_angle = -hfov_deg/2.0; h_angle < hfov_deg/2.0; h_angle += angle_increment)
    {
        for (Real v_angle = -vfov_deg/2.0; v_angle < vfov_deg/2.0; v_angle += angle_increment)
        {
            Real x_local = std::sin(h_angle * M_PI/180.0) * std::cos(v_angle * M_PI/180.0);
            Real y_local = std::sin(v_angle * M_PI/180.0);
            Real z_local = std::cos(h_angle * M_PI/180.0) * std::cos(v_angle * M_PI/180.0);
            const Vec3r dir_local(x_local, y_local, z_local);
            const Vec3r ray_dir = R_camera * dir_local;

            // std::cout << "Ray dir: " << ray_dir[0] << ", " << ray_dir[1] << ", " << ray_dir[2] << std::endl;
            EmbreeHit hit = castRay(origin, ray_dir);
            if (hit.obj)
            {
                // std::cout << "Hit!" << std::endl;
                hit_points.push_back(hit.hit_point);
            }
        }
    }

    return hit_points;
}

std::vector<PointsWithClass> EmbreeScene::partialViewPointCloudsWithClass(const Vec3r& origin, const Vec3r& view_dir, const Vec3r& up_dir, Real hfov_deg, Real vfov_deg, Real sample_density) const
{
    // calculate "right" direction from view direction and up direction
    const Vec3r right_dir = up_dir.cross(view_dir);
    Mat3r R_camera;
    R_camera.col(0) = right_dir;    // x-axis is "right" direction
    R_camera.col(1) = up_dir;       // y-axis is "up" direction
    R_camera.col(2) = view_dir;     // z-axis is "view" direction

    // create rays by sampling spherical coordinates and transforming into local frame
    Real angle_increment = 1.0/sample_density;

    // keep track of which classes we have put where - maps classification to index in the vector
    std::unordered_map<std::string, int> class_to_vector_index;

    // store point clouds
    std::vector<PointsWithClass> point_clouds;

    for (Real h_angle = -hfov_deg/2.0; h_angle < hfov_deg/2.0; h_angle += angle_increment)
    {
        for (Real v_angle = -vfov_deg/2.0; v_angle < vfov_deg/2.0; v_angle += angle_increment)
        {
            Real x_local = std::sin(h_angle * M_PI/180.0) * std::cos(v_angle * M_PI/180.0);
            Real y_local = std::sin(v_angle * M_PI/180.0);
            Real z_local = std::cos(h_angle * M_PI/180.0) * std::cos(v_angle * M_PI/180.0);
            const Vec3r dir_local(x_local, y_local, z_local);
            const Vec3r ray_dir = R_camera * dir_local;

            EmbreeHit hit = castRay(origin, ray_dir);
            if (hit.obj)
            {
                // get the class of the face that we hit
                std::string classification = "";
                if (hit.obj->mesh()->hasFaceProperty<int>("class"))
                {
                    int class_num = hit.obj->mesh()->getFaceProperty<int>("class").get(hit.prim_index);
                    /** TODO: dynamic_cast = yucky, can we do better?
                     * 
                     * 
                     */
                    if (auto xpbd_obj = dynamic_cast<const Sim::XPBDMeshObject_Base*>(hit.obj))
                    {
                        classification = xpbd_obj->materialClasses()[class_num]->label();
                    }
                    else if (auto xpbd_obj = dynamic_cast<const Sim::FirstOrderXPBDMeshObject_Base*>(hit.obj))
                    {
                        classification = xpbd_obj->materialClasses()[class_num]->label();
                    }
                    else if (auto obj = dynamic_cast<const Sim::Object*>(hit.obj))
                    {
                        classification = obj->materialClass()->label();
                    }
                }
                else
                {
                    if (auto obj = dynamic_cast<const Sim::Object*>(hit.obj))
                    {
                        classification = obj->materialClass()->label();
                    } 
                }
                    

                // put the point in the appropriate vector
                auto map_it = class_to_vector_index.find(classification);
                if (map_it != class_to_vector_index.end())  // we already have a points vector started for this class
                {
                    point_clouds[map_it->second].points.push_back(hit.hit_point);
                }
                else
                {
                    // create a point cloud and conservatively reserve space for the points that will go in it
                    PointsWithClass point_cloud;
                    point_cloud.classification = classification;
                    point_cloud.points.reserve(hfov_deg * vfov_deg * sample_density * sample_density);
                    point_cloud.points.push_back(hit.hit_point);

                    point_clouds.push_back(std::move(point_cloud));

                    // add an entry to the map so we know which index in the vector is associated with this class
                    class_to_vector_index[classification] = point_clouds.size()-1;
                }
            }
        }
    }

    return point_clouds;
}

} // namespace Geometry