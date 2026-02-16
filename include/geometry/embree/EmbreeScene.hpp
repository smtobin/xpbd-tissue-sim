#ifndef __EMBREE_MANAGER_HPP
#define __EMBREE_MANAGER_HPP

#include <embree4/rtcore.h>

#include "geometry/Mesh.hpp"
#include "geometry/TetMesh.hpp"

#include "geometry/embree/EmbreeMeshGeometry.hpp"
#include "geometry/embree/EmbreeTetMeshGeometry.hpp"
#include "geometry/embree/EmbreeVirtuosoArmGeometry.hpp"
#include "geometry/embree/EmbreeQueryStructs.hpp"

#include "simobject/MeshObject.hpp"
#include "simobject/VirtuosoArm.hpp"

// #include "common/SimulationTypeDefs.hpp"

#include <map>
#include <set>

namespace Sim
{
    class Object;
    class TetMeshObject;
}

namespace Geometry
{

/** A simple struct for a point cloud that has class information associated with it.
 */
struct PointsWithClass
{
    std::vector<Vec3r> points;
    std::string classification;
};

/** A class for interfacing with the Embree API.
 * TODO: add high-level collision scene for broad-phase collision detection
 * TODO: replace custom triangle-ray intersection function with native Embree version. Will require using triangle geometry instead of user geometry.
 * 
 * Supports: ray-tracing queries, closest-point queries, point-in-tetrahedron queries (for tetrahedral volume meshes)
 * 
 * Multiple Embree "scenes" are maintained:
 *   - _collision_scene is for broad-phase collision detection. It maintains a BVH of AABBs for objects added to the EmbreeScene. (TODO)
 *   - _ray_scene is for finer-grain ray-tracing. It maintains a BVH of triangle primitives for meshes added to the EmbreeScene. This is useful for tracing rays to generate a partial-view point cloud, for example.
 *   - each TetMeshObject (i.e. volumetric mesh) added to the EmbreeScene maintains its own scene used for point-in-tetrahedron queries.
 * 
 * When a surface mesh (MeshObject) is added to EmbreeScene, it is added to the ray-tracing scene (_ray_scene) and the collision scene (_collision_scene).
 * 
 * When a volumetric mesh (TetMeshObject) is added to EmbreeScene, it is added to the ray-tracing scene (_ray_scene), the collision scene (_collision_scene), and its own private scene for point-in-tetrahedron queries.
 * A TetMeshObject derives from MeshObject such that the surface mesh can easily be separated from the volumetric part, which will be faster for ray-tracing queries. 
 * 
*/
class EmbreeScene
{   
    public:

    EmbreeScene();

    ~EmbreeScene();

    /** Add a generic object to the EmbreeScene */
    template<typename ObjectType>
    void addObject(const ObjectType* obj_ptr)
    {
        std::cout << "Adding object..." << std::endl;
        unsigned geom_id = _setupObject(obj_ptr);
        std::cout << " new geom_id: " << geom_id << std::endl;

        if (geom_id != std::numeric_limits<unsigned>::max())
            _geomID_to_obj[geom_id] = obj_ptr;
    }

    /** Add a tetrahedral mesh object to the EmbreeScene */
    // void addObject(const Sim::TetMeshObject* obj);

    /** Add a surface mesh object to the EmbreeScene */
    // void addObject(const Sim::MeshObject* obj);

    /** Add a Virtuoso arm to the EmbreeScene */
    // void addObject(const Sim::VirtuosoArm* obj);

    /** Updates all Embree scenes. */
    void update();

    /** Updates the Embree scenes for a specific object. */
    void updateObject(const Sim::MeshObject* obj);

    /** Updates the Embree scenes for a specific object. */
    void updateObject(const Sim::TetMeshObject* obj);

    /** Updates only the ray-casting scene but not the individual object scenes. */
    void updateRayScene();

    /** Samples a point cloud from the surfaces of objects in the Embree scene from a given viewpoint.
     * Rays are cast from the view origin, with horizontal angles in [-hfov_deg/2, hfov_deg/2] and vertical angles in [-vfov_deg/2, vfov_deg/2].
     * @param origin : the view origin
     * @param view_dir : the "forward" direction of the view transform
     * @param up_dir : the "up" direction of the view transform - defines the vertical direction of the sampling (i.e. vertical FOV is defined in this direction)
     * @param hfov_deg : the horizontal FOV to sample, in degrees
     * @param vfov_deg : the vertical FOV to sample, in degrees
     * @param sample_density : the number of samples per degree
     */
    std::vector<Vec3r> partialViewPointCloud(const Vec3r& origin, const Vec3r& view_dir, const Vec3r& up_dir, Real hfov_deg, Real vfov_deg, Real sample_density) const;

    std::vector<PointsWithClass> partialViewPointCloudsWithClass(const Vec3r& origin, const Vec3r& view_dir, const Vec3r& up_dir, Real hfov_deg, Real vfov_deg, Real sample_density) const;

    /** Casts a ray and reports its intersection.
     * @param ray_origin : origin of the ray
     * @param ray_dir : direction of the ray (a unit vector)
     * @returns a struct with the intersection info
     */
    EmbreeRayHit castRay(const Vec3r& ray_origin, const Vec3r& ray_dir) const;

    /** Casts multiple rays and reports their intersections.
     * Selects the highest available SIMD parallelization (1, 4, 8, or 16) depending on what is available on the CPU.
     * @param origins : the origins of the rays
     * @param dirs : the directions of the rays (unit vectors)
     * @param hits (OUTPUT) : vector of structs with intersection info. This will be initially cleared.
     */
    void castRays(const std::vector<Vec3r>& origins, const std::vector<Vec3r>& dirs, std::vector<EmbreeRayHit>& hits) const;
    
    /** Finds the closest point on a surface mesh to the specified point.
     * @param point : the query point
     * @param obj_ptr : a pointer to the MeshObject that we should find the closest point on
     * @returns a struct with the closest point info
     */
    EmbreePQHit closestPointSurfaceMesh(const Vec3r& point, const Sim::MeshObject* obj_ptr) const;

    /** Finds the closest point on the surface of a tetrahedral mesh to the specified point. 
     * @param point : the query point
     * @param obj_ptr : a poitner to the TetMeshObject that we should find the closest point on
     * @returns a struct with the closest point info
    */
    EmbreePQHit closestPointTetMesh(const Vec3r& point, const Sim::TetMeshObject* obj_ptr) const;

    /** Finds the closest point on the surface of the undeformed tetrahedral mesh to the specified point. */
    EmbreePQHit closestPointUndeformedTetMesh(const Vec3r& point, const Sim::TetMeshObject* obj_ptr) const;

    /** Returns all the tetrahedra in a tetrahedral mesh that contain the specified point.
     * @param point : the query point
     * @param radius : the query radius - set to 0 for a strict point-in-tetrahedra query
     * @param obj_ptr : a pointer to the TetMeshObject to query
     * @returns the set of tetrahedra in the mesh that contain the specified point (can be empty)
     */
    std::set<EmbreePQHit> pointInTetrahedraQuery(const Vec3r& point, Real radius, const Sim::TetMeshObject* obj_ptr) const;

    /** Returns all the tetrahedra in a tetrahedral mesh that contain the specified vertex, ignoring all tetrahedra that share the vertex.
     * Used for checking for self-collisions in deformable tetrahedral meshes.
     * @param vertex_index : the index of the vertex in the tetrahedral mesh to test
     * @param obj_ptr : a pointer to the TetMeshObject that we are testing
     * @returns the set of tetrahedra in the mesh (excluding those that have the vertex in question as one of its vertices) that contain the specified vertex (can be empty)
     */
    std::set<EmbreePQHit> tetMeshSelfCollisionQuery(int vertex_index, const Sim::TetMeshObject* obj_ptr) const;

    private:
    /** Sets up a ray given the origin and direction. */
    RTCRayHit _createRayHit(const Vec3r& origin, const Vec3r& dir) const;

    /** Creates an EmbreeHit result struct from a RTCRayHit result */
    EmbreeRayHit _processRayHit(const RTCRayHit& rayhit, const Vec3r& origin, const Vec3r& dir) const;

    template<typename ObjectType>
    unsigned _setupObject(const ObjectType* /* obj_ptr */)
    {
        return std::numeric_limits<unsigned>::max();
    }
    
    unsigned _setupObject(const Sim::MeshObject* mesh_obj);
    unsigned _setupObject(const Sim::TetMeshObject* tet_mesh_obj);
    unsigned _setupObject(const Sim::VirtuosoArm* arm_obj);

    template<bool IsFirstOrder>
    unsigned _setupObject(const Sim::XPBDMeshObject_Base_<IsFirstOrder>* xpbd_obj)
    {
        // explicitly cast to TetMeshObject so the correct overload gets called
        return _setupObject((const Sim::TetMeshObject*)xpbd_obj);
    }

    /** Sets up the Embree geometry and scenes for a surface mesh. The primitive Embree triangle type is used.
     * This includes:
     *   - creating a dynamic RTCGeometry for the surface mesh and adding it to the ray-tracing scene
     *   - creating a static RTCGeometry for the undeformed (initial) surface mesh and creating a static undeformed scene just for this mesh
     */
    unsigned _setupEmbreeForSurfaceMesh(EmbreeMeshGeometry& mesh_geom);

    /** Sets up the Embree geometry and scenes for a tetrahedral (volume) mesh. A custom user geometry for tetrahedra is used.
     * This includes everything in _setupEmbreeForSurfaceMesh (using only the surface part of the volume mesh), and:
     *   - a dynamic RTCGeometry and scene specifically for point-in-tetrahedra queries (i.e. the scene just has this mesh in it)
     */
    unsigned _setupEmbreeForTetMesh(EmbreeTetMeshGeometry& tet_mesh_geom);

    EmbreePQHit _closestPointQuery(const Vec3r& point, const Sim::MeshObject* obj_ptr, const EmbreeMeshGeometry* geom) const;
    EmbreePQHit _closestPointQueryUndeformed(const Vec3r& point, const Sim::MeshObject* obj_ptr, const EmbreeMeshGeometry* geom) const;

    /** Embree device and scene */
    RTCDevice _device;
    RTCScene _collision_scene;
    RTCScene _ray_scene;
    
    /** maps object pointers to their Embree user geometries */ 
    std::map<const Sim::MeshObject*, EmbreeMeshGeometry*> _mesh_to_embree_geom;
    std::map<const Sim::TetMeshObject*, EmbreeTetMeshGeometry*> _tet_mesh_to_embree_geom;
    std::map<const Sim::VirtuosoArm*, EmbreeVirtuosoArmGeometry*> _arm_to_embree_geom;

    /** maps Embree geomID back to object pointers */
    std::map<unsigned, SimulationObjectConstPtrVariantType> _geomID_to_obj;

    /** Stores all the Embree user geometries */
    std::vector<EmbreeMeshGeometry> _embree_mesh_geoms;
    std::vector<EmbreeTetMeshGeometry> _embree_tet_mesh_geoms;
    std::vector<EmbreeVirtuosoArmGeometry> _embree_arm_geoms;

    bool _hasAVX512;
    bool _hasAVX;
    bool _hasSSE;

};

} // namespace Geometry

#endif // __EMBREE_MANAGER_HPP