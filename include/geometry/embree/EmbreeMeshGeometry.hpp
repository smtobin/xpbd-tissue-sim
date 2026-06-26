#ifndef __EMBREE_MESH_GEOMETRY_HPP
#define __EMBREE_MESH_GEOMETRY_HPP

#include <embree4/rtcore.h>

#include "geometry/Mesh.hpp"


namespace Geometry
{


/** User-defined Embree gometry to create BVH for surface meshes. */
class EmbreeMeshGeometry
{
    public:

    explicit EmbreeMeshGeometry(const Geometry::Mesh* mesh);

    ~EmbreeMeshGeometry();
    
    const Mesh* mesh() const { return _mesh; }
    unsigned meshGeomID() const { return _mesh_geom_id; }
    void setMeshGeomID(unsigned id) { _mesh_geom_id = id; }

    unsigned undeformedMeshGeomID() const { return _undeformed_mesh_geom_id; }
    void setUndeformedMeshGeomID(unsigned id) { _undeformed_mesh_geom_id = id; }

    RTCScene undeformedScene() const { return _undeformed_scene; }
    void setUndeformedScene(RTCScene scene) { _undeformed_scene = scene; }

    /** Returns the initial vertices of the mesh. */
    const Geometry::Mesh::vertices_vec_type& initialVertices() const { return _initial_vertices; }

    /** Updates the triangle geometry associated with this mesh.
     * Note: does not commit the geometry!
     */
    void updateSurfaceMeshGeometryBuffers(RTCGeometry geom);

    static void boundsFuncTriangle(const struct RTCBoundsFunctionArguments *args);
    static void intersectFuncTriangle(const RTCIntersectFunctionNArguments *args);
    static bool pointQueryFuncTriangle(RTCPointQueryFunctionArguments *args);

    static void boundsFuncTriangleInitialVertices(const struct RTCBoundsFunctionArguments *args);
    static void intersectFuncTriangleInitialVertices(const RTCIntersectFunctionNArguments *args);
    static bool pointQueryFuncTriangleInitialVertices(RTCPointQueryFunctionArguments *args);

    static Vec3r _closestPointTriangle(const Vec3r& p, const Vec3r& a, const Vec3r& b, const Vec3r& c);

    private:

    /** Performs a ray-triangle intersection test.
     * Returns whether or not there is a hit. If there is a hit, the distance and hit_point outputs are filled out.
     */
    static bool _rayTriangleIntersect(RTCRay* ray, RTCHit* hit, const Vec3r& a, const Vec3r& b, const Vec3r& c);

    const Geometry::Mesh* _mesh;
    Geometry::Mesh::vertices_vec_type _initial_vertices;
    unsigned _mesh_geom_id;

    unsigned _undeformed_mesh_geom_id;
    RTCScene _undeformed_scene;     // Embree scene specifically for this mesh that never gets updated - used for SDF-like queries
};

} // namespace Geometry

#endif // __EMBREE_MESH_GEOMETRY_HPP