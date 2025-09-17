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

    /** Returns a pointer to face indices (3 consecutive indices make up a face). */
    // const int* faceIndices() const { return _mesh->faces().data(); }

    /** Returns a pointer to the vertices. */
    const float* vertices() const;

    /** Returns a pointer to the initial vertices of the mesh. */
    const float* initialVertices() const { return _initial_vertex_buffer.data(); }

    /** Copies mesh vertices to float vertex buffer. */
    void copyVertices();

    static void boundsFuncTriangle(const struct RTCBoundsFunctionArguments *args);
    static void intersectFuncTriangle(const RTCIntersectFunctionNArguments *args);
    static bool pointQueryFuncTriangle(RTCPointQueryFunctionArguments *args);

    static void boundsFuncTriangleInitialVertices(const struct RTCBoundsFunctionArguments *args);
    static void intersectFuncTriangleInitialVertices(const RTCIntersectFunctionNArguments *args);
    static bool pointQueryFuncTriangleInitialVertices(RTCPointQueryFunctionArguments *args);

    protected:

    static void _closestPointTriangle(const float p_in[3], const float v0[3], const float v1[3], const float v2[3], float p_out[3]);

    private:

    /** Performs a ray-triangle intersection test.
     * Returns whether or not there is a hit. If there is a hit, the distance and hit_point outputs are filled out.
     */
    static bool _rayTriangleIntersect(RTCRay* ray, RTCHit* hit,
    const float a_[3], const float b_[3], const float c_[3]);

    const Geometry::Mesh* _mesh;
    std::vector<float> _vertex_buffer;
    std::vector<float> _initial_vertex_buffer;
    unsigned _mesh_geom_id;

    unsigned _undeformed_mesh_geom_id;
    RTCScene _undeformed_scene;     // Embree scene specifically for this mesh that never gets updated - used for SDF-like queries
};

} // namespace Geometry

#endif // __EMBREE_MESH_GEOMETRY_HPP