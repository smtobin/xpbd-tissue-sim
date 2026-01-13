#ifndef __EMBREE_TET_MESH_GEOMETRY_HPP
#define __EMBREE_TET_MESH_GEOMETRY_HPP

#include "geometry/embree/EmbreeMeshGeometry.hpp"
#include "geometry/TetMesh.hpp"

namespace Geometry
{

/** User-defined Embree geometry that is used to create the BVH */
class EmbreeTetMeshGeometry : public EmbreeMeshGeometry
{
    public:

    explicit EmbreeTetMeshGeometry(const Geometry::TetMesh* tet_mesh);

    ~EmbreeTetMeshGeometry();

    const TetMesh* tetMesh() const { return _tet_mesh; }
    unsigned tetMeshGeomID() const { return _tet_mesh_geom_id; }
    void setTetMeshGeomID(unsigned id) { _tet_mesh_geom_id = id; }

    RTCScene tetScene() const { return _tet_scene; }
    void setTetScene(RTCScene scene) { _tet_scene = scene; }

    /** Returns a pointer to element indices (4 consecutive indices make up an element) */
    // const int* elementIndices() const { return _tet_mesh->elements().data(); }

    static bool isPointInTetrahedron(const float p[3], const float *v0, const float *v1, const float *v2, const float *v3);
    static float squaredDistanceToTetrahedron(const float p[3], const float* v0, const float* v1, const float* v2, const float* v3);
    static void boundsFuncTetrahedra(const struct RTCBoundsFunctionArguments *args);
    static void intersectFuncTetrahedra(const RTCIntersectFunctionNArguments *args);
    static bool pointQueryFuncTetrahedra(RTCPointQueryFunctionArguments *args);

    private:
    const Geometry::TetMesh* _tet_mesh;      // the volumetric (tetrahedral) mesh to create the BVH for
    unsigned _tet_mesh_geom_id;                        // Embree geometry ID in the scene

    RTCScene _tet_scene;    // Embree scene specifically for this tet mesh - used for point-in-tetrahedron queries
};

} // namespace Geometry

#endif // __EMBREE_TET_MESH_GEOMETRY_HPP