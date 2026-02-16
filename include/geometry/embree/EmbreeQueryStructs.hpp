#ifndef __EMBREE_QUERY_STRUCTS_HPP
#define __EMBREE_QUERY_STRUCTS_HPP

#include "common/types.hpp"
#include "common/SimulationTypeDefs.hpp"

#include <set>

namespace Sim
{
    class MeshObject;
    class TetMeshObject;
}

namespace Geometry
{

class EmbreeMeshGeometry;
class EmbreeTetMeshGeometry;

/** "Hit" result for ray queries. */
struct EmbreeRayHit
{
    SimulationObjectConstPtrVariantType obj;
    int prim_index;             // index of primitive hit (could be triangle or tetrahedron depending on context)
    Vec3r hit_point;            // where the primitive was hit (really only makes sense for ray intersections)

    bool operator <(const EmbreeRayHit& other) const
    {
        return std::tie(obj, prim_index) < std::tie(other.obj, other.prim_index);
    }

    bool operator ==(const EmbreeRayHit& other) const
    {
        return std::tie(obj, prim_index, hit_point) == std::tie(other.obj, other.prim_index, other.hit_point);
    }
};

/** "Hit" result for point queries. */
struct EmbreePQHit
{
    const Sim::MeshObject* obj;     // pointer to the mesh object in sim
    int prim_index;                 // index of primitive
    Vec3r hit_point;                // the result of the point query

    bool operator <(const EmbreePQHit& other) const
    {
        return std::tie(obj, prim_index) < std::tie(other.obj, other.prim_index);
    }

    bool operator ==(const EmbreePQHit& other) const
    {
        return std::tie(obj, prim_index, hit_point) == std::tie(other.obj, other.prim_index, other.hit_point);
    }
};

/** User-defined Embree point query data to be used during Embree point-in-tetrahedra queries. */
struct EmbreePointQueryUserData
{
    const Sim::TetMeshObject* obj_ptr;  // pointer to object who owns geometry being queried
    const EmbreeTetMeshGeometry* geom;  // the geometry being queried
    std::set<EmbreePQHit> result;          // the "result" of the point query - i.e. unique list of elements the point is inside
    Vec3r point;                 // the query point
    int vertex_ind;                     // (for self-collision queries) the vertex index of the queried point - used to exclude tetrahedra that contain the vertex
    float radius=0;                       // the radius of the point query - set to 0 for a strict point-in-tetrahedra query
};

/** User-defined Embree point query data for closest point queries. */
struct EmbreeClosestPointQueryUserData
{
    const Sim::MeshObject* obj_ptr;
    const EmbreeMeshGeometry* geom;
    EmbreePQHit result;
    Vec3r point;
};

} // namespace Geometry

#endif // __EMBREE_QUERY_STRUCTS_HPP