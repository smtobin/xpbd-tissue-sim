#include "geometry/embree/EmbreeTetMeshGeometry.hpp"
#include "geometry/embree/EmbreeQueryStructs.hpp"

#include "simobject/MeshObject.hpp"

namespace Geometry
{

EmbreeTetMeshGeometry::EmbreeTetMeshGeometry(const Geometry::TetMesh* tet_mesh)
    : EmbreeMeshGeometry(tet_mesh), _tet_mesh(tet_mesh), _tet_scene(nullptr)
{
}

EmbreeTetMeshGeometry::~EmbreeTetMeshGeometry()
{
    if (_tet_scene)
        rtcReleaseScene(_tet_scene);
}

bool EmbreeTetMeshGeometry::isPointInTetrahedron(const float p[3], const float *v0, const float *v1, const float *v2, const float *v3)
{

    // Helper function to compute normal and check same side
    auto sameSide = [](const float p[3], const float a[3], const float b[3],
                       const float c[3], const float d[3]) -> bool
    {
        // Compute normal vector of face (a,b,c)
        float ab[3] = {b[0] - a[0], b[1] - a[1], b[2] - a[2]};
        float ac[3] = {c[0] - a[0], c[1] - a[1], c[2] - a[2]};
        float normal[3] = {
            ab[1] * ac[2] - ab[2] * ac[1],
            ab[2] * ac[0] - ab[0] * ac[2],
            ab[0] * ac[1] - ab[1] * ac[0]};

        // Compute dot products to check if p and d are on same side
        float dotP = normal[0] * (p[0] - a[0]) + normal[1] * (p[1] - a[1]) + normal[2] * (p[2] - a[2]);
        float dotD = normal[0] * (d[0] - a[0]) + normal[1] * (d[1] - a[1]) + normal[2] * (d[2] - a[2]);

        return (dotP * dotD >= 0);
    };

    // Point is inside if it's on the same side of all faces
    return sameSide(p, v0, v1, v2, v3) &&
           sameSide(p, v0, v1, v3, v2) &&
           sameSide(p, v0, v2, v3, v1) &&
           sameSide(p, v1, v2, v3, v0);
}

float EmbreeTetMeshGeometry::squaredDistanceToTetrahedron(const float p[3], const float* v0, const float* v1, const float* v2, const float* v3)
{
    if (isPointInTetrahedron(p, v0, v1, v2, v3))
        return 0.0f;
    
    float p_out1[3], p_out2[3], p_out3[3], p_out4[3];
    EmbreeMeshGeometry::_closestPointTriangle(p, v0, v1, v2, p_out1);
    EmbreeMeshGeometry::_closestPointTriangle(p, v0, v3, v1, p_out2);
    EmbreeMeshGeometry::_closestPointTriangle(p, v0, v2, v3, p_out3);
    EmbreeMeshGeometry::_closestPointTriangle(p, v1, v3, v2, p_out4);

    auto sq_dist = [](const float p1[3], const float p2[3]) -> float
    {
        float p_diff[3];
        p_diff[0] = p1[0] - p2[0];
        p_diff[1] = p1[1] - p2[1];
        p_diff[2] = p1[2] - p2[2];
        return p_diff[0]*p_diff[0] + p_diff[1]*p_diff[1] + p_diff[2]*p_diff[2];
    };

    float d1 = sq_dist(p, p_out1);
    float d2 = sq_dist(p, p_out2);
    float d3 = sq_dist(p, p_out3);
    float d4 = sq_dist(p, p_out4);

    return std::min({d1, d2, d3, d4});

}

void EmbreeTetMeshGeometry::boundsFuncTetrahedra(const struct RTCBoundsFunctionArguments *args)
{
    const EmbreeTetMeshGeometry *geom = static_cast<const EmbreeTetMeshGeometry *>(args->geometryUserPtr);
    const int *indices = geom->tetMesh()->element(args->primID).data();
    const float *v1 = geom->vertices() + 3 * indices[0];
    const float *v2 = geom->vertices() + 3 * indices[1];
    const float *v3 = geom->vertices() + 3 * indices[2];
    const float *v4 = geom->vertices() + 3 * indices[3];

    RTCBounds* bounds = args->bounds_o;
    bounds->lower_x = std::min({v1[0], v2[0], v3[0], v4[0]});
    bounds->lower_y = std::min({v1[1], v2[1], v3[1], v4[1]});
    bounds->lower_z = std::min({v1[2], v2[2], v3[2], v4[2]});

    bounds->upper_x = std::max({v1[0], v2[0], v3[0], v4[0]});
    bounds->upper_y = std::max({v1[1], v2[1], v3[1], v4[1]});
    bounds->upper_z = std::max({v1[2], v2[2], v3[2], v4[2]});
}

void EmbreeTetMeshGeometry::intersectFuncTetrahedra(const RTCIntersectFunctionNArguments *args)
{
    // Basic ray-tetrahedron intersection implementation
    // This function is required for RTCGeometry but we won't use it
    // for point queries directly.
}

bool EmbreeTetMeshGeometry::pointQueryFuncTetrahedra(RTCPointQueryFunctionArguments *args)
{
    // Get user data containing the query point and results vector
    EmbreePointQueryUserData *userData = static_cast<EmbreePointQueryUserData *>(args->userPtr);
    // Get the geometry data
    const EmbreeTetMeshGeometry *geom = userData->geom;

    // only consider point queries for the geometry we're interested in
    // TODO: should we do point queries for 
    if (args->geomID != geom->tetMeshGeomID())
        return false;

    
    const int *indices = geom->tetMesh()->element(args->primID).data();
    const float *v1 = geom->vertices() + 3 * indices[0];
    const float *v2 = geom->vertices() + 3 * indices[1];
    const float *v3 = geom->vertices() + 3 * indices[2];
    const float *v4 = geom->vertices() + 3 * indices[3];

    const float *point = userData->point;

    // std::cout << "Point query: " << point[0] << ", " << point[1] << ", " << point[2] << std::endl;

    if (userData->vertex_ind != -1 && (
        userData->vertex_ind == indices[0] ||
        userData->vertex_ind == indices[1] ||
        userData->vertex_ind == indices[2] ||
        userData->vertex_ind == indices[3]))
    {
        // the tetrahedron in question has this query point as one of its vertices, so we should not include it
        return false;
    }

    // std::cout << "Testing element " << args->primID << "..." << std::endl;

    // Check if the point is inside this tetrahedron
    if (userData->radius == 0.0f && isPointInTetrahedron(point, v1, v2, v3, v4))
    {
        EmbreeHit hit;
        hit.obj = userData->obj_ptr;
        hit.prim_index = args->primID;
        // Add this tetrahedron's ID to the results
        userData->result.insert(hit);
        // std::cout << "Hit! Prim index: " << hit.prim_index << " Distance: " << std::sqrt(squaredDistanceToTetrahedron(point, v1, v2, v3, v4)) << " Radius: " << userData->radius << std::endl;

        // done searching
        return false;
    }
    else if (userData->radius != 0.0f && squaredDistanceToTetrahedron(point, v1, v2, v3, v4) <= userData->radius*userData->radius)
    {
        EmbreeHit hit;
        hit.obj = userData->obj_ptr;
        hit.prim_index = args->primID;
        // Add this tetrahedron's ID to the results
        userData->result.insert(hit);

        // continue searching
        return false;
    }
    else
    {
        // std::cout << "Miss. Distance: " << std::sqrt(squaredDistanceToTetrahedron(point, v1, v2, v3, v4)) << "; Radius: " << userData->radius << std::endl;
    }

    return false; // Continue traversal to find all potential tetrahedra
}

} // namespace Geometry