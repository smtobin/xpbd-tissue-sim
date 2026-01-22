#include <embree4/rtcore.h>

#include "geometry/embree/EmbreeMeshGeometry.hpp"
#include "geometry/embree/EmbreeQueryStructs.hpp"
#include "geometry/embree/EmbreeTetMeshGeometry.hpp"

#include <iostream>

namespace Geometry
{

EmbreeMeshGeometry::EmbreeMeshGeometry(const Geometry::Mesh* mesh)
: _mesh(mesh), _undeformed_scene(nullptr)
{
    _initial_vertices = mesh->vertices();
}

EmbreeMeshGeometry::~EmbreeMeshGeometry()
{
    if (_undeformed_scene)
        rtcReleaseScene(_undeformed_scene);
}

void EmbreeMeshGeometry::updateSurfaceMeshGeometryBuffers(RTCGeometry geom)
{
    int num_vertices = _mesh->vertices().totalSize();
    int num_faces = _mesh->faces().totalSize();

    // allocate vertex buffer
    // important: allocate enough space for ALL vertices (including tombstones)
    // so that the indices in the faces are correct
    float* vertex_buffer = (float*)rtcSetNewGeometryBuffer(
        geom,
        RTC_BUFFER_TYPE_VERTEX,
        0,
        RTC_FORMAT_FLOAT3,
        3*sizeof(float),
        num_vertices
    );

    // copy vertices into buffer
    for (const auto& index : _mesh->vertices().validIndices())
    {
        const Vec3r& v  = _mesh->vertex(index);
        vertex_buffer[3*index] = static_cast<float>(v[0]);
        vertex_buffer[3*index+1] = static_cast<float>(v[1]);
        vertex_buffer[3*index+2] = static_cast<float>(v[2]);
    }

    // allocate face index buffer
    // important: allocate enough space for ALL faces (including tombstones)
    // so that the Embree primitive index is correct
    unsigned int* index_buffer = (unsigned int*)rtcSetNewGeometryBuffer(
        geom,
        RTC_BUFFER_TYPE_INDEX,
        0,
        RTC_FORMAT_UINT3,
        3*sizeof(unsigned int),
        num_faces
    );

    // copy faces into buffer
    // a safe vertex index for invalid faces
    int safe_index = *_mesh->vertices().validIndices().begin();
    for (int i = 0; i < _mesh->faces().totalSize(); i++)
    {
        // if the face is valid, copy it over
        if (_mesh->faceValid(i))
        {
            const Vec3i& face = _mesh->face(i);
            index_buffer[3*i] = static_cast<unsigned int>(face[0]);
            index_buffer[3*i+1] = static_cast<unsigned int>(face[1]);
            index_buffer[3*i+2] = static_cast<unsigned int>(face[2]);
        }
        // face is invalid, set it as degenerate (with all 3 vertices the same)
        // make sure to use a valid vertex
        else
        {
            index_buffer[3*i] = safe_index;
            index_buffer[3*i+1] = safe_index;
            index_buffer[3*i+2] = safe_index;
        }
        
    }
}

void EmbreeMeshGeometry::boundsFuncTriangle(const struct RTCBoundsFunctionArguments *args)
{
    const EmbreeMeshGeometry *geom = static_cast<const EmbreeMeshGeometry *>(args->geometryUserPtr);
    const Vec3i& indices = geom->mesh()->face(args->primID);
    const Vec3r& v1 = geom->mesh()->vertex(indices[0]);
    const Vec3r& v2 = geom->mesh()->vertex(indices[1]);
    const Vec3r& v3 = geom->mesh()->vertex(indices[2]);

    RTCBounds* bounds = args->bounds_o;
    bounds->lower_x = std::min({v1[0], v2[0], v3[0]});
    bounds->lower_y = std::min({v1[1], v2[1], v3[1]});
    bounds->lower_z = std::min({v1[2], v2[2], v3[2]});

    bounds->upper_x = std::max({v1[0], v2[0], v3[0]});
    bounds->upper_y = std::max({v1[1], v2[1], v3[1]});
    bounds->upper_z = std::max({v1[2], v2[2], v3[2]});
}

void EmbreeMeshGeometry::boundsFuncTriangleInitialVertices(const struct RTCBoundsFunctionArguments *args)
{
    const EmbreeMeshGeometry *geom = static_cast<const EmbreeMeshGeometry *>(args->geometryUserPtr);
    const Vec3i& indices = geom->mesh()->face(args->primID);
    const Vec3r& v1 = geom->mesh()->vertex(indices[0]);
    const Vec3r& v2 = geom->mesh()->vertex(indices[1]);
    const Vec3r& v3 = geom->mesh()->vertex(indices[2]);

    RTCBounds* bounds = args->bounds_o;
    bounds->lower_x = std::min({v1[0], v2[0], v3[0]});
    bounds->lower_y = std::min({v1[1], v2[1], v3[1]});
    bounds->lower_z = std::min({v1[2], v2[2], v3[2]});

    bounds->upper_x = std::max({v1[0], v2[0], v3[0]});
    bounds->upper_y = std::max({v1[1], v2[1], v3[1]});
    bounds->upper_z = std::max({v1[2], v2[2], v3[2]});
}

void EmbreeMeshGeometry::intersectFuncTriangle(const RTCIntersectFunctionNArguments *args)
{
    const int* valid = args->valid;
    const EmbreeMeshGeometry* geom = static_cast<EmbreeMeshGeometry*>(args->geometryUserPtr);

    // only consider the geometry we're interested in
    // if args->geomId is "invalid", we are interested in all geometry
    // if (args->geomID != RTC_INVALID_GEOMETRY_ID && args->geomID != geom->meshGeomID())
    //     return;

    assert(args->N == 1);

    if (!valid[0])
        return;

    // get vertices of face
    const Vec3i& indices = geom->mesh()->face(args->primID);
    const Vec3r& v1 = geom->mesh()->vertex(indices[0]);
    const Vec3r& v2 = geom->mesh()->vertex(indices[1]);
    const Vec3r& v3 = geom->mesh()->vertex(indices[2]);

    // loop through cast rays (there may be more than 1 in a batch)
    RTCRayHit* rayhit = (RTCRayHit*)args->rayhit;

    // test intersection between ray and triangle
    // this function will update the ray's tfar and the hit's u and v and geometry normal (if there is an intersection)
    if (_rayTriangleIntersect(&rayhit->ray, &rayhit->hit, v1, v2, v3))
    {
        // if there is an intersection, update the hit's primID and geomID for the intersected face
        rayhit->hit.primID = args->primID;
        rayhit->hit.geomID = args->geomID;
    }
}

void EmbreeMeshGeometry::intersectFuncTriangleInitialVertices(const RTCIntersectFunctionNArguments *args)
{
    const int* valid = args->valid;
    const EmbreeMeshGeometry* geom = static_cast<EmbreeMeshGeometry*>(args->geometryUserPtr);

    // only consider the geometry we're interested in
    // if args->geomId is "invalid", we are interested in all geometry
    // if (args->geomID != RTC_INVALID_GEOMETRY_ID && args->geomID != geom->meshGeomID())
    //     return;

    assert(args->N == 1);

    if (!valid[0])
        return;

    // get vertices of face
    const Vec3i& indices = geom->mesh()->face(args->primID);
    const Geometry::Mesh::vertices_vec_type& initial_vertices = geom->initialVertices();
    const Vec3r& v1 = initial_vertices[indices[0]];
    const Vec3r& v2 = initial_vertices[indices[1]];
    const Vec3r& v3 = initial_vertices[indices[2]];

    // loop through cast rays (there may be more than 1 in a batch)
    RTCRayHit* rayhit = (RTCRayHit*)args->rayhit;

    // test intersection between ray and triangle
    // this function will update the ray's tfar and the hit's u and v and geometry normal (if there is an intersection)
    if (_rayTriangleIntersect(&rayhit->ray, &rayhit->hit, v1, v2, v3))
    {
        // if there is an intersection, update the hit's primID and geomID for the intersected face
        rayhit->hit.primID = args->primID;
        rayhit->hit.geomID = args->geomID;
    }
}

bool EmbreeMeshGeometry::pointQueryFuncTriangle(RTCPointQueryFunctionArguments *args)
{
    // Get user data containing the query point and results vector
    EmbreeClosestPointQueryUserData *userData = static_cast<EmbreeClosestPointQueryUserData *>(args->userPtr);
    // Get the geometry data
    const EmbreeMeshGeometry *geom = userData->geom;

    // only consider point queries for the geometry we're interested in
    // TODO: should we do point queries for 
    if (args->geomID != geom->meshGeomID())
        return true;

    
    const Vec3i& indices = geom->mesh()->face(args->primID);
    const Vec3r& v1 = geom->mesh()->vertex(indices[0]);
    const Vec3r& v2 = geom->mesh()->vertex(indices[1]);
    const Vec3r& v3 = geom->mesh()->vertex(indices[2]);

    Vec3r closest_point = _closestPointTriangle(userData->point, v1, v2, v3);
    const Real d = (closest_point - userData->point).norm();

    if (d < args->query->radius)
    {
        EmbreeHit& hit = userData->result;
        args->query->radius = d;
        hit.prim_index = args->primID;
        hit.hit_point = closest_point;

        return true; // return true to indicate that the query radius changed
    }

    return false;
}

bool EmbreeMeshGeometry::pointQueryFuncTriangleInitialVertices(RTCPointQueryFunctionArguments *args)
{
    // Get user data containing the query point and results vector
    EmbreeClosestPointQueryUserData *userData = static_cast<EmbreeClosestPointQueryUserData *>(args->userPtr);
    // Get the geometry data
    const EmbreeMeshGeometry *geom = userData->geom;

    // only consider point queries for the geometry we're interested in
    // TODO: should we do point queries for 
    if (args->geomID != geom->meshGeomID())
        return true;

    
    const Vec3i& indices = geom->mesh()->face(args->primID);
    const Geometry::Mesh::vertices_vec_type& initial_vertices = geom->initialVertices();
    const Vec3r& v1 = initial_vertices[indices[0]];
    const Vec3r& v2 = initial_vertices[indices[1]];
    const Vec3r& v3 = initial_vertices[indices[2]];

    Vec3r closest_point = _closestPointTriangle(userData->point, v1, v2, v3);
    const Real d = (userData->point - closest_point).norm();

    if (d < args->query->radius)
    {
        EmbreeHit& hit = userData->result;
        args->query->radius = d;
        hit.prim_index = args->primID;
        hit.hit_point = closest_point;

        return true; // return true to indicate that the query radius changed
    }

    return false;
}

// adapted from: https://github.com/RenderKit/embree/blob/master/tutorials/common/math/closest_point.h
// void EmbreeMeshGeometry::_closestPointTriangle(const float p_[3], const float a_[3], const float b_[3], const float c_[3], float out_[3])
Vec3r EmbreeMeshGeometry::_closestPointTriangle(const Vec3r& p, const Vec3r& a, const Vec3r& b, const Vec3r& c)
{

    const Vec3r ab = b - a;
    const Vec3r ac = c - a;
    const Vec3r ap = p - a;

    const Real d1 = ab.dot(ap);
    const Real d2 = ac.dot(ap);
    if (d1 <= 0.f && d2 <= 0.f)
    {
        return a;
    }

    const Vec3r bp = p - b;
    const Real d3 = ab.dot(bp);
    const Real d4 = ac.dot(bp);
    if (d3 >= 0.f && d4 <= d3)
    {
        return b;
    }

    const Vec3r cp = p - c;
    const Real d5 = ab.dot(cp);
    const Real d6 = ac.dot(cp);
    if (d6 >= 0.f && d5 <= d6)
    {
        return c;
    }

    const Real vc = d1 * d4 - d3 * d2;
    if (vc <= 0.f && d1 >= 0.f && d3 <= 0.f)
    {
        const Real v = d1 / (d1 - d3);
        return a + v * ab;
    }
    
    const Real vb = d5 * d2 - d1 * d6;
    if (vb <= 0.f && d2 >= 0.f && d6 <= 0.f)
    {
        const Real v = d2 / (d2 - d6);
        return a + v * ac;
    }
    
    const Real va = d3 * d6 - d5 * d4;
    if (va <= 0.f && (d4 - d3) >= 0.f && (d5 - d6) >= 0.f)
    {
        const Real v = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        return b + v * (c - b);
    }

    const Real denom = 1.f / (va + vb + vc);
    const Real v = vb * denom;
    const Real w = vc * denom;
    return a + v * ab + w * ac;
}

bool EmbreeMeshGeometry::_rayTriangleIntersect(RTCRay* ray, RTCHit* hit, const Vec3r& a, const Vec3r& b, const Vec3r& c)
{
    const Vec3r ray_origin(ray->org_x, ray->org_y, ray->org_z);
    const Vec3r ray_dir(ray->dir_x, ray->dir_y, ray->dir_z);

    constexpr float epsilon = std::numeric_limits<float>::epsilon();

    const Vec3r edge1 = b - a;
    const Vec3r edge2 = c - a;
    const Vec3r ray_cross_e2 = ray_dir.cross(edge2);
    const Real det = edge1.dot(ray_cross_e2);

    if (det > -epsilon && det < epsilon)
        return false;
    
    const Real inv_det = 1.0 / det;
    const Vec3r s = ray_origin - a;
    const Real v = inv_det * s.dot(ray_cross_e2);

    if ( (v < 0 && std::abs(v) > epsilon) || (v > 1 && std::abs(v - 1) > epsilon) )
        return false;

    const Vec3r s_cross_e1 = s.cross(edge1);
    const Real w = inv_det * ray_dir.dot(s_cross_e1);

    if ( (w < 0 && std::abs(w) > epsilon) || (w + v > 1 && std::abs(w + v - 1) > epsilon) )
        return false;
    
    const Real t = inv_det * edge2.dot(s_cross_e1);

    if (t >= ray->tnear && t <= ray->tfar)
    {
        ray->tfar = t;
        hit->u = (1-v-w);
        hit->v = v;

        // compute normal
        const Vec3r n = edge1.cross(edge2);
        hit->Ng_x = n[0];
        hit->Ng_y = n[1];
        hit->Ng_z = n[2];

        return true;
    }
    else
    {
        return false;
    }
}

} // namespace Geometry