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
    // if double precision is being used in the sim, we must allocate space fo the float vertex buffer
    if constexpr (std::is_same_v<Real, double>)
        _vertex_buffer.resize(mesh->numVertices()*3);
    
    int index = 0;
    _initial_vertex_buffer.resize(mesh->numVertices()*3);
    for (const auto& v : _mesh->vertices())
    {
        _initial_vertex_buffer[3*index] = static_cast<float>(v[0]);
        _initial_vertex_buffer[3*index+1] = static_cast<float>(v[1]);
        _initial_vertex_buffer[3*index+2] = static_cast<float>(v[2]);
        index++;
    }
}

EmbreeMeshGeometry::~EmbreeMeshGeometry()
{
    if (_undeformed_scene)
        rtcReleaseScene(_undeformed_scene);
}

const float* EmbreeMeshGeometry::vertices() const
{
    return _vertex_buffer.data();
}

void EmbreeMeshGeometry::copyVertices()
{
    int index = 0;
    _vertex_buffer.resize(_mesh->numVertices()*3);
    for (const auto& v : _mesh->vertices())
    {
        _vertex_buffer[3*index] = static_cast<float>(v[0]);
        _vertex_buffer[3*index+1] = static_cast<float>(v[1]);
        _vertex_buffer[3*index+2] = static_cast<float>(v[2]);
        index++;
    }
}

void EmbreeMeshGeometry::boundsFuncTriangle(const struct RTCBoundsFunctionArguments *args)
{
    const EmbreeMeshGeometry *geom = static_cast<const EmbreeMeshGeometry *>(args->geometryUserPtr);
    const Vec3i& indices = geom->mesh()->face(args->primID);
    const float *v1 = geom->vertices() + 3 * indices[0];
    const float *v2 = geom->vertices() + 3 * indices[1];
    const float *v3 = geom->vertices() + 3 * indices[2];

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
    const float *v1 = geom->initialVertices() + 3 * indices[0];
    const float *v2 = geom->initialVertices() + 3 * indices[1];
    const float *v3 = geom->initialVertices() + 3 * indices[2];

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
    const float *v1 = geom->vertices() + 3 * indices[0];
    const float *v2 = geom->vertices() + 3 * indices[1];
    const float *v3 = geom->vertices() + 3 * indices[2];

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
    const float *v1 = geom->initialVertices() + 3 * indices[0];
    const float *v2 = geom->initialVertices() + 3 * indices[1];
    const float *v3 = geom->initialVertices() + 3 * indices[2];

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
    const float *v1 = geom->vertices() + 3 * indices[0];
    const float *v2 = geom->vertices() + 3 * indices[1];
    const float *v3 = geom->vertices() + 3 * indices[2];

    
    const float *point = userData->point;
    float closest_point[3]; closest_point[0] = 0; closest_point[1] = 0; closest_point[2] = 0;

    _closestPointTriangle(point, v1, v2, v3, closest_point);
    const float d = (Eigen::Map<const Eigen::Vector3f>(point) - Eigen::Map<const Eigen::Vector3f>(closest_point)).norm();

    if (d < args->query->radius)
    {
        EmbreeHit& hit = userData->result;
        args->query->radius = d;
        hit.prim_index = args->primID;
        hit.hit_point[0] = Real(closest_point[0]); 
        hit.hit_point[1] = Real(closest_point[1]);
        hit.hit_point[2] = Real(closest_point[2]);

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
    const float *v1 = geom->initialVertices() + 3 * indices[0];
    const float *v2 = geom->initialVertices() + 3 * indices[1];
    const float *v3 = geom->initialVertices() + 3 * indices[2];

    
    const float *point = userData->point;
    float closest_point[3]; closest_point[0] = 0; closest_point[1] = 0; closest_point[2] = 0;

    _closestPointTriangle(point, v1, v2, v3, closest_point);
    const float d = (Eigen::Map<const Eigen::Vector3f>(point) - Eigen::Map<const Eigen::Vector3f>(closest_point)).norm();

    if (d < args->query->radius)
    {
        EmbreeHit& hit = userData->result;
        args->query->radius = d;
        hit.prim_index = args->primID;
        hit.hit_point[0] = Real(closest_point[0]); 
        hit.hit_point[1] = Real(closest_point[1]);
        hit.hit_point[2] = Real(closest_point[2]);

        return true; // return true to indicate that the query radius changed
    }

    return false;
}

// adapted from: https://github.com/RenderKit/embree/blob/master/tutorials/common/math/closest_point.h
void EmbreeMeshGeometry::_closestPointTriangle(const float p_[3], const float a_[3], const float b_[3], const float c_[3], float out_[3])
{
    Eigen::Map<const Eigen::Vector3f> p(p_);
    Eigen::Map<const Eigen::Vector3f> a(a_);
    Eigen::Map<const Eigen::Vector3f> b(b_);
    Eigen::Map<const Eigen::Vector3f> c(c_);
    Eigen::Map<Eigen::Vector3f> out(out_);

    const Eigen::Vector3f ab = b - a;
    const Eigen::Vector3f ac = c - a;
    const Eigen::Vector3f ap = p - a;

    const float d1 = ab.dot(ap);
    const float d2 = ac.dot(ap);
    if (d1 <= 0.f && d2 <= 0.f)
    {
        out = a;
        return;
    }

    const Eigen::Vector3f bp = p - b;
    const float d3 = ab.dot(bp);
    const float d4 = ac.dot(bp);
    if (d3 >= 0.f && d4 <= d3)
    {
        out = b;
        return;
    }

    const Eigen::Vector3f cp = p - c;
    const float d5 = ab.dot(cp);
    const float d6 = ac.dot(cp);
    if (d6 >= 0.f && d5 <= d6)
    {
        out = c;
        return;
    }

    const float vc = d1 * d4 - d3 * d2;
    if (vc <= 0.f && d1 >= 0.f && d3 <= 0.f)
    {
        const float v = d1 / (d1 - d3);
        out = a + v * ab;
        return;
    }
    
    const float vb = d5 * d2 - d1 * d6;
    if (vb <= 0.f && d2 >= 0.f && d6 <= 0.f)
    {
        const float v = d2 / (d2 - d6);
        out = a + v * ac;
        return;
    }
    
    const float va = d3 * d6 - d5 * d4;
    if (va <= 0.f && (d4 - d3) >= 0.f && (d5 - d6) >= 0.f)
    {
        const float v = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        out = b + v * (c - b);
        return;
    }

    const float denom = 1.f / (va + vb + vc);
    const float v = vb * denom;
    const float w = vc * denom;
    out = a + v * ab + w * ac;
    // out_[0] = out[0]; out_[1] = out[1]; out_[2] = out[2];
    return;
}

bool EmbreeMeshGeometry::_rayTriangleIntersect(RTCRay* ray, RTCHit* hit,
    const float a_[3], const float b_[3], const float c_[3])
{
    // Eigen::Map<const Eigen::Vector3f> ray_origin(ray_origin_);
    // Eigen::Map<const Eigen::Vector3f> ray_dir(ray_dir_);
    const Eigen::Vector3f ray_origin(ray->org_x, ray->org_y, ray->org_z);
    const Eigen::Vector3f ray_dir(ray->dir_x, ray->dir_y, ray->dir_z);

    Eigen::Map<const Eigen::Vector3f> a(a_);
    Eigen::Map<const Eigen::Vector3f> b(b_);
    Eigen::Map<const Eigen::Vector3f> c(c_);

    constexpr float epsilon = std::numeric_limits<float>::epsilon();

    const Eigen::Vector3f edge1 = b - a;
    const Eigen::Vector3f edge2 = c - a;
    const Eigen::Vector3f ray_cross_e2 = ray_dir.cross(edge2);
    const float det = edge1.dot(ray_cross_e2);

    if (det > -epsilon && det < epsilon)
        return false;
    
    const float inv_det = 1.0 / det;
    const Eigen::Vector3f s = ray_origin - a;
    const float v = inv_det * s.dot(ray_cross_e2);

    if ( (v < 0 && std::abs(v) > epsilon) || (v > 1 && std::abs(v - 1) > epsilon) )
        return false;

    const Eigen::Vector3f s_cross_e1 = s.cross(edge1);
    const float w = inv_det * ray_dir.dot(s_cross_e1);

    if ( (w < 0 && std::abs(w) > epsilon) || (w + v > 1 && std::abs(w + v - 1) > epsilon) )
        return false;
    
    const float t = inv_det * edge2.dot(s_cross_e1);

    if (t >= ray->tnear && t <= ray->tfar)
    {
        ray->tfar = t;
        hit->u = (1-v-w);
        hit->v = v;

        // compute normal
        const Eigen::Vector3f n = edge1.cross(edge2);
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