#include "geometry/embree/EmbreeVirtuosoArmGeometry.hpp"
#include <embree4/rtcore.h>

namespace Geometry
{

EmbreeVirtuosoArmGeometry::EmbreeVirtuosoArmGeometry(const Sim::VirtuosoArm* arm)
    : _arm(arm)
{

}

EmbreeVirtuosoArmGeometry::~EmbreeVirtuosoArmGeometry() = default;

void EmbreeVirtuosoArmGeometry::boundsFuncCapsule(const struct RTCBoundsFunctionArguments *args)
{
    const EmbreeVirtuosoArmGeometry *geom = static_cast<const EmbreeVirtuosoArmGeometry *>(args->geometryUserPtr);
    const Sim::VirtuosoArm* arm = geom->arm();
    unsigned int primID = args->primID;
    Capsule segment = arm->segment(primID);

    const Vec3r& p1 = segment.p1();
    const Vec3r& p2 = segment.p2();

    RTCBounds* bounds = args->bounds_o;
    bounds->lower_x = std::min(p1[0], p2[0]);
    bounds->lower_y = std::min(p1[1], p2[1]);
    bounds->lower_z = std::min(p1[2], p2[2]);

    bounds->upper_x = std::max(p1[0], p2[0]);
    bounds->upper_y = std::max(p1[1], p2[1]);
    bounds->upper_z = std::max(p1[2], p2[2]);
}

void EmbreeVirtuosoArmGeometry::intersectFuncCapsule(const RTCIntersectFunctionNArguments* args)
{
    int N = args->N;    // number of rays in packet
    int* valid = args->valid;

    const EmbreeVirtuosoArmGeometry *geom = static_cast<const EmbreeVirtuosoArmGeometry *>(args->geometryUserPtr);
    const Sim::VirtuosoArm* arm = geom->arm();
    unsigned int primID = args->primID;
    Capsule segment = arm->segment(primID);

    struct RTCRayHitN* rayhit = (struct RTCRayHitN*)args->rayhit;
    RTCRayN* rays = RTCRayHitN_RayN(rayhit, N);
    RTCHitN* hits = RTCRayHitN_HitN(rayhit, N);

    for (int i = 0; i < N; i++)
    {
        if (!valid[i])
            continue;

        Vec3r ray_origin(
            RTCRayN_org_x(rays, N, i),
            RTCRayN_org_y(rays, N, i),
            RTCRayN_org_z(rays, N, i)
        );

        Vec3r ray_dir(
            RTCRayN_dir_x(rays, N, i),
            RTCRayN_dir_y(rays, N, i),
            RTCRayN_dir_z(rays, N, i)  
        );

        Real ray_tnear = RTCRayN_tnear(rays, N, i);
        Real ray_tfar = RTCRayN_tfar(rays, N, i);

        Real t_hit;
        Vec3r normal;

        if (_intersectRayCapsule(ray_origin, ray_dir, ray_tnear, ray_tfar, segment.p1(), segment.p2(), segment.radius(), t_hit, normal) )
        {
            RTCRayN_tfar(rays, N, i) = t_hit;
            RTCHitN_u(hits, N, i) = 0.0f;
            RTCHitN_v(hits, N, i) = 0.0f;
            RTCHitN_geomID(hits, N, i) = args->geomID;
            RTCHitN_primID(hits, N, i) = primID;
            RTCHitN_Ng_x(hits, N, i) = normal[0];
            RTCHitN_Ng_y(hits, N, i) = normal[1];
            RTCHitN_Ng_z(hits, N, i) = normal[2];
        }

    }
}

bool EmbreeVirtuosoArmGeometry::_intersectRayCapsule(
    const Vec3r& ray_origin, const Vec3r& ray_dir, Real ray_tnear, Real ray_tfar,
    const Vec3r& cap_p1, const Vec3r& cap_p2, Real cap_radius,
    Real& t_hit, Vec3r& normal
)
{
    // capsule line segment direction
    Vec3r cap_dir = cap_p2 - cap_p1;
    Real cap_length_sq = cap_dir.squaredNorm();
    if (cap_length_sq < 1e-12)
    {
        // degenerate capsule (sphere)
        Vec3r oc = ray_origin - cap_p1;
        Real a = ray_dir.dot(ray_dir);
        Real b = 2 * oc.dot(ray_dir);
        Real c = oc.dot(oc) - cap_radius * cap_radius;
        Real discriminant = b*b - 4*a*c;

        if (discriminant < 0)
            return false;
        
        Real t = -b - std::sqrt(discriminant) / (2*a);
        if ( t < ray_tnear || t > ray_tfar)
        {
            // check other solution
            t = -b + std::sqrt(discriminant) / (2*a);
            if (t < ray_tnear || t > ray_tfar)
                return false;
        }

        // hit!
        t_hit = t;
        Vec3r hit_point = ray_origin + ray_dir*t;
        normal = (hit_point - cap_p1).normalized();
        return true;
    }

    Real cap_length = std::sqrt(cap_length_sq);
    cap_dir = cap_dir.normalized();

    // find closest points between ray and capsule axis
    Vec3r ray_to_p1 = cap_p1 - ray_origin;
    Real d_dot_v = ray_dir.dot(cap_dir);
    Real d_dot_ao = ray_dir.dot(ray_to_p1);
    Real v_dot_ao = cap_dir.dot(ray_to_p1);

    Real denom = 1 - d_dot_v * d_dot_v;

    Real t_ray, t_capsule;
    if (std::abs(denom) < 1e-8)
    {
        // ray and capsule are parallel
        t_ray = 0;
        t_capsule = v_dot_ao;
    }
    else
    {
        t_ray = (d_dot_ao - v_dot_ao * d_dot_v) / denom;
        t_capsule = (d_dot_ao * d_dot_v - v_dot_ao) / denom;
    }

    // clamp capsule parameter to segment
    t_capsule = std::max(Real(0.0), std::min(cap_length, t_capsule));

    // find closest point on capsule axis
    Vec3r cap_cp = cap_p1 + cap_dir*t_capsule;

    // vector from ray origin to closet point on capsule
    Vec3r ray_to_cap_cp = cap_cp - ray_origin;

    // project onto ray to find closest point on ray
    Real proj = ray_to_cap_cp.dot(ray_dir);

    // find closet point on ray
    Vec3r ray_cp = ray_origin + ray_dir * proj;

    // distance between closest points
    Vec3r diff = ray_cp - cap_cp;
    Real dist = diff.norm();

    if (dist > cap_radius)
        return false;
    
    // calculate intersection point along ray
    Real offset = std::sqrt(cap_radius*cap_radius - dist*dist);
    t_hit = proj - offset;

    if (t_hit < ray_tnear || t_hit > ray_tfar)
    {
        t_hit = proj + offset;
        if (t_hit < ray_tnear || t_hit > ray_tfar)
            return false;
    }

    // calculate normal
    Vec3r hit_point = ray_origin + ray_dir*t_hit;
    // find closet point on capsule axis to hit point
    Vec3r to_hit = hit_point - cap_p1;
    Real t_axis = std::max(Real(0.0), std::min(cap_length, to_hit.dot(cap_dir)));

    Vec3r axis_cp = cap_p1 + t_axis*cap_dir;
    normal = (hit_point - axis_cp).normalized();

    return true;
}

} // namespace Geometry