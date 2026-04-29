#include "geometry/embree/EmbreeObjectGeometry.hpp"
#include <embree4/rtcore.h>

namespace Geometry
{

EmbreeObjectGeometry::EmbreeObjectGeometry(const Sim::Object* obj)
    : _obj(obj)
{

}

EmbreeObjectGeometry::~EmbreeObjectGeometry() = default;

void EmbreeObjectGeometry::boundsFuncObject(const struct RTCBoundsFunctionArguments *args)
{
    const EmbreeObjectGeometry *geom = static_cast<const EmbreeObjectGeometry *>(args->geometryUserPtr);
    const Sim::Object* obj = geom->object();
    AABB bbox = obj->boundingBox();

    RTCBounds* bounds = args->bounds_o;
    bounds->lower_x = bbox.min[0];
    bounds->lower_y = bbox.min[1];
    bounds->lower_z = bbox.min[2];

    bounds->upper_x = bbox.max[0];
    bounds->upper_y = bbox.max[1];
    bounds->upper_z = bbox.max[2];
}

void EmbreeObjectGeometry::intersectFuncObject(const RTCIntersectFunctionNArguments* args)
{
    int N = args->N;    // number of rays in packet
    int* valid = args->valid;

    const EmbreeObjectGeometry *geom = static_cast<const EmbreeObjectGeometry *>(args->geometryUserPtr);
    const Sim::Object* obj = geom->object();

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

        if (_intersectRaySDF(ray_origin, ray_dir, ray_tnear, ray_tfar, obj->SDF(), t_hit, normal) )
        {
            RTCRayN_tfar(rays, N, i) = t_hit;
            RTCHitN_u(hits, N, i) = 0.0f;
            RTCHitN_v(hits, N, i) = 0.0f;
            RTCHitN_geomID(hits, N, i) = args->geomID;
            RTCHitN_primID(hits, N, i) = 0;
            RTCHitN_Ng_x(hits, N, i) = normal[0];
            RTCHitN_Ng_y(hits, N, i) = normal[1];
            RTCHitN_Ng_z(hits, N, i) = normal[2];
        }

    }
}

bool EmbreeObjectGeometry::_intersectRaySDF(
        const Vec3r& ray_origin, const Vec3r& ray_dir, Real ray_tnear, Real ray_tfar,
        const Geometry::SDF* sdf,
        Real& t_hit, Vec3r& normal
    )
{
    const int maxSteps = 32;
    const Real epsilon = 1e-4f;

    Real t = ray_tnear;

    for (int i = 0; i < maxSteps && t < ray_tfar; i++) {
        Vec3r p = ray_origin + t * ray_dir;

        Real d = sdf->evaluate(p);

        if (d < epsilon) 
        {
            // Hit!
            t_hit = t;

            normal = sdf->gradient(p);
            return true;
        }

        // sphere tracing
        t += d;
    }

    return false;
}

} // namespace Geometry