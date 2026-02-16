#ifndef __EMBREE_VIRTUOSO_ARM_GEOMETRY_HPP
#define __EMBREE_VIRTUOSO_ARM_GEOMETRY_HPP

#include <embree4/rtcore.h>
#include "simobject/VirtuosoArm.hpp"

namespace Geometry
{

/** User-defined Embree geometry that is used to create the BVH */
class EmbreeVirtuosoArmGeometry
{
    public:

    explicit EmbreeVirtuosoArmGeometry(const Sim::VirtuosoArm* arm);

    ~EmbreeVirtuosoArmGeometry();

    const Sim::VirtuosoArm* arm() const { return _arm; }

    unsigned geomID() const { return _arm_geom_id; }
    void setGeomID(unsigned id) { _arm_geom_id = id; }

    static void intersectFuncCapsule(const RTCIntersectFunctionNArguments* args);
    static void boundsFuncCapsule(const struct RTCBoundsFunctionArguments *args);

    private:
    static bool _intersectRayCapsule(
        const Vec3r& ray_origin, const Vec3r& ray_dir, Real ray_tnear, Real ray_tfar,
        const Vec3r& cap_p1, const Vec3r& cap_p2, Real cap_radius,
        Real& t_hit, Vec3r& normal
    );

    const Sim::VirtuosoArm* _arm;      // the volumetric (tetrahedral) mesh to create the BVH for
    unsigned _arm_geom_id;                        // Embree geometry ID in the scene
};

} // namespace Geometry

#endif // __EMBREE_TET_MESH_GEOMETRY_HPP