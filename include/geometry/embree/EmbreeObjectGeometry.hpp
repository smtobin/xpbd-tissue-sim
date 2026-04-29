#ifndef __EMBREE_OBJECT_GEOMETRY_HPP
#define __EMBREE_OBJECT_GEOMETRY_HPP

#include <embree4/rtcore.h>
#include "simobject/Object.hpp"

namespace Geometry
{

/** User-defined Embree geometry that is used to create the BVH */
class EmbreeObjectGeometry
{
    public:

    explicit EmbreeObjectGeometry(const Sim::Object* object);

    ~EmbreeObjectGeometry();

    const Sim::Object* object() const { return _obj; }

    unsigned geomID() const { return _obj_geom_id; }
    void setGeomID(unsigned id) { _obj_geom_id = id; }

    static void intersectFuncObject(const RTCIntersectFunctionNArguments* args);
    static void boundsFuncObject(const struct RTCBoundsFunctionArguments *args);

    private:
    static bool _intersectRaySDF(
        const Vec3r& ray_origin, const Vec3r& ray_dir, Real ray_tnear, Real ray_tfar,
        const Geometry::SDF* sdf,
        Real& t_hit, Vec3r& normal
    );

    const Sim::Object* _obj;      // the volumetric (tetrahedral) mesh to create the BVH for
    unsigned _obj_geom_id;                        // Embree geometry ID in the scene
};

} // namespace Geometry

#endif // __EMBREE_TET_MESH_GEOMETRY_HPP