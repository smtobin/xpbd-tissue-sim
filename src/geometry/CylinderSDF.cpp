#include "geometry/CylinderSDF.hpp"
#include "utils/GeometryUtils.hpp"

#include "simobject/RigidPrimitives.hpp"

#include "common/Pack.hpp"


#ifdef HAVE_CUDA
#include "gpu/resource/CylinderSDFGPUResource.hpp"
#endif

namespace Geometry
{

CylinderSDF::CylinderSDF(const Sim::RigidCylinder* cyl)
    : _cyl(cyl)
{

}

inline Real CylinderSDF::evaluate(const Vec3r& x) const
{
    // transform x into body coordinates
    const Vec3r x_body = _cyl->globalToBody(x);

    // the distance from x to the cylinder in the XY plane (i.e. the distance from x to the surface of an infinite cylinder with the same radius)
    const Real xy_dist = std::sqrt(x_body[0]*x_body[0] + x_body[1]*x_body[1]) - _cyl->radius();
    // the distance from x to the top/bottom of the cylinder solely in the z-direction
    const Real z_dist = std::abs(x_body[2]) - 0.5*_cyl->height();

    // if this is positive, x is outside the cylinder in the XY plane
    const Real outside_dist_xy = std::max(xy_dist, Real(0.0));
    // if this is positive, x is outside the cylinder along the Z-axis
    const Real outside_dist_z = std::max(z_dist, Real(0.0));

    // the distance from x to the surface of the cylinder when x is outside the cylinder
    // evaluates to 0 when x is fully inside the cylinder
    const Real dist_when_outside = std::sqrt(outside_dist_xy*outside_dist_xy + outside_dist_z*outside_dist_z);

    // the distance from x to the surface of the cylinder when x is inside the cylinder
    // this is the max (i.e. closest to zero) between the XY distance and the Z distance
    // evaluates to 0 when x is outside the cylinder
    const Real dist_when_inside = std::min(std::max(xy_dist, z_dist), Real(0.0));

    return dist_when_outside + dist_when_inside;
}

inline Vec3r CylinderSDF::gradient(const Vec3r& x) const 
{
    // transform x into body coordinates
    const Vec3r x_body = _cyl->globalToBody(x);

    // the distance from x to the cylinder in the XY plane (i.e. the distance from x to the surface of an infinite cylinder with the same radius)        
    const Real xy_dist = std::sqrt(x_body[0]*x_body[0] + x_body[1]*x_body[1]) - _cyl->radius();

    // the distance from x to the top/bottom of the cylinder solely in the z-direction
    const Real z_dist = std::abs(x_body[2]) - 0.5*_cyl->height();

    // if this is positive, x is outside the cylinder in the XY plane
    const Real outside_dist_xy = std::max(xy_dist, Real(0.0));
    // if this is positive, x is outside the cylinder along the Z-axis
    const Real outside_dist_z = std::max(z_dist, Real(0.0));

    // the distance from x to the surface of the cylinder when x is inside the cylinder
    // this is the max (i.e. closest to zero) between the XY distance and the Z distance
    // evaluates to 0 when x is outside the cylinder
    const Real inside_dist = std::min(std::max(xy_dist, z_dist), Real(0.0));

    const Vec3r grad_when_outside = Vec3r({x_body[0]*(outside_dist_xy>0), x_body[1]*(outside_dist_xy>0), x_body[2]*(outside_dist_z>0)}).normalized();
    const Vec3r grad_when_inside = Vec3r({x_body[0]*(inside_dist==xy_dist), x_body[1]*(inside_dist==xy_dist), x_body[2]*(inside_dist==z_dist)}).normalized();
    return GeometryUtils::rotateVectorByQuat(grad_when_outside + grad_when_inside, _cyl->orientation());

}

#ifdef HAVE_CUDA
inline void CylinderSDF::createGPUResource() 
{
    _gpu_resource = std::make_unique<Sim::CylinderSDFGPUResource>(this);
    _gpu_resource->allocate();
}
#endif

void CylinderSDF::serialize(std::vector<std::byte>& buf) const
{
    pack(buf, _cyl);
}

void CylinderSDF::deserialize(const std::byte*& buf)
{
    unpack(buf, _cyl);
}

} // namespace Geometry