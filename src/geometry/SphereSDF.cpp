#include "geometry/SphereSDF.hpp"

#include "simobject/RigidPrimitives.hpp"

#include "common/Pack.hpp"

#ifdef HAVE_CUDA
#include "gpu/resource/SphereSDFGPUResource.hpp"
#endif

namespace Geometry
{

SphereSDF::SphereSDF(const Sim::RigidSphere* sphere)
    : SDF(), _sphere(sphere)
{}

inline Real SphereSDF::evaluate(const Vec3r& x) const
{
    // the distance from any point the surface of the sphere is simply the distance of the point to the sphere center minus the radius
    return (x - _sphere->position()).norm() - _sphere->radius();
}

inline Vec3r SphereSDF::gradient(const Vec3r& x) const
{
    // the gradient simply is a normalized vector pointing out from the sphere center in the direction of x
    return (x - _sphere->position()).normalized();
}

 #ifdef HAVE_CUDA
inline void SphereSDF::createGPUResource() 
{
    _gpu_resource = std::make_unique<Sim::SphereSDFGPUResource>(this);
    _gpu_resource->allocate();
}
 #endif

void SphereSDF::serialize(std::vector<std::byte>& buf) const
{
    pack(buf, _sphere);
}

void SphereSDF::deserialize(const std::byte*& buf)
{
    unpack(buf, _sphere);
}

} // namespace Geometry