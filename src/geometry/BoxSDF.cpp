#include "geometry/BoxSDF.hpp"
#include "utils/GeometryUtils.hpp"

#include "simobject/RigidPrimitives.hpp"

#include "common/Pack.hpp"

#ifdef HAVE_CUDA
#include "gpu/resource/BoxSDFGPUResource.hpp"
#endif

namespace Geometry
{

BoxSDF::BoxSDF(const Sim::RigidBox* box)
    : SDF(), _box(box)
{

}

inline Real BoxSDF::evaluate(const Vec3r& x) const
{
    // transform x into body coordinates
    const Vec3r x_body = _box->globalToBody(x);
    
    // with x in body coordinates, we find the distance from x to an axis-aligned box centered at the origin
    // due to the symmetry of a box centered at the origin, we only need to handle the first quadrant case
    // we ultimately want to avoid branches (i.e. if statements) as much as possible for max speed

    // put x into the first quadrant and find the distance between each coordinate and the box semisize
    const Vec3r q = x_body.cwiseAbs() - 0.5*_box->size();
    
    // if any of these are positive, x is outside the box
    const Real q0 = std::max(q[0], Real(0.0));
    const Real q1 = std::max(q[1], Real(0.0));
    const Real q2 = std::max(q[2], Real(0.0));

    // if the max of the q vector is negative, x is inside the box
    const Real max_q = q.maxCoeff();

    // distance of x to the surface of the box - evaluates to 0 when x is inside the box
    const Real dist_when_outside = std::sqrt(q0*q0 + q1*q1 + q2*q2);
    // distance of x to the surface of the box - evaluates to 0 when x is outside the box
    const Real dist_when_inside = std::min(max_q, Real(0.0));

    return dist_when_outside + dist_when_inside;
}

inline Vec3r BoxSDF::gradient(const Vec3r& x) const
{
    // transform x into body coordinates
    const Vec3r x_body = _box->globalToBody(x);
    
    // with x in body coordinates, we can find the gradient evaluated at x for an axis-aligned box centered at the origin
    // and then rotate the gradient back to global coordinates
    // we ultiately want to avoid branches (i.e. if statements) as much as possible for max speed

    // put x into the first quadrant and find the distance between each coordinate and the box semisize
    const Vec3r q = x_body.cwiseAbs() - _box->size()/2.0;

    // if any of these are positive, x is outside the box
    const Real q0 = std::max(q[0], Real(0.0));
    const Real q1 = std::max(q[1], Real(0.0));
    const Real q2 = std::max(q[2], Real(0.0));

    // get the sign of each component of x in body coordinates
    const int sign0 = x_body[0] > 0 ? 1 : -1;
    const int sign1 = x_body[1] > 0 ? 1 : -1;
    const int sign2 = x_body[2] > 0 ? 1 : -1;

    // if the max of the q vector is negative, x is inside the box
    const Real max_q = q.maxCoeff();
    const Real min_dist = std::min(max_q, Real(0.0));

    // when x is outside the box, the gradient is the normalized vector of all positive components of q
    // use the sign of x_body to account for when x_body is not in the first quadrant
    const Vec3r grad_when_outside = Vec3r({sign0*q0, sign1*q1, sign2*q2}).normalized();
    
    // when x is inside the box, the gradient is axis-aligned in the direction of the closest box face
    // again, use the sign of x_body to account for when x_body is not in the first quadrant
    const Vec3r grad_when_inside(sign0*static_cast<Real>(q[0]==min_dist), sign1*static_cast<Real>(q[1]==min_dist), sign2*static_cast<Real>(q[2]==min_dist));
    const Vec3r grad = grad_when_outside + grad_when_inside;

    // transform the gradient vector back into global coordinates        
    return GeometryUtils::rotateVectorByQuat(grad, _box->orientation());
}

 #ifdef HAVE_CUDA
inline void BoxSDF::createGPUResource() 
{
    _gpu_resource = std::make_unique<Sim::BoxSDFGPUResource>(this);
    _gpu_resource->allocate();
}
 #endif

void BoxSDF::serialize(std::vector<std::byte>& buf) const
{
    pack(buf, _box);
}

void BoxSDF::deserialize(const std::byte*& buf)
{
    unpack(buf, _box);
}

} // namespace Geometry