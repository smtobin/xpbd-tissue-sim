#include "solver/constraint/RigidBodyConstraint.hpp"
#include "simobject/RigidObject.hpp"

namespace Solver
{

Real PositionalRigidBodyXPBDHelper::weight() const
{
    // calculate the point on the rigid body in the body frame
    const Vec3r r_body = _rigid_obj->globalToBody(_point_on_body);
    // calculate the direction of the correction in the body frame
    const Vec3r n_body = GeometryUtils::rotateVectorByQuat(_direction, GeometryUtils::inverseQuat(_rigid_obj->orientation()));
    
    // use the defined formula to get the weight
    const Vec3r r_cross_n = r_body.cross(n_body);
    return 1.0/_rigid_obj->mass() + r_cross_n.transpose() * _rigid_obj->invI() * r_cross_n;
}

void PositionalRigidBodyXPBDHelper::update(Real dlam, Real* position_update, Real* orientation_update) const
{
    // calculate the point on the rigid body in the body frame
    const Vec3r r_body = _rigid_obj->globalToBody(_point_on_body);
    // calculate the direction of the correction in the body frame
    const Vec3r n_body = GeometryUtils::rotateVectorByQuat(_direction, GeometryUtils::inverseQuat(_rigid_obj->orientation()));
    
    // compute the position update (in the global frame)
    const Vec3r pos_update = dlam * _direction / _rigid_obj->mass();

    // compute the body angular velocity (with quantities in the body frame, since I is in the rest state i.e. body frame)
    const Vec3r omega_body = 0.5 * _rigid_obj->invI() * (r_body.cross(dlam * n_body));
    // convert body anuglar velocity to spatial angular velocity
    const Vec3r omega_spatial = GeometryUtils::rotateVectorByQuat(omega_body, _rigid_obj->orientation());
    // compute the orientation update (in the global frame)
    const Vec4r or_update = GeometryUtils::quatMult(Vec4r(omega_spatial[0], omega_spatial[1], omega_spatial[2], 0), _rigid_obj->orientation());

    // populate the update vectors
    position_update[0] = pos_update[0];
    position_update[1] = pos_update[1];
    position_update[2] = pos_update[2];

    orientation_update[0] = or_update[0];
    orientation_update[1] = or_update[1];
    orientation_update[2] = or_update[2];
    orientation_update[3] = or_update[3];
}


Real AngularRigidBodyXPBDHelper::weight() const
{
    // compute rotation axis in the body frame
    const Vec3r rot_axis_body = GeometryUtils::rotateVectorByQuat(_rot_axis, GeometryUtils::inverseQuat(_rigid_obj->orientation()));
    // compute weight based on above formula
    return rot_axis_body.transpose() * _rigid_obj->invI() * rot_axis_body;
}

void AngularRigidBodyXPBDHelper::update(Real dlam, Real* position_update, Real* orientation_update) const
{
    // compute rotation axis in the body frame
    const Vec3r rot_axis_body = GeometryUtils::rotateVectorByQuat(_rot_axis, GeometryUtils::inverseQuat(_rigid_obj->orientation()));
    // compute the body angular velocity (with quantities in the body frame, since I is in the rest state i.e. body frame)
    const Vec3r omega_body = 0.5 * _rigid_obj->invI() * (dlam * rot_axis_body);
    // convert to global angular velocity of the body
    const Vec3r omega_spatial = GeometryUtils::rotateVectorByQuat(omega_body, _rigid_obj->orientation());
    // compute orientation update
    const Vec4r or_update = GeometryUtils::quatMult(Vec4r(omega_spatial[0], omega_spatial[1], omega_spatial[2], 0), _rigid_obj->orientation());

    // populate the update vectors (position update is 0)
    position_update[0] = 0;
    position_update[1] = 0;
    position_update[2] = 0;

    orientation_update[0] = or_update[0];
    orientation_update[1] = or_update[1];
    orientation_update[2] = or_update[2];
    orientation_update[3] = or_update[3];
}

} // namespace Solver