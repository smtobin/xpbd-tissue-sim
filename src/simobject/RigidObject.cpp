#include "simobject/RigidObject.hpp"
#include "utils/GeometryUtils.hpp"
#include "simulation/Simulation.hpp"

namespace Sim
{

RigidObject::RigidObject(const Simulation* sim, const ConfigType* config)
    : Object(sim, config)
{
    _p = config->initialPosition();
    _p_prev = _p;
    const Vec3r& initial_rotation_rad = config->initialRotation() * 3.1415 / 180.0;
    _q = GeometryUtils::eulXYZ2Quat(initial_rotation_rad[0], initial_rotation_rad[1], initial_rotation_rad[2]);
    _q_prev = _q;

    _v = config->initialVelocity();
    _w = config->initialAngularVelocity();

    _fixed = config->fixed();
}

std::string RigidObject::toString(const int indent) const
{
    const std::string indent_str(indent, '\t');
    std::stringstream ss;
    ss << indent_str << "=====" << type() << "=====" << std::endl;
    ss << indent_str << "Position: (" << _p[0] << ", " << _p[1] << ", " << _p[2] << ")" << std::endl;
    ss << indent_str << "Orientation: (" << _q[0] << ", " << _q[1] << ", " << _q[2] << ", " << _q[3] << std::endl;
    ss << indent_str << "Translational Velocity: (" << _v[0] << ", " << _v[1] << ", " << _v[2] << ")" << std::endl;
    ss << indent_str << "Angular Velocity: (" << _w[0] << ", " << _w[1] << ", " << _w[2] << ")" << std::endl;
    ss << indent_str << "Mass: " << _m << std::endl;
    ss << indent_str << "Inertia Matrix: " << _I << std::endl;

    return ss.str();
}

void RigidObject::update()
{
    if (_fixed)
        return;

    // update positions inertially
    const Vec3r f_ext({0,0,-_m*_sim->gAccel()});
    _v = _v + _sim->dt() * f_ext / _m;
    _p = _p + _sim->dt() * _v;

    // update orientation inertially
    const Vec3r t_ext({0.0, 0.0, 0.0});
    Vec3r _w_body = GeometryUtils::rotateVectorByQuat(_w, GeometryUtils::inverseQuat(_q));
    _w_body = _w_body + _sim->dt() * _I_inv * (t_ext - (_w_body.cross(_I*_w_body)));
    _w = GeometryUtils::rotateVectorByQuat(_w_body, _q);
    const Vec4r w4({_w[0], _w[1], _w[2], 0.0});
    _q = _q + 0.5 * _sim->dt() * (GeometryUtils::quatMult(w4, _q));
    _q.normalize();

    // TODO: solve constraints here

}

void RigidObject::velocityUpdate()
{
    if (_fixed)
        return;

    // TODO: uncomment this (commenting out so that the rigid cursor object for VirtuosoSimulation does not accumulate velocity when it follows the mouse)
    // update linear velocity
    _v = (_p - _p_prev) / _sim->dt();

    // update angular velocity
    const Vec4r dq = GeometryUtils::quatMult(_q, GeometryUtils::inverseQuat(_q_prev));
    _w = 2 / _sim->dt() * dq(Eigen::seq(0,2));
    if (dq[3] < 0)  
        _w = -_w;

    _p_prev = _p;
    _q_prev = _q;
}

void RigidObject::applyForceAtPoint(const Vec3r& f, const Vec3r& p) 
{
    // update position
    _p = _p + _sim->dt() * _sim->dt() * f / _m;

    // update orientation
    const Vec3r torque = (p - _p).cross(f);
    const Vec3r body_torque = GeometryUtils::rotateVectorByQuat(torque, GeometryUtils::inverseQuat(_q));
    const Vec3r body_omega = _sim->dt() * _I_inv * body_torque;
    const Vec3r global_omega = GeometryUtils::rotateVectorByQuat(body_omega, _q);
    const Vec4r w4({global_omega[0], global_omega[1], global_omega[2], 0.0});
    _q = _q + 0.5 * _sim->dt() * (GeometryUtils::quatMult(w4, _q));
}

// Vec3r RigidObject::globalToBody(const Vec3r& x) const
// {
//     return GeometryUtils::rotateVectorByQuat(x - _p, GeometryUtils::inverseQuat(_q));
// }

// Vec3r RigidObject::bodyToGlobal(const Vec3r& x) const
// {
//     return _p + GeometryUtils::rotateVectorByQuat(x, _q);
// }

Geometry::TransformationMatrix RigidObject::transform() const
{
    return Geometry::TransformationMatrix(GeometryUtils::quatToMat(_q), _p);
}


} // namespace Simulation