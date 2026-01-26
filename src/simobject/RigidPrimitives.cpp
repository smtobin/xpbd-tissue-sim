#include "simobject/RigidPrimitives.hpp"
#include "utils/GeometryUtils.hpp"

namespace Sim
{

RigidSphere::RigidSphere(const Simulation* sim, const ConfigType* config)
    : RigidObject(sim, config)
{
    _radius = config->radius();

    // mass is volume * density = 4/3 * pi * r^3 * density
    _m = 4.0/3.0 * 3.1415 * _radius * _radius * _radius * config->density();

    // moment of inertia of sphere about axis through its center is 2/5 * m * r^2
    _I(0,0) = 2.0/5.0 * _m * _radius * _radius;
    _I(1,1) = 2.0/5.0 * _m * _radius * _radius;
    _I(2,2) = 2.0/5.0 * _m * _radius * _radius;

    _I_inv = _I.inverse();

    // set the characteristic dimension to the diameter
    _char_dim = 2*_radius;
}

std::string RigidSphere::toString(const int indent) const
{
    // TODO: better toString
    return RigidObject::toString(indent);
}

void RigidSphere::setup()
{
    // no setup needed for now
}

Geometry::AABB RigidSphere::boundingBox() const
{
    return Geometry::AABB(_p.array() - _radius, _p.array() + _radius);
}



/////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////



RigidBox::RigidBox(const Simulation* sim, const ConfigType* config)
    : RigidObject(sim, config)
{
    _size = config->size();

    _m = _size[0] * _size[1] * _size[2] * config->density();
    _I(0,0) = 1.0/12.0 * _m * (_size[1]*_size[1] + _size[2]*_size[2]);
    _I(1,1) = 1.0/12.0 * _m * (_size[0]*_size[0] + _size[2]*_size[2]);
    _I(2,2) = 1.0/12.0 * _m * (_size[0]*_size[0] + _size[1]*_size[1]);

    _I_inv = _I.inverse();

    _origin_bbox_points.col(0) = Vec3r({-_size[0]/2, -_size[1]/2, -_size[2]/2});
    _origin_bbox_points.col(1) = Vec3r({ _size[0]/2, -_size[1]/2, -_size[2]/2});
    _origin_bbox_points.col(2) = Vec3r({ _size[0]/2,  _size[1]/2, -_size[2]/2});
    _origin_bbox_points.col(3) = Vec3r({-_size[0]/2,  _size[1]/2, -_size[2]/2});
    _origin_bbox_points.col(4) = Vec3r({-_size[0]/2, -_size[1]/2,  _size[2]/2});
    _origin_bbox_points.col(5) = Vec3r({ _size[0]/2, -_size[1]/2,  _size[2]/2});
    _origin_bbox_points.col(6) = Vec3r({ _size[0]/2,  _size[1]/2,  _size[2]/2});
    _origin_bbox_points.col(7) = Vec3r({-_size[0]/2,  _size[1]/2,  _size[2]/2});

    // set the characteristic dimension to the minimum side length
    _char_dim = _size.minCoeff();
}

std::string RigidBox::toString(const int indent) const
{
    // TODO: better toString
    return RigidObject::toString(indent);
}

void RigidBox::setup()
{
    // no setup needed for now
}

Geometry::AABB RigidBox::boundingBox() const
{
    // transform the corners of the box into its current coordinates
    // and find the bounding box around that
    const Mat3r rot_mat = GeometryUtils::quatToMat(_q);
    
    // transform the original bounding box (centered at the origin) to the box's current position and orientation
    Eigen::Matrix<Real, 3, 8> transformed_bbox_points = rot_mat * _origin_bbox_points;
    transformed_bbox_points.colwise() += _p;

    // found the AABB around the transformed bounding box
    const Vec3r min = transformed_bbox_points.rowwise().minCoeff();
    const Vec3r max = transformed_bbox_points.rowwise().maxCoeff();
    return Geometry::AABB(min, max);

}



/////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////



RigidCylinder::RigidCylinder(const Simulation* sim, const ConfigType* config)
    : RigidObject(sim, config)
{
    _radius = config->radius();
    _height = config->height();

    _m = 3.1415 * _radius * _radius * _height * config->density();
    _I(0,0) = 1.0/12.0 * _m * (3*_radius*_radius + _height*_height);
    _I(1,1) = 1.0/12.0 * _m * (3*_radius*_radius + _height*_height);
    _I(2,2) = 1.0/2.0 * _m * _radius * _radius;

    _I_inv = _I.inverse();

    _origin_bbox_points.col(0) = Vec3r({-_radius, -_radius, -_height/2});
    _origin_bbox_points.col(1) = Vec3r({ _radius, -_radius, -_height/2});
    _origin_bbox_points.col(2) = Vec3r({ _radius,  _radius, -_height/2});
    _origin_bbox_points.col(3) = Vec3r({-_radius,  _radius, -_height/2});
    _origin_bbox_points.col(4) = Vec3r({-_radius, -_radius,  _height/2});
    _origin_bbox_points.col(5) = Vec3r({ _radius, -_radius,  _height/2});
    _origin_bbox_points.col(6) = Vec3r({ _radius,  _radius,  _height/2});
    _origin_bbox_points.col(7) = Vec3r({-_radius,  _radius,  _height/2});

    // set the characteristic dimension to the smaller between the diameter and the height
    _char_dim = std::min(_height, _radius*2);
}

std::string RigidCylinder::toString(const int indent) const
{
    // TODO: better toString
    return RigidObject::toString(indent);
}

void RigidCylinder::setup()
{
    // no setup needed right now
}

Geometry::AABB RigidCylinder::boundingBox() const
{
    // transform the corners of the box into its current coordinates
    // and find the bounding box around that
    const Mat3r rot_mat = GeometryUtils::quatToMat(_q);
    
    // transform the original bounding box (centered at the origin) to the box's current position and orientation
    Eigen::Matrix<Real, 3, 8> transformed_bbox_points = rot_mat * _origin_bbox_points;
    transformed_bbox_points.colwise() += _p;

    // found the AABB around the transformed bounding box
    const Vec3r min = transformed_bbox_points.rowwise().minCoeff();
    const Vec3r max = transformed_bbox_points.rowwise().maxCoeff();
    return Geometry::AABB(min, max);

}


} // namespace Sim