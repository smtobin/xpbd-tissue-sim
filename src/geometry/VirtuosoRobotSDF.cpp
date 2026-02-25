#include "geometry/VirtuosoRobotSDF.hpp"

#include "simobject/VirtuosoRobot.hpp"

namespace Geometry
{

VirtuosoRobotSDF::VirtuosoRobotSDF(Sim::VirtuosoRobot* virtuoso_robot)
    : _virtuoso_robot(virtuoso_robot), _arm1_sdf(nullptr), _arm2_sdf(nullptr)
{
    if (_virtuoso_robot->hasArm1())
    {
        virtuoso_robot->arm1()->createSDF();
        _arm1_sdf = _virtuoso_robot->arm1()->SDF();
    }

    if (_virtuoso_robot->hasArm2())
    {
        virtuoso_robot->arm2()->createSDF();
        _arm2_sdf = _virtuoso_robot->arm2()->SDF();
    }
}

Real VirtuosoRobotSDF::evaluate(const Vec3r& x) const
{
    Real d1 = std::numeric_limits<Real>::infinity();
    Real d2 = std::numeric_limits<Real>::infinity();

    if (_arm1_sdf)
        d1 = _arm1_sdf->evaluate(x);
    if (_arm2_sdf)
        d2 = _arm2_sdf->evaluate(x);

    return std::min(d1, d2);
}

Vec3r VirtuosoRobotSDF::gradient(const Vec3r& x) const
{
    Real d1 = std::numeric_limits<Real>::infinity();
    Real d2 = std::numeric_limits<Real>::infinity();

    if (_arm1_sdf)
        d1 = _arm1_sdf->evaluate(x);
    if (_arm2_sdf)
        d2 = _arm2_sdf->evaluate(x);

    if (d1 < d2)
        return _arm1_sdf->gradient(x);
    else
        return _arm2_sdf->gradient(x);
}

void VirtuosoRobotSDF::serialize(std::vector<std::byte>& buf) const
{
    pack(buf, _virtuoso_robot);
    pack(buf, _arm1_sdf);
    pack(buf, _arm2_sdf);
}

void VirtuosoRobotSDF::deserialize(const std::byte*& buf)
{
    unpack(buf, _virtuoso_robot);
    unpack(buf, _arm1_sdf);
    unpack(buf, _arm2_sdf);
}

} // namespace Geometry