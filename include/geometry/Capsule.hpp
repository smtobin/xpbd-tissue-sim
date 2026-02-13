#ifndef __CAPSULE_HPP
#define __CAPSULE_HPP

#include "common/types.hpp"

namespace Geometry
{

class Capsule
{
public:
    Capsule(const Vec3r& p1, const Vec3r& p2, Real radius)
        : _p1(p1), _p2(p2), _radius(radius)
    {}

    const Vec3r& p1() const { return _p1; }
    const Vec3r& p2() const { return _p2; }
    Real radius() const { return _radius; }

private:
    Vec3r _p1;
    Vec3r _p2;
    Real _radius;
};

} // namespace Geometry

#endif // __CAPSULE_HPP