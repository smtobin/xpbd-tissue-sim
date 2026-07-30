#ifndef __AABB_HPP
#define __AABB_HPP

#include "common/types.hpp"

namespace Geometry
{

/** An axis-aligned bounding box (AABB). */
struct AABB
{
    Vec3r min;    // the min coordinates of the bounding box
    Vec3r max;    // the max coordinates of the bounding box

    /** Initialization from vectors */
    AABB(const Vec3r& min_, const Vec3r& max_)
        : min(min_), max(max_)
    {}

    /** Initialization from individual coordinates */
    AABB(const Real min_x, const Real min_y, const Real min_z, const Real max_x, const Real max_y, const Real max_z)
        : min(min_x, min_y, min_z), max(max_x, max_y, max_z)
    {}

    AABB()
        : min(Vec3r::Constant(std::numeric_limits<Real>::max())), max(Vec3r::Constant(std::numeric_limits<Real>::lowest()))
    {
    }

    /** Returns the center of the bounding box. */
    Vec3r center() const
    {
        return min + 0.5*(max - min);
    }

    /** Returns the size of the bounding box. */
    Vec3r size() const
    {
        return max - min;
    }

    /** Returns true if two AABBs overlap, false otherwise. */
    bool collidesWith(const AABB& other) const
    {
        return (min[0] <= other.max[0] && max[0] >= other.min[0]) &&
               (min[1] <= other.max[1] && max[1] >= other.min[1]) &&
               (min[2] <= other.max[2] && max[2] >= other.min[2]);
    }
};

} // namespace Geometry

#endif // __AABB_HPP