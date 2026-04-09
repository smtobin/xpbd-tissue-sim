#ifndef __COORDINATE_FRAME_HPP
#define __COORDINATE_FRAME_HPP

// #include <Eigen/Dense>
#include "geometry/TransformationMatrix.hpp"

namespace Geometry
{

class CoordinateFrame
{
    public:
    explicit CoordinateFrame(const TransformationMatrix& transform)
        : _transform(transform) 
    {
    }

    explicit CoordinateFrame(TransformationMatrix&& transform)
        : _transform(std::move(transform))
    {
    }
    
    explicit CoordinateFrame(const Mat4r& transform)
        : _transform(transform)
    {
    }

    explicit CoordinateFrame()
        : _transform(Mat4r::Identity())
    {
    }

    void serialize(std::vector<std::byte>& buf) const
    {
        pack(buf, _transform);
    }
    void deserialize(const std::byte*& buf)
    {
        unpack(buf, _transform);
    }

    CoordinateFrame operator*(const TransformationMatrix& transform) const
    {
        return CoordinateFrame(_transform * transform);
    }

    const TransformationMatrix& transform() const { return _transform; }
    Vec3r origin() const { return _transform.translation(); }

    /** Sets location and orientation of the coordinate frame via a specified SE3 transformation matrix. */
    void setTransform(const Mat4r& new_transform) { _transform = TransformationMatrix(new_transform); }

    /** Sets the origin of the coordinate frame, keeping the rotation of the coordinate axes the same. */
    void setOrigin(const Vec3r& new_origin) { _transform.setTranslation(new_origin); }

    /** Transform the coordinate frame by a specified SE3 transformation matrix. */
    void applyTransform(const TransformationMatrix& transform) { _transform *= transform; } 

    protected:
    // Mat4r _transform;     // transform relative to the global origin, SE3 4x4 transformation matrix
    TransformationMatrix _transform;
};

} // namespace Geometry

#endif // __COORDINATE_FRAME_HPP