#ifndef __TRANSFORMATION_MATRIX_HPP
#define __TRANSFORMATION_MATRIX_HPP

#include <Eigen/Dense>

#include "utils/GeometryUtils.hpp"

namespace Geometry
{

class TransformationMatrix
{
    public:
    explicit TransformationMatrix()
        : _matrix(Mat4r::Identity())
    {}

    explicit TransformationMatrix(const Mat3r& rot_mat, const Vec3r& origin)
        : _matrix(Mat4r::Identity())
    {
        _matrix.block<3, 3>(0,0) = rot_mat;
        _matrix.block<3, 1>(0,3) = origin;
    }

    explicit TransformationMatrix(const Mat4r& t_mat)
        : _matrix(t_mat)
    {}

    bool operator==(const TransformationMatrix& other) const
    {
        return _matrix.isApprox(other._matrix, 1e-8);
    }

    TransformationMatrix operator*(const TransformationMatrix& other) const
    {
        const Mat4r res = _matrix * other._matrix;
        return TransformationMatrix(res);
    }

    void operator*=(const TransformationMatrix& other)
    {
        _matrix *= other._matrix;
    }

    TransformationMatrix operator+(const TransformationMatrix& other) const
    {
        const Mat4r res = _matrix + other._matrix;
        return TransformationMatrix(res);
    }

    TransformationMatrix operator-(const TransformationMatrix& other) const
    {
        const Mat4r res = _matrix - other._matrix;
        return TransformationMatrix(res);
    }

    const Mat4r& asMatrix() const { return _matrix; }

    Vec3r translation() const { return _matrix.block<3, 1>(0,3); }
    void setTranslation(const Vec3r& new_trans) { _matrix.block<3, 1>(0,3) = new_trans; }

    Mat3r rotMat() const { return _matrix.block<3,3>(0,0); }

    TransformationMatrix inverse() const
    {
        return TransformationMatrix(rotMat().transpose(), -rotMat().transpose()*translation());
    }

    Mat6r adjoint() const
    {
        Mat6r mat;
        mat << rotMat(), Mat3r::Zero(), GeometryUtils::Bracket_so3(translation())*rotMat(), rotMat();
        return mat;
    }

    private:
    Mat4r _matrix;
};

} // namespace Geometry

#endif // __TRANSFORMATION_MATRIX_HPP