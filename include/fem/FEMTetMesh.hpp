#ifndef __FEM_MESH_HPP
#define __FEM_MESH_HPP

#include "geometry/Mesh.hpp"
#include "geometry/TetMesh.hpp"

#include <array>

namespace FEM
{
    
/** A wrapper around Geometry::TetMesh that implements the finite element method for scalar-valued functions (i.e. temperature, voltage). */
class FEMTetMesh
{
public:
    /** Typedef for the gradients of the element shape functions. Note that the gradient as column vector convention is used. */
    using ElementShapeFunctionGradientsMat = Eigen::Matrix<Real, 3, 4>;
    
    /** Typedef for the gradients of the face shape functions. Note that the gradient as column vector convention is used. */
    using FaceJacobianMat = Eigen::Matrix<Real, 2, 3>;

    /** Gauss quadrature points and weights */
    // for triangular faces
    static constexpr int NUM_FACE_QUADRATURE_PTS = 3;
    static constexpr std::array<Real, 3> FACE_QUADRATURE_e1 = {1.0/6.0, 2.0/3.0, 1.0/6.0};
    static constexpr std::array<Real, 3> FACE_QUADRATURE_e2 = {1.0/6.0, 1.0/6.0, 2.0/3.0};
    static constexpr std::array<Real, 3> FACE_QUADRATURE_weights = {1.0/6.0, 1.0/6.0, 1.0/6.0};
    // for tetrahedral elements
    static constexpr int NUM_ELEMENT_QUADRATURE_PTS = 1;
    static constexpr std::array<Real, 1> ELEMENT_QUADRATURE_e1 = {0.25};
    static constexpr std::array<Real, 1> ELEMENT_QUADRATURE_e2 = {0.25};
    static constexpr std::array<Real, 1> ELEMENT_QUADRATURE_e3 = {0.25};
    static constexpr std::array<Real, 1> ELEMENT_QUADRATURE_weights = {0.25};

    FEMTetMesh(const Geometry::TetMesh* mesh);

    /** Element-related computations */

    /** Computes the element Jacobian (for linear tetrahedra) for the specified element.
     * Useful for computing volume integrals and tetrahedral shape function derivatives:
     *  - The determinant of this Jacobian maps volumes in the parent domain to the physical domain.
     *  - The inverse of this Jacobian maps shape function derivatives w.r.t natural coordinates to shape function derivatives w.r.t physical coordinates.
     */
    Mat3r elementJacobian(int element_index) const;

    /** Computes the shape function matrix at the specified natural coordinates (e1, e2, e3).
     */
    Vec4r elementShapeFunctions(Real e1, Real e2, Real e3) const;

    /** Computes the gradients of the shape functions for the specified element.
     * These are constant throughout the element, so no natural coordinates need to be given.
     */
    ElementShapeFunctionGradientsMat elementShapeFunctionGradients(int element_index) const;


    /** Face-related computations */

    /** Computes the face Jacobian (i.e. for a straight-sided triangle) for the specified face.
     * Useful for computing area (surface) integrals and surface shape function derivatives used by the natural boundary term:
     *  - The determinant of this Jacobian maps areas in the parent domain to the physical domain.
     *  - The inverse of this Jacobian maps shape function derivatives w.r.t natural coordinates to shape function derivatives w.r.t physical coordinates.
     */
    FaceJacobianMat faceJacobian(int face_index) const;

    /** Computes the shape function for the specified face, at the specified natural coordinates (e1, e2). */
    Vec3r faceShapeFunctions(Real e1, Real e2) const;
    


    

private:
    const Geometry::TetMesh* _mesh;
};

} // namespace FEM

#endif // __FEM_MESH_HPP