#ifndef __LAPLACIAN_FEM_SOLVER_HPP
#define __LAPLACIAN_FEM_SOLVER_HPP

#include "geometry/TetMesh.hpp"
#include "fem/FEMTetMesh.hpp"

#include <unordered_map>

namespace FEM
{

/** Solves the Laplace equation
 *      del * (k delV) = 0
 * using the finite element method over a tetrahedral mesh with linear elements.
 * 
 * k is taken as constant throughout the mesh (for now), though in theory it can vary per element.
 * 
 * For now, all boundary flux (i.e. the natural boundary) is assumed to be 0.
 */
class LaplacianFEMSolver
{
public:
    LaplacianFEMSolver(const Geometry::TetMesh* mesh, Real k);

    /** Adds a new essential boundary condition at the specified index.
     *   i.e. RHS[index] = value
     */
    void setEssentialBoundary(int vertex_index, Real value);

    /** Clears all essential boundary conditions. */
    void clearEssentialBoundary();

    /** Solves the Laplace equation on the tetrahedral mesh given the current essential boundary conditions.
     * Returns the nodal values.
     */
    VecXr solve();

private:
    /** Computes the elemental stiffness matrix using a 1-point Gauss quadrature (the centroid of the tet) */
    Mat4r _elementStiffnessMatrix(int element_index) const;

    /** Assembles the global system matrix and RHS vector. */
    void _assembly();

private:
    const Geometry::TetMesh* _mesh;
    FEMTetMesh _fem_mesh;

    /** The constant in the Laplace equation. */
    Real _k;

    /** Map that stores the essential boundary.
     * The key is the vertex index on the boundary, the value is the value at the essential boundary.
     */
    std::unordered_map<int, Real> _essential_boundary;

    /** The global system matrix. */
    MatXr _system_matrix;
    /** The global RHS vector. */
    VecXr _RHS_vec;
};

} // namespace FEM

#endif