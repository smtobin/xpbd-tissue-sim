#ifndef __VOLTAGE_FEM_SOLVER_HPP
#define __VOLTAGE_FEM_SOLVER_HPP

#include "geometry/RefinedTetMesh.hpp"
#include "fem/FEMTetMesh.hpp"

#include <petscksp.h>

#include <unordered_map>

namespace FEM
{

/** Solves the Laplace equation
 *      del * (k delV) = 0
 * using the finite element method over a tetrahedral mesh with linear elements, with a pseudo-time iteration.
 * 
 * Rather than solve the Laplace equation directly, we integrate with explicit Euler.
 * 
 * k is taken as constant throughout the mesh (for now), though in theory it can vary per element.
 * 
 * For now, all boundary flux (i.e. the natural boundary) is assumed to be 0.
 */
class VoltageFEMSolver
{
public:

    // use row-major ordering for the elemental stiffness matrices, since this ordering is what PETSc expects.
    // default in Eigen is column-major
    using ElementStiffnessMatrixType = Eigen::Matrix<Real, 4, 4, Eigen::RowMajor>;

    VoltageFEMSolver(Geometry::RefinedTetMesh* mesh, Real k);

    /** Adds a new essential boundary condition at the specified index.
     *   i.e. RHS[index] = value
     */
    void setVoltageAtBoundary(int vertex_index, Real value, bool permanent=false);

    /** Clears all essential boundary conditions. */
    void clearVoltageBoundary();

    /** Solves the Laplace equation on the tetrahedral mesh given the current essential boundary conditions.
     * Returns the nodal values.
     */
    VecXr solve();

    /** Steps the Laplace equation forward in time using Forward Euler integration. */
    void step(Real dt);

    const std::vector<Real>& voltage() const { return _V; }

private:
    /** Allocates the appropriate amount of memory for the FEM system. */
    void _allocateMemory();

    /** Computes the elemental stiffness matrix using a 1-point Gauss quadrature (the centroid of the tet) */
    ElementStiffnessMatrixType _elementStiffnessMatrix(int element_index) const;

    /** Assembles the global system matrix and RHS vector. */
    void _assembly();

private:
    Geometry::RefinedTetMesh* _mesh;
    FEMTetMesh _fem_mesh;

    /** The constant in the Laplace equation. */
    Real _k;

    /** Map that stores the "temporary" essential boundary.
     * These get erased with clearVoltageBoundary().
     * The key is the vertex index on the boundary, the value is the value at the essential boundary.
     */
    std::unordered_map<int, Real> _temporary_essential_boundary;

    /** Map that stores the "permanent" essential boundary.
     * These DO NOT get erased with clearVoltageBoundary().
     * The key is the vertex index, the value is the value at the essential boundary.
     */
    std::unordered_map<int, Real> _permanent_essential_boundary;

    /** Whether or not vertex index is on temperature essential boundary. */
    std::vector<bool> _on_essential_boundary;

    std::vector<Real> _V;
    std::vector<Real> _V_prev;

    /** The global system matrix. */
    // MatXr _system_matrix;
    /** The global RHS vector. */
    // VecXr _RHS_vec;

    /** PETSc global system matrix. */
    Mat _A;
    /** PETSc global RHS vector. */
    Vec _b;
    /** PETSc global solution vector. */
    Vec _x;
    /** PETSc linear solver context. */
    KSP _ksp;

    /** Track the number of vertices and elements in the mesh.
     * When these change, we need to reallocate memory.
     * 
     * TODO: Is this a reliable way to detect if we need to reallocate memory?
     */
    int _prev_num_vertices;
    int _prev_num_elements;
};

} // namespace FEM

#endif // __VOLTAGE_FEM_SOLVER_HPP