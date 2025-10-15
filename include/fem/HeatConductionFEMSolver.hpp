#ifndef __HEAT_CONDUCTION_FEM_SOLVER_HPP
#define __HEAT_CONDUCTION_FEM_SOLVER_HPP


#include "geometry/TetMesh.hpp"
#include "fem/FEMTetMesh.hpp"
#include "fem/LaplacianFEMSolver.hpp"

#include <unordered_map>

namespace FEM
{

/** Solves the heat conduction equation
 *      rho * c * dT/dt = k * del * delT + q_g
 * using the finite element method over a tetrahedral mesh with linear elements.
 * 
 * k is taken as constant throughout the mesh (for now), though in theory it can vary per element.
 * 
 * q_g is the heat generation term, which is computed from Joule heating
 *      q_g = sigma * || delV ||^2
 * where delV is the voltage potential gradient and sigma is the electrical conductivity.
 * 
 * Convective heat loss at the surface is assumed
 *      (k delT) * n + h(T - T_a) = 0
 * where h is the convection heat transfer coefficient, and T_a is the ambient temperature.
 */
class HeatConductionFEMSolver
{
public:
    HeatConductionFEMSolver(Geometry::TetMesh* mesh, Real rho, Real c, Real k, Real sigma, Real h, Real T_a);

    /** Adds a new essential boundary condition for temperature at the specified index. */
    void setTemperatureAtBoundary(int vertex_index, Real value);

    /** Clears all temperature essential boundary conditions. */
    void clearTemperatureBoundary();

    /** Specifies voltage at a vertex. */
    void setVoltageAtBoundary(int vertex_index, Real voltage);

    /** Clears all voltage essential boundary conditions. */
    void clearVoltageBoundary();

    /** Solves the Laplace equation on the tetrahedral mesh given the current essential boundary conditions.
     * Returns the nodal values.
     */
    VecXr solve();

    /** Steps the Laplace equation forward in time using Forward Euler integration. */
    void step(Real dt);

private:
    /** Computes the elemental stiffness matrix using a 1-point Gauss quadrature (the centroid of the tet) */
    Mat4r _elementStiffnessMatrix(int element_index) const;

    /** Computes an element's contribution to the RHS vector, using a 1-point Gauss quadrature (the centroid of the tet) */
    Vec4r _elementRHSVector(int element_index) const;

    /** Computes a surface face's contribution to the stiffness matrix, using 1-point Gauss quadrature */
    Mat3r _faceStiffnessMatrix(int face_index) const;

    /** Assembles the global system matrix and RHS vector. */
    void _assembly();

private:
    /** The tetrahedral mesh */
    Geometry::TetMesh* _mesh;
    /** Wrapper around the tet mesh for doing FEM calculations */
    FEMTetMesh _fem_mesh;
    /** Laplace equation solver used to solve for the voltage potential and its gradient. */
    LaplacianFEMSolver _laplace_solver;

    /** Material constants */

    /** The density. */
    Real _rho;

    /** The specific heat capacity. */
    Real _c;

    /** The thermal conductivity. */
    Real _k;

    /** The electrical conductivity. */
    Real _sigma;

    /** The convective heat transfer coefficient. */
    Real _h;

    /** The ambient temperature. */
    Real _T_a;

    /** Map that stores the essential boundary.
     * The key is the vertex index on the boundary, the value is the value at the essential boundary.
     */
    std::unordered_map<int, Real> _essential_boundary;

    /** The voltage potential at each vertex (from Laplace solver) */
    VecXr _V;
    /** The global system matrix. */
    MatXr _system_matrix;
    /** The global RHS vector. */
    VecXr _RHS_vec;

    /** Tempuratures */
    std::vector<Real> _T;
    std::vector<Real> _T_prev;

    /** Whether or not vertex index is on temperature essential boundary. */
    std::vector<bool> _on_essential_boundary;

    /** Lumped thermal masses - approximate this as constant throughout the sim. */
    std::vector<Real> _M;
};

} // namespace FEM


#endif // __HEAT_CONDUCTION_FEM_SOLVER_HPP