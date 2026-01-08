#ifndef __HEAT_CONDUCTION_FEM_SOLVER_HPP
#define __HEAT_CONDUCTION_FEM_SOLVER_HPP


#include "geometry/RefinedTetMesh.hpp"
#include "fem/FEMTetMesh.hpp"
#include "fem/VoltageFEMSolver.hpp"

#include "simobject/ElasticMaterial.hpp"

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
    HeatConductionFEMSolver(Geometry::RefinedTetMesh* mesh, const ElasticMaterial& material, Real h, Real T_a);

    /** Adds a new essential boundary condition for temperature at the specified index. */
    void setTemperatureAtBoundary(int vertex_index, Real value);

    /** Clears all temperature essential boundary conditions. */
    void clearTemperatureBoundary();

    /** Specifies voltage at a vertex. */
    void setVoltageAtBoundary(int vertex_index, Real voltage, bool permanent=false);

    /** Clears all voltage essential boundary conditions. */
    void clearVoltageBoundary();

    /** Solves the Laplace equation on the tetrahedral mesh given the current essential boundary conditions.
     * Returns the nodal values.
     */
    VecXr solve();

    /** Steps the Laplace equation forward in time using Forward Euler integration. */
    void step(Real dt);

private:
    /** Allocates the appropriate amount of memory for the FEM system. */
    void _allocateMemory();

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
    Geometry::RefinedTetMesh* _mesh;
    /** Wrapper around the tet mesh for doing FEM calculations */
    FEMTetMesh _fem_mesh;
    /** Laplace equation solver used to solve for the voltage potential and its gradient. */
    VoltageFEMSolver _voltage_solver;

    /** Material constants */

    const ElasticMaterial& _material;

    /** The convective heat transfer coefficient. */
    Real _h;

    /** The ambient temperature. */
    Real _T_a;

    /** Map that stores the essential boundary.
     * The key is the vertex index on the boundary, the value is the value at the essential boundary.
     */
    std::unordered_map<int, Real> _essential_boundary;

    /** Tempuratures */
    std::vector<Real> _T;
    std::vector<Real> _T_prev;

    /** Whether or not vertex index is on temperature essential boundary. */
    std::vector<bool> _on_essential_boundary;

    /** Lumped thermal masses - approximate this as constant throughout the sim. */
    std::vector<Real> _M;

    /** Track the latest topology version of the mesh. Allows us to detect topology changes and reallocate accordingly. */
    unsigned long _latest_topology_version;
};

} // namespace FEM


#endif // __HEAT_CONDUCTION_FEM_SOLVER_HPP