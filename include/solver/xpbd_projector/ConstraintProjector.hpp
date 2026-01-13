#ifndef __CONSTRAINT_PROJECTOR_HPP
#define __CONSTRAINT_PROJECTOR_HPP

#include "solver/constraint/PositionReference.hpp"
#include "solver/constraint/ConstraintReference.hpp"
#include "solver/xpbd_solver/XPBDSolverUpdates.hpp"

#include "common/TypeList.hpp"

#ifdef HAVE_CUDA
#include "gpu/projector/GPUConstraintProjector.cuh"
#endif

namespace Solver
{

/** Performs a XPBD constraint projection for a single constraint.
 *  
 * IsFirstOrder template parameter indicates if the projection follows the 1st-Order XPBD algorithm
 * 
 * Constraint template parameter is the type of constraint being projected (HydrostaticConstraint, DeviatoricConstraint, etc.)
*/
template<bool IsFirstOrder, class Constraint>
class ConstraintProjector
{
    public:
    /** Number of constraints projected */
    constexpr static int NUM_CONSTRAINTS = 1;
    /** Number of rigid bodies involved in the constraint projection. This will be 0 because rigid bodies are handled specially
     * with a different type of constraint projector.
     */
    constexpr static int NUM_RIGID_BODIES = 0;
    /** Maximum number of coordinates involved in the contsraint projection */
    constexpr static int MAX_NUM_COORDINATES = Constraint::NUM_COORDINATES;

    /** List of constraint types being projected (will be a single constraint for this projector) */
    using constraint_type_list = TypeList<Constraint>;
    /** Whether or not the 1st-Order algorithm is used */
    constexpr static bool is_first_order = IsFirstOrder;

    public:
    /** Constructor */
    explicit ConstraintProjector(Real dt, ConstraintReference<Constraint>&& constraint_ref)
        : _dt(dt), _constraint(constraint_ref), _valid(true)
    {
    }

    /** Default constructor - projector marked invalid */
    explicit ConstraintProjector()
        : _valid(false)
    {   
    }

    /** Sets the validity of the ConstraintProjector. When valid, it is assumed the Constraint exists to be projected.
     * @param valid : the new validity of the ConstraintProjector
     */
    void setValidity(bool valid) { _valid = valid; }

    /** @returns whether or not the ConstraintProjector is valid */
    bool isValid() const { return _valid; }

    /** @returns The number of coordinates that the constraint projector affects.
     * For a single constraint projector, this is just the number of coordinates affected by the single constraint.
     */
    int numCoordinates() { return MAX_NUM_COORDINATES; }

    /** @returns the positions (as PositionReferences) affect by the constraint projection. This will just be the 
     * positions of the single projected constraint.
    */
    const std::vector<PositionReference>& positions() const { return _constraint->positions(); }

    const ConstraintReference<Constraint>& constraint() const { return _constraint; }

    Real lambda() const { return _lambda; }

    Real dt() const { return _dt; }

    /** The constraint forces on each of the affected positions caused by this constraint.
     * @returns the constraint forces on each of the affected positions of this constraint. The returned vector is ordered such that
     * the forces are applied to the corresponding position at the same index in the positions() vector.
     * 
     * This method must be called AFTER constraint projection has been performed (i.e. after lambda has been calculated). 
     */
    std::vector<Vec3r> constraintForces() const
    {
        std::vector<Vec3r> forces( Constraint::NUM_POSITIONS );
        // Real delC[Constraint::NUM_COORDINATES];
        // _constraint->gradient(delC);
        for (int i = 0; i < Constraint::NUM_POSITIONS; i++)
        {
            // if 1st-order, F = delC^T * lambda / dt
            // if 2nd-order, F = delC^T * lambda / (dt*dt)
            if constexpr (IsFirstOrder)
                forces[i] = Eigen::Map<const Vec3r>(_delC + 3*i) * _lambda / _dt;
            else
                forces[i] = Eigen::Map<const Vec3r>(_delC + 3*i) * _lambda / (_dt*_dt);
        }

        return forces;
    }

    /** Initialize the Lagrange multipliers */
    void initialize()
    {
        _lambda = 0;
    }

    /** Perform the XPBD constraint projection
     * @param coordinate_updates_ptr (OUTPUT) - a (currently empty) array of CoordinateUpdate structs of numCoordinates() size. Stores the resulting state updates from the XPBD projection. 
     */
    void project(CoordinateUpdate* coordinate_updates_ptr)
    {
        // evalute the constraint's value and gradient
        Real C;
        // Real delC[Constraint::NUM_COORDINATES];
        _constraint->evaluateWithGradient(&C, _delC);

        // if inequality constraint, make sure that the constraint should actually be enforced
        if (C > 0 && _constraint->isInequality())
        {
            for (int i = 0; i < Constraint::NUM_COORDINATES; i++) 
            { 
                coordinate_updates_ptr[i].ptr = nullptr;
                coordinate_updates_ptr[i].update = 0;
            }
            return;
        }

        // compute alpha_tilde
        // if using 1st order XPBD method - alpha_tilde = alpha / dt
        // if using 2nd order XPBD method - alpha_tilde = alpha / dt^2
        Real alpha_tilde;
        if constexpr(IsFirstOrder)
        {
            alpha_tilde = _constraint->alpha() / _dt;
        }
        else
        {
            alpha_tilde = _constraint->alpha() / (_dt * _dt);
        }

        // calculate LHS of lambda update: delC^T * M^-1 * delC + alpha_tilde
        Real LHS = alpha_tilde;
        const std::vector<PositionReference>& positions = _constraint->positions();
        
        for (int i = 0; i < Constraint::NUM_POSITIONS; i++)
        {
            LHS += positions[i].inv_mass * (_delC[3*i]*_delC[3*i] + _delC[3*i+1]*_delC[3*i+1] + _delC[3*i+2]*_delC[3*i+2]);
        }

        // compute RHS of lambda update: -C - alpha_tilde*lambda
        Real RHS = -C - alpha_tilde * _lambda;

        // compute lambda update
        Real dlam = RHS / LHS;
        _lambda += dlam;

        // compute position updates
        for (int i = 0; i < Constraint::NUM_POSITIONS; i++)
        {
            Real update_x = positions[i].inv_mass * _delC[3*i] * dlam;
            Real update_y = positions[i].inv_mass * _delC[3*i+1] * dlam;
            Real update_z = positions[i].inv_mass * _delC[3*i+2] * dlam;
            
            coordinate_updates_ptr[3*i].ptr = positions[i].positionPtr();
            coordinate_updates_ptr[3*i].update = update_x;
            coordinate_updates_ptr[3*i+1].ptr = positions[i].positionPtr()+1;
            coordinate_updates_ptr[3*i+1].update = update_y;
            coordinate_updates_ptr[3*i+2].ptr = positions[i].positionPtr()+2;
            coordinate_updates_ptr[3*i+2].update = update_z;
        }
    }

    #ifdef HAVE_CUDA
    typedef GPUConstraintProjector<IsFirstOrder, typename Constraint::GPUConstraintType> GPUConstraintProjectorType;
    GPUConstraintProjectorType createGPUConstraintProjector() const
    {
        typename Constraint::GPUConstraintType gpu_constraint = _constraint->createGPUConstraint();
        return GPUConstraintProjectorType(std::move(gpu_constraint), _dt);
    }
    #endif

    private:
    Real _dt;       // time step of the simulation
    Real _lambda;   // Lagrange multiplier for the projection
    Real _delC[Constraint::NUM_COORDINATES];
    ConstraintReference<Constraint> _constraint;    // a consistent reference to the constraint being projected
    bool _valid;    // whether or not this projector is valid (if invalid, projection will not occur)
    

};


} // Solver

#endif // __CONSTRAINT_PROJECTOR_HPP