#ifndef __RIGID_BODY_CONSTRAINT_PROJECTOR_HPP
#define __RIGID_BODY_CONSTRAINT_PROJECTOR_HPP

#include "solver/constraint/PositionReference.hpp"
#include "solver/constraint/ConstraintReference.hpp"
#include "solver/xpbd_solver/XPBDSolverUpdates.hpp"
#include "solver/constraint/RigidBodyConstraint.hpp"

#include <type_traits>

#ifdef HAVE_CUDA
#include "gpu/projector/GPUConstraintProjector.cuh"
#endif

namespace Solver
{

// TODO: 1st order XPBD and Rigid Bodies just doesn't really make sense - can we always have this be false?
template<bool IsFirstOrder, class RBConstraint>
class RigidBodyConstraintProjector
{
    public:
    constexpr static int NUM_CONSTRAINTS = 1;
    constexpr static int NUM_RIGID_BODIES = RBConstraint::NUM_RIGID_BODIES;
    constexpr static int MAX_NUM_COORDINATES = RBConstraint::NUM_COORDINATES;

    /** List of constraint types being projected (will be a single constraint for this projector) */
    using constraint_type_list = TypeList<RBConstraint>;
    /** Whether or not the 1st-Order algorithm is used */
    constexpr static bool is_first_order = IsFirstOrder;

    public:
    explicit RigidBodyConstraintProjector(Real dt, ConstraintReference<RBConstraint>&& constraint_ref)
        : _dt(dt), _constraint(constraint_ref), _valid(true)
    {
    }

    /** Default constructor - projector marked invalid */
    explicit RigidBodyConstraintProjector()
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

    const ConstraintReference<RBConstraint>& constraint() const { return _constraint; }

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
        std::vector<Vec3r> forces( RBConstraint::NUM_POSITIONS );
        Real delC[RBConstraint::NUM_COORDINATES];
        _constraint->gradient(delC);
        for (int i = 0; i < RBConstraint::NUM_POSITIONS; i++)
        {
            // if 1st-order, F = delC^T * lambda / dt
            // if 2nd-order, F = delC^T * lambda / (dt*dt)
            if constexpr (IsFirstOrder)
                forces[i] = Eigen::Map<Vec3r>(delC + 3*i) * _lambda / _dt;
            else
                forces[i] = Eigen::Map<Vec3r>(delC + 3*i) * _lambda / (_dt*_dt);
        }

        return forces;
    }

    void initialize()
    {
        _lambda = 0;
    }

    void project(CoordinateUpdate* coordinate_updates_ptr, RigidBodyUpdate* rigid_body_updates_ptr)
    {
        Real C;
        Real delC[RBConstraint::NUM_COORDINATES];
        _constraint->evaluateWithGradient(&C, delC);

        // if inequality constraint, make sure that the constraint should actually be enforce
        if (C > 0 && _constraint->isInequality())
        {
            for (int i = 0; i < RBConstraint::NUM_COORDINATES; i++) 
            { 
                coordinate_updates_ptr[i].ptr = nullptr;
            }
            for (int i = 0; i < RBConstraint::NUM_RIGID_BODIES; i++)
            {
                rigid_body_updates_ptr[i].obj_ptr = nullptr;
            }
            return;
        }

        // calculate LHS of lambda update: delC^T * M^-1 * delC
        Real alpha_tilde = _constraint->alpha() / (_dt * _dt);
        Real LHS = alpha_tilde;
        const std::vector<PositionReference>& positions = _constraint->positions();
        
        for (int i = 0; i < RBConstraint::NUM_POSITIONS; i++)
        {
            LHS += positions[i].inv_mass * (delC[3*i]*delC[3*i] + delC[3*i+1]*delC[3*i+1] + delC[3*i+2]*delC[3*i+2]);
        }

        // compute RHS of lambda update: -C - alpha_tilde*lambda
        Real RHS = -C - alpha_tilde * _lambda;

        // compute lambda update
        Real dlam = RHS / LHS;
        _lambda += dlam;

        // compute position updates
        for (int i = 0; i < RBConstraint::NUM_POSITIONS; i++)
        {
            Real update_x = positions[i].inv_mass * delC[3*i] * dlam;
            Real update_y = positions[i].inv_mass * delC[3*i+1] * dlam;
            Real update_z = positions[i].inv_mass * delC[3*i+2] * dlam;
            
            coordinate_updates_ptr[3*i].ptr = positions[i].positionPtr();
            coordinate_updates_ptr[3*i].update = update_x;
            coordinate_updates_ptr[3*i+1].ptr = positions[i].positionPtr()+1;
            coordinate_updates_ptr[3*i+1].update = update_y;
            coordinate_updates_ptr[3*i+2].ptr = positions[i].positionPtr()+2;
            coordinate_updates_ptr[3*i+2].update = update_z;
        }

        // compute the update for each rigid body using the corresponding RigidBodyXPBDHelper object
        for (int ri = 0; ri < _constraint->numRigidBodies(); ri++)
        {
            rigid_body_updates_ptr[ri].obj_ptr = _constraint->rigidBodies()[ri];
            _constraint->rigidBodyHelpers()[ri]->update(dlam, rigid_body_updates_ptr[ri].position_update, rigid_body_updates_ptr[ri].orientation_update);
        }
    }

    // TODO: implement specific GPUConstraintProjector type for rigid body constraints - need RigidObjectGPUResource and stuff like that
    #ifdef HAVE_CUDA
    typedef GPUConstraintProjector<IsFirstOrder, typename RBConstraint::GPUConstraintType> GPUConstraintProjectorType;
    GPUConstraintProjectorType createGPUConstraintProjector() const
    {
        typename RBConstraint::GPUConstraintType gpu_constraint = _constraint->createGPUConstraint();
        return GPUConstraintProjectorType(std::move(gpu_constraint), _dt);
    }
    #endif

    private:
    Real _dt;
    Real _lambda;
    ConstraintReference<RBConstraint> _constraint;
    bool _valid;
};

} // namespace Solver

#endif // __RIGID_BODY_CONSTRAINT_PROJECTOR_HPP