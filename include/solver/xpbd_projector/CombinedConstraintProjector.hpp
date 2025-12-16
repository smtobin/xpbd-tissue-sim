#ifndef __COMBINED_CONSTRAINT_PROJECTOR_HPP
#define __COMBINED_CONSTRAINT_PROJECTOR_HPP

#include "solver/constraint/PositionReference.hpp"
#include "solver/constraint/ConstraintReference.hpp"
#include "solver/xpbd_solver/XPBDSolverUpdates.hpp"

#include "common/TypeList.hpp"

#include <iostream>
#include <variant>

#ifdef HAVE_CUDA
#include "gpu/projector/GPUCombinedConstraintProjector.cuh"
#endif

namespace Solver
{

/** Performs a XPBD constraint projection for a two constraints simulatenously.
 * 
 * IMPORTANT NOTE AND TODO: Right now we assume that Constraint1 and Constraint2 share the exact same coordinates. 
 *   In general, this may not be the case and we should check for this.
 *   However, right now the only constraints being projected simultaneously are the Hydrostatic and Deviatoric constraints for the same tetrahedral element, which do in fact share the same coordinates.
 * 
 * IsFirstOrder template parameter indicates if the projection follows the 1st-Order XPBD algorithm
 * 
 * Constraint1 template parameter is the type of constraint being projected (HydrostaticConstraint, DeviatoricConstraint, etc.)
 * Constraint2 template parameter is the other type of constraint being projected
*/
template<bool IsFirstOrder, class Constraint1, class Constraint2>
class CombinedConstraintProjector
{
    public:
    /** Number of constraints projected */
    constexpr static int NUM_CONSTRAINTS = 2;
    /** Number of rigid bodies involved in the constraint projection. This will be 0 because rigid bodies are handled specially
     * with a different type of constraint projector.
     */
    constexpr static int NUM_RIGID_BODIES = 0;
    /** Maximum number of coordinates involved in the constraint projection (may actually be less due to sharing of coordinates between constraints) */
    constexpr static int MAX_NUM_COORDINATES = Constraint1::NUM_COORDINATES + Constraint2::NUM_COORDINATES;
    
    /** List of constraint types being projected */
    using constraint_type_list = TypeList<Constraint1, Constraint2>;
    /** Whether or not the 1st-Order algorithm is being used.*/
    constexpr static bool is_first_order = IsFirstOrder;

    public:
    /** Constructor */
    explicit CombinedConstraintProjector(Real dt, ConstraintReference<Constraint1>&& constraint1, ConstraintReference<Constraint2>&& constraint2)
        : _dt(dt), _constraint1(constraint1), _constraint2(constraint2), _valid(true)
    {
    }

    /** Special constructor enabled for 1st-Order constraint projectors.
     * Accepts an additional argument, the B^-1 for the coordinates involved in this constraint projection.
     */
    template<bool B = IsFirstOrder, std::enable_if_t<B, int> = 0>
    CombinedConstraintProjector(Real dt, ConstraintReference<Constraint1>&& constraint1, ConstraintReference<Constraint2>&& constraint2,
        const Eigen::Matrix<Real, Constraint1::NUM_COORDINATES, Constraint1::NUM_COORDINATES>& B_e_inv)
        : _dt(dt), _constraint1(constraint1), _constraint2(constraint2), _valid(true), _B_e_inv(B_e_inv)
    {
    }

    /** Default constructor - projector marked invalid */
    explicit CombinedConstraintProjector()
        : _valid(false)
    {
    }

    /** Sets the validity of the ConstraintProjector. When valid, it is assumed the Constraint exists to be projected.
     * @param valid : the new validity of the ConstraintProjector
     */
    void setValidity(bool valid) { _valid = valid; }

    /** @returns whether or not the ConstraintProjector is valid */
    bool isValid() const { return _valid; }

    int numCoordinates() { return Constraint1::NUM_COORDINATES; } // TODO: for now we're assuming that constraint1 and constraint2 share the same coords exactly

    /** @returns the positions (as PositionReferences) affect by the constraint projection. This will just be the 
     * positions of the single projected constraint.
     * 
     * TODO: assuming constraint1 and constraint2 share the same positions in the same order
    */
    const std::vector<PositionReference>& positions() const { return _constraint1->positions(); }

    const ConstraintReference<Constraint1>& constraint1() const { return _constraint1; }
    const ConstraintReference<Constraint2>& constraint2() const { return _constraint2; }

    Vec2r lambda() const { return Vec2r(_lambda[0], _lambda[1]); }

    Real dt() const { return _dt; }

    /** The constraint forces on each of the affected positions caused by this constraint.
     * @returns the constraint forces on each of the affected positions of this constraint. The returned vector is ordered such that
     * the forces are applied to the corresponding position at the same index in the positions() vector.
     * 
     * This method must be called AFTER constraint projection has been performed (i.e. after lambda has been calculated). 
     */
    std::vector<Vec3r> constraintForces() const
    {
        /** TODO: FOR NOW assuming that both constraints share exactly the same coordinates, in exactly the same order. FIND A MORE GENERAL WAY */
        std::vector<Vec3r> forces( Constraint1::NUM_POSITIONS );
        Real delC1[Constraint1::NUM_COORDINATES];
        Real delC2[Constraint2::NUM_COORDINATES];
        _constraint1->gradient(delC1);
        _constraint2->gradient(delC2);
        for (int i = 0; i < Constraint1::NUM_POSITIONS; i++)
        {
            // if 1st-order, F = delC^T * lambda / dt
            // if 2nd-order, F = delC^T * lambda / (dt*dt)
            if constexpr (IsFirstOrder)
                forces[i] = (Eigen::Map<Vec3r>(delC1 + 3*i) * _lambda[0] + Eigen::Map<Vec3r>(delC2 + 3*i) * _lambda[1])  / _dt;
            else
                forces[i] = (Eigen::Map<Vec3r>(delC1 + 3*i) * _lambda[0] + Eigen::Map<Vec3r>(delC2 + 3*i) * _lambda[1]) / (_dt*_dt);
        }

        return forces;
    }

    /** Initialize the Lagrange multipliers. */
    void initialize()
    {
        _lambda[0] = 0;
        _lambda[1] = 0;
    }

    /** Perform the XPBD constraint projection
     * @param coordinate_updates_ptr (OUTPUT) - a (currently empty) array of CoordinateUpdate structs of numCoordinates() size. Stores the resulting state updates from the XPBD projection. 
     */
    void project(CoordinateUpdate* coordinate_updates_ptr)
    {
        Real C[2];
        Real delC[Constraint1::NUM_COORDINATES + Constraint2::NUM_COORDINATES];
        _constraint1->evaluateWithGradient(C, delC);
        _constraint2->evaluateWithGradient(C+1, delC+Constraint1::NUM_COORDINATES);

        // std::cout << "Cd_grad: " << delC[0]<<", "<<delC[1]<<", "<<delC[2]<<", "<<delC[3]<<", "<<delC[4]<<", "<<delC[5]<<", "<<delC[6]<<", "<<delC[7]<<", "<<delC[8]<<", "<<delC[9]<<", "<<delC[10]<<", "<<delC[11]<<std::endl;
        // std::cout << "Ch_grad: " << delC[12]<<", "<<delC[13]<<", "<<delC[14]<<", "<<delC[15]<<", "<<delC[16]<<", "<<delC[17]<<", "<<delC[18]<<", "<<delC[19]<<", "<<delC[20]<<", "<<delC[21]<<", "<<delC[22]<<", "<<delC[23]<<std::endl;
        
        // compute alpha_tilde
        // if using 1st order XPBD method - alpha_tilde = alpha / dt
        // if using 2nd order XPBD method - alpha_tilde = alpha / dt^2
        Real alpha_tilde[2];
        if constexpr(IsFirstOrder)
        {
            alpha_tilde[0] = _constraint1->alpha() / _dt;
            alpha_tilde[1] = _constraint2->alpha() / _dt;
        }
        else
        {
            alpha_tilde[0] = _constraint1->alpha() / (_dt * _dt);
            alpha_tilde[1] = _constraint2->alpha() / (_dt * _dt);
        }

        // calculate LHS of lambda update: delC^T * M^-1 * delC
        Real LHS[4];
        for (int ci = 0; ci < 2; ci++)
        {
            for (int cj = 0; cj < 2; cj++)
            {
                if (ci == cj) LHS[cj*2 + ci] = alpha_tilde[ci];
                else LHS[cj*2 + ci] = 0;
            }
        }

        // TODO: FOR NOW assuming that both constraints share exactly the same coordinates, in exactly the same order. FIND A MORE GENERAL WAY
        for (int ci = 0; ci < 2; ci++)
        {
            float* delC_i = delC + ci*Constraint1::NUM_COORDINATES;
            for (int cj = ci; cj < 2; cj++)
            {
                float* delC_j = delC + cj*Constraint1::NUM_COORDINATES;

                for (int i = 0; i < Constraint1::NUM_POSITIONS; i++)
                {
                    LHS[cj*2 + ci] += _constraint1->positions()[i].inv_mass * (delC_i[3*i]*delC_j[3*i] + delC_i[3*i+1]*delC_j[3*i+1] + delC_i[3*i+2]*delC_j[3*i+2]);
                }

                LHS[ci*2 + cj] = LHS[cj*2 + ci];
            }
            
        }
        // compute RHS of lambda update: -C - alpha_tilde * lambda
        float RHS[2];
        for (int ci = 0; ci < 2; ci++)
        {
            RHS[ci] = -C[ci] - alpha_tilde[ci] * _lambda[ci];
        }

        // compute lambda update - solve 2x2 system
        float dlam[2];
        const float det = LHS[0]*LHS[3] - LHS[1]*LHS[2];

        dlam[0] = (RHS[0]*LHS[3] - RHS[1]*LHS[2]) / det;
        dlam[1] = (RHS[1]*LHS[0] - RHS[0]*LHS[1]) / det;

        // std::cout << "dlam[0]: " << dlam[0] << ", dlam[1]: " << dlam[1] << std::endl;

        // update lambdas
        _lambda[0] += dlam[0];
        _lambda[1] += dlam[1];

        // compute position updates
        // TODO: FOR NOW ASSUMING THAT BOTH CONSTRAINTS SHARE EXACTLY THE THE SAME COORDINATES, IN THE SAME ORDER
        Real* delC_c2 = delC + Constraint2::NUM_COORDINATES;
        for (int i = 0; i < Constraint1::NUM_POSITIONS; i++)
        {
            Real update_x = _constraint1->positions()[i].inv_mass * (delC[3*i] * dlam[0] + delC_c2[3*i] * dlam[1]);
            Real update_y = _constraint1->positions()[i].inv_mass * (delC[3*i+1] * dlam[0] + delC_c2[3*i+1] * dlam[1]);
            Real update_z = _constraint1->positions()[i].inv_mass * (delC[3*i+2] * dlam[0] + delC_c2[3*i+2] * dlam[1]);
            
            coordinate_updates_ptr[3*i].ptr = _constraint1->positions()[i].positionPtr();
            coordinate_updates_ptr[3*i].update = update_x;
            coordinate_updates_ptr[3*i+1].ptr = _constraint1->positions()[i].positionPtr()+1;
            coordinate_updates_ptr[3*i+1].update = update_y;
            coordinate_updates_ptr[3*i+2].ptr = _constraint1->positions()[i].positionPtr()+2;
            coordinate_updates_ptr[3*i+2].update = update_z;
        }
    }

    #ifdef HAVE_CUDA
    typedef GPUCombinedConstraintProjector<IsFirstOrder, typename Constraint1::GPUConstraintType, typename Constraint2::GPUConstraintType> GPUConstraintProjectorType;
    GPUConstraintProjectorType createGPUConstraintProjector() const
    {
        typename Constraint1::GPUConstraintType gpu_constraint1 = _constraint1->createGPUConstraint();
        typename Constraint2::GPUConstraintType gpu_constraint2 = _constraint2->createGPUConstraint();
        return GPUConstraintProjectorType(std::move(gpu_constraint1), std::move(gpu_constraint2), _dt);
    }
    #endif

    private:
    Real _dt;
    Real _lambda[2];
    ConstraintReference<Constraint1> _constraint1;
    ConstraintReference<Constraint2> _constraint2;
    bool _valid;

    /** The dense B^-1 matrix associated with the coordinates projected by this projector */
    typename std::conditional<IsFirstOrder, Eigen::Matrix<Real, Constraint1::NUM_COORDINATES, Constraint1::NUM_COORDINATES>, std::monostate>::type _B_e_inv;
};


} // namespace Solver

#endif // __COMBINED_CONSTRAINT_PROJECTOR_HPP