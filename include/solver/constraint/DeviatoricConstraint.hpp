#ifndef __DEVIATORIC_CONSTRAINT_HPP
#define __DEVIATORIC_CONSTRAINT_HPP

#include "solver/constraint/ElementConstraint.hpp"
#include "simobject/ElasticMaterial.hpp"

#ifdef HAVE_CUDA
#include "gpu/constraint/GPUDeviatoricConstraint.cuh"
#endif

namespace Solver
{

// TODO: systematic way for forward declarations...
template<bool IsFirstOrder, typename Constraint1, typename Constraint2>
class CombinedConstraintProjector;

class HydrostaticConstraint;

/** Represents the deviatoric constraint derived from the Stable Neo-hookean strain energy, proposed by Macklin et. al: https://mmacklin.com/neohookean.pdf
 */
class DeviatoricConstraint : public ElementConstraint
{   
    friend class CombinedConstraintProjector<true, DeviatoricConstraint, HydrostaticConstraint>;
    friend class CombinedConstraintProjector<false, DeviatoricConstraint, HydrostaticConstraint>;

    public:
    constexpr static int NUM_POSITIONS = 4;
    constexpr static int NUM_COORDINATES = 12;

    public:
    DeviatoricConstraint();

    /** Creates the deviatoric constraint from a MeshObject and the 4 vertices that make up the tetrahedral element. */
    DeviatoricConstraint( int v1, PositionReference::VecType* vec_ptr1, Real m1,
                          int v2, PositionReference::VecType* vec_ptr2, Real m2,
                          int v3, PositionReference::VecType* vec_ptr3, Real m3,
                          int v4, PositionReference::VecType* vec_ptr4, Real m4,
                          const ElasticMaterial& material);

    DeviatoricConstraint( int v1, PositionReference::VecType* vec_ptr1, Real m1,
                          int v2, PositionReference::VecType* vec_ptr2, Real m2,
                          int v3, PositionReference::VecType* vec_ptr3, Real m3,
                          int v4, PositionReference::VecType* vec_ptr4, Real m4,
                          const ElasticMaterial& material, const Mat3r& Q, Real rest_volume);

    int numPositions() const override { return NUM_POSITIONS; }
    int numCoordinates() const override { return NUM_COORDINATES; }
    bool isInequality() const override { return false; }


    /** Evaluates the current value of this constraint with pre-allocated memory.
     * i.e. returns C(x)
     * 
     * @param C (OUTPUT) - the pointer to the (currently empty) value of the constraint
     */
    void evaluate(Real* C) const;

    /** Computes the gradient of this constraint in vector form with pre-allocated memory.
     * i.e. returns delC(x)
     * 
     * @param grad (OUTPUT) - the pointer to the (currently empty) constraint gradient vector. Expects it to be _gradient_vector_size x 1.
     */
    void gradient(Real* grad) const;

    /** Computes the value and gradient of this constraint with pre-allocated memory.
     * i.e. returns C(x) and delC(x) together.
     * 
     * This may be desirable when there would be duplicate work involved to evaluate constraint and its gradient separately.
     * 
     * Inline for performance reasons.
     * 
     * @param C (OUTPUT) - the pointer to the (currently empty) value of the constraint
     * @param grad (OUTPUT) - the pointer to the (currently empty) constraint gradient vector. Expects it to be _gradient_vector_size x 1.
     */
    void evaluateWithGradient(Real* C, Real* grad) const
    {
        Real F[9];
        Real X[9];
        _computeF(F, X);
        _evaluate(C, F);
        _gradient(grad, C, F);
    }

    Vec3r elasticForce(int index) const
    {
        Real C;
        Real delC[12];
        evaluateWithGradient(&C, delC);

        // compute elastic force for index
        int pos_index = -1;
        for (unsigned i = 0; i < _positions.size(); i++)
        {
            if (index == _positions[i].index)
            {
                pos_index = i;
                break;
            }
        }
        if (pos_index == -1)
            assert(0 && "This constraint does not affect passed position!");
        
        const Vec3r grad = Eigen::Map<Vec3r>(delC + 3*pos_index);

        return -grad * (1.0/_alpha) * C;
    }

    #ifdef HAVE_CUDA
    typedef GPUDeviatoricConstraint GPUConstraintType;
    GPUConstraintType createGPUConstraint() const;
    #endif

    private:
    /** Note - these are inline for performance reasons. (~7% speedup) */

    /** Helper method to evaluate the constraint given the deformation gradient, F, using pre-allocated memory.
     * Avoids the need to recompute F if we already have it.
     */
    void _evaluate(Real* C, Real* F) const
    {
        // C = frob(F)
        *C = std::sqrt(F[0]*F[0] + F[1]*F[1] + F[2]*F[2] + F[3]*F[3] + F[4]*F[4] + F[5]*F[5] + F[6]*F[6] + F[7]*F[7] + F[8]*F[8]);
    }

    /** Helper method to evaluate the constraint gradient given the deformation gradient, F, useing pre-allocated memory.
     * Avoids the need to recompute F and C(x) if we already have it.
     */
    void _gradient(Real* delC, Real* C, Real* F) const
    {
        // for A = 1/C * F * Q^T,
        // 1st column of A is delC wrt 1st position
        // 2nd column of A is delC wrt 2nd position
        // 3rd column of A is delC wrt 3rd position
        // delC wrt 4th position is (-1st column - 2nd column - 3rd column)
        // see supplementary material of Macklin paper for more details

        Real inv_C = 1.0/(*C);

        // F is column major
        // calculation of delC wrt 1st position
        delC[0] = inv_C * (F[0]*_Q(0,0) + F[3]*_Q(0,1) + F[6]*_Q(0,2));
        delC[1] = inv_C * (F[1]*_Q(0,0) + F[4]*_Q(0,1) + F[7]*_Q(0,2));
        delC[2] = inv_C * (F[2]*_Q(0,0) + F[5]*_Q(0,1) + F[8]*_Q(0,2));

        // calculation of delC wrt 2nd position
        delC[3] = inv_C * (F[0]*_Q(1,0) + F[3]*_Q(1,1) + F[6]*_Q(1,2));
        delC[4] = inv_C * (F[1]*_Q(1,0) + F[4]*_Q(1,1) + F[7]*_Q(1,2));
        delC[5] = inv_C * (F[2]*_Q(1,0) + F[5]*_Q(1,1) + F[8]*_Q(1,2));

        // calculation of delC wrt 3rd position
        delC[6] = inv_C * (F[0]*_Q(2,0) + F[3]*_Q(2,1) + F[6]*_Q(2,2));
        delC[7] = inv_C * (F[1]*_Q(2,0) + F[4]*_Q(2,1) + F[7]*_Q(2,2));
        delC[8] = inv_C * (F[2]*_Q(2,0) + F[5]*_Q(2,1) + F[8]*_Q(2,2));

        // calculation of delC wrt 4th position
        delC[9]  = -delC[0] - delC[3] - delC[6];
        delC[10] = -delC[1] - delC[4] - delC[7];
        delC[11] = -delC[2] - delC[5] - delC[8];
    }

};

} // namespace Solver

#endif // __DEVIATORIC_CONSTRAINT_HPP