#ifndef __MIDPOINT_CONSTRAINT_HPP
#define __MIDPOINT_CONSTRAINT_HPP

#include "solver/constraint/Constraint.hpp"

#include <iostream>

namespace Solver
{

class MidpointConstraint : public Constraint
{
    public:
    constexpr static int NUM_POSITIONS = 3; 
    constexpr static int NUM_COORDINATES = 9;
    
    public:
    MidpointConstraint();

    /** Constructor for the midpoint constraint.
     * v1 is constrained to be the midpoint between v2 and v3
     */
    MidpointConstraint( int v1, PositionReference::VecType* vec_ptr1, Real m1,
                        int v2, PositionReference::VecType* vec_ptr2, Real m2,
                        int v3, PositionReference::VecType* vec_ptr3, Real m3);

    int numPositions() const override { return NUM_POSITIONS; }
    int numCoordinates() const override { return NUM_COORDINATES; }
    bool isInequality() const override { return false; }


    /** Evaluates the current value of this constraint with pre-allocated memory.
     * i.e. returns C(x)
     * 
     * @param C (OUTPUT) - the pointer to the (currently empty) value of the constraint
     */
    void evaluate(Real* C) const override
    {
        const Vec3r diff = _positions[0].position() - 0.5*(_positions[1].position() + _positions[2].position());
        *C = diff.norm();
    }

    /** Computes the gradient of this constraint in vector form with pre-allocated memory.
     * i.e. returns delC(x)
     * 
     * @param grad (OUTPUT) - the pointer to the (currently empty) constraint gradient vector. Expects it to be _gradient_vector_size x 1.
     */
    void gradient(Real* grad) const override
    {
        const Vec3r diff = _positions[0].position() - 0.5*(_positions[1].position() + _positions[2].position());
        Real C = diff.norm();

        if (C < Real(1e-12))
        {
            grad[0] = 1; grad[2] = 0; grad[3] = 0;
            grad[3] = 0.5; grad[4] = 0; grad[5] = 0;
            grad[6] = 0.5; grad[7] = 0; grad[8] = 0;
        }
        else
        {
            Vec3r grad_v1 = diff / C;
            Vec3r grad_v2 = -0.5*grad_v1;
            grad[0] = grad_v1[0]; grad[1] = grad_v1[1]; grad[2] = grad_v1[2];   // gradient w.r.t. v1
            grad[3] = grad_v2[0]; grad[4] = grad_v2[1]; grad[5] = grad_v2[2];   // gradient w.r.t. v2
            grad[6] = grad_v2[0]; grad[7] = grad_v2[1]; grad[8] = grad_v2[2];   // gradient w.r.t. v3 (= grad w.r.t. v2)
        }
    }


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
    void evaluateWithGradient(Real* C, Real* grad) const override
    {
        const Vec3r diff = _positions[0].position() - 0.5*(_positions[1].position() + _positions[2].position());
        *C = diff.norm();

        if (*C < Real(1e-12))
        {
            grad[0] = 1; grad[2] = 0; grad[3] = 0;
            grad[3] = -0.5; grad[4] = 0; grad[5] = 0;
            grad[6] = -0.5; grad[7] = 0; grad[8] = 0;
        }
        else
        {
            Vec3r grad_v1 = diff / *C;
            Vec3r grad_v2 = -0.5*grad_v1;
            grad[0] = grad_v1[0]; grad[1] = grad_v1[1]; grad[2] = grad_v1[2];   // gradient w.r.t. v1
            grad[3] = grad_v2[0]; grad[4] = grad_v2[1]; grad[5] = grad_v2[2];   // gradient w.r.t. v2
            grad[6] = grad_v2[0]; grad[7] = grad_v2[1]; grad[8] = grad_v2[2];   // gradient w.r.t. v3 (= grad w.r.t. v2)
        }
    }
};
    
} // namespace Solver

#endif // __MIDPOINT_CONSTRAINT_HPP