#ifndef __ATTACHMENT_CONSTRAINT_HPP
#define __ATTACHMENT_CONSTRAINT_HPP

#include "solver/constraint/Constraint.hpp"

#include <iostream>

#ifdef HAVE_CUDA
#include "gpu/constraint/GPUAttachmentConstraint.cuh"
#endif

namespace Solver
{

class AttachmentConstraint : public Constraint
{
    public:
    constexpr static int NUM_POSITIONS = 1;
    constexpr static int NUM_COORDINATES = 4;

    explicit AttachmentConstraint(int v_ind, PositionReference::VecType* vec_ptr, Real m, const Vec3r* attached_pos_ptr, const Vec3r& attachment_offset);

    int numPositions() const override { return NUM_POSITIONS; }
    int numCoordinates() const override { return NUM_COORDINATES; }
    bool isInequality() const override { return false; }


    /** Evaluates the current value of this constraint with pre-allocated memory.
     * i.e. returns C(x)
     * 
     * @param C (OUTPUT) - the pointer to the (currently empty) value of the constraint
     */
    inline void evaluate(Real* C) const override
    {
        *C = ( _positions[0].position() - (*_attached_pos_ptr + _attachment_offset) ).norm();
    }

    /** Computes the gradient of this constraint in vector form with pre-allocated memory.
     * i.e. returns delC(x)
     * 
     * @param grad (OUTPUT) - the pointer to the (currently empty) constraint gradient vector. Expects it to be _gradient_vector_size x 1.
     */
    inline void gradient(Real* grad) const override
    {
        const Vec3r& attach_pt = (*_attached_pos_ptr + _attachment_offset);
        const Real dist = ( _positions[0].position() - attach_pt ).norm();
        if (dist < Real(1e-12))
        {
            grad[0] = Real(1e-12);
            grad[1] = Real(1e-12);
            grad[2] = Real(1e-12);
        }
        else
        {
            grad[0] = (_positions[0].position()[0] - attach_pt[0]) / dist;
            grad[1] = (_positions[0].position()[1] - attach_pt[1]) / dist;
            grad[2] = (_positions[0].position()[2] - attach_pt[2]) / dist;
        }
    }


    /** Computes the value and gradient of this constraint with pre-allocated memory.
     * i.e. returns C(x) and delC(x) together.
     * 
     * This may be desirable when there would be duplicate work involved to evaluate constraint and its gradient separately.
     * 
     * @param C (OUTPUT) - the pointer to the (currently empty) value of the constraint
     * @param grad (OUTPUT) - the pointer to the (currently empty) constraint gradient vector. Expects it to be _gradient_vector_size x 1.
     */
    void evaluateWithGradient(Real* C, Real* grad) const override
    {
        const Vec3r& attach_pt = (*_attached_pos_ptr + _attachment_offset);
        const Real dist = ( _positions[0].position() - attach_pt ).norm();
        *C = dist;

        if (dist < Real(1e-12))
        {
            grad[0] = Real(1e-12);
            grad[1] = Real(1e-12);
            grad[2] = Real(1e-12);
        }
        else
        {
            grad[0] = (_positions[0].position()[0] - attach_pt[0]) / dist;
            grad[1] = (_positions[0].position()[1] - attach_pt[1]) / dist;
            grad[2] = (_positions[0].position()[2] - attach_pt[2]) / dist;
        }
        
    }

    #ifdef HAVE_CUDA
    typedef GPUAttachmentConstraint GPUConstraintType;
    GPUConstraintType createGPUConstraint() const;
    #endif

    private:
    const Vec3r* _attached_pos_ptr;
    const Vec3r _attachment_offset;

};

} // namespace Solver

#endif // __ATTACHMENT_CONSTRAINT_HPP