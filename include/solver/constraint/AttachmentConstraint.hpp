#ifndef __ATTACHMENT_CONSTRAINT_HPP
#define __ATTACHMENT_CONSTRAINT_HPP

#include "solver/constraint/Constraint.hpp"

#include <iostream>

namespace Solver
{

class AttachmentConstraint : public Constraint
{
    public:
    constexpr static int NUM_POSITIONS = 1;
    constexpr static int NUM_COORDINATES = 4;

    AttachmentConstraint() = default;
    explicit AttachmentConstraint(int v_ind, PositionReference::VecType* vec_ptr, Real m, const Vec3r* attached_pos_ptr);

    explicit AttachmentConstraint(int v_ind, PositionReference::VecType* vec_ptr, Real m, int attached_ind, const std::vector<Vec3r>* attached_vec_ptr);

    explicit AttachmentConstraint(int v_ind, PositionReference::VecType* vec_ptr, Real m, const Vec3r& attached_pos);

    virtual void serialize(std::vector<std::byte>& buf) const override
    {
        Constraint::serialize(buf);
        pack(buf, _attached_pos);
        pack(buf, _attached_pos_ptr);
        pack(buf, _attached_vec_ptr);
        pack(buf, _attached_pos_ind);
    }
    
    virtual void deserialize(const std::byte*& buf) override
    {
        Constraint::deserialize(buf);
        unpack(buf, _attached_pos);
        unpack(buf, _attached_pos_ptr);
        unpack(buf, _attached_vec_ptr);
        unpack(buf, _attached_pos_ind);
    }

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
        *C = ( _positions[0].position() - _attachmentPosition() ).norm();
    }

    /** Computes the gradient of this constraint in vector form with pre-allocated memory.
     * i.e. returns delC(x)
     * 
     * @param grad (OUTPUT) - the pointer to the (currently empty) constraint gradient vector. Expects it to be _gradient_vector_size x 1.
     */
    inline void gradient(Real* grad) const override
    {
        Vec3r attach_pt = _attachmentPosition();
        const Real dist = ( _positions[0].position() - attach_pt ).norm();
        if (dist < Real(1e-12))
        {
            grad[0] = 1;
            grad[1] = 0;
            grad[2] = 0;
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
        Vec3r attach_pt = _attachmentPosition();
        const Real dist = ( _positions[0].position() - attach_pt ).norm();
        *C = dist;

        if (dist < Real(1e-12))
        {
            grad[0] = 1;
            grad[1] = 0;
            grad[2] = 0;
        }
        else
        {
            grad[0] = (_positions[0].position()[0] - attach_pt[0]) / dist;
            grad[1] = (_positions[0].position()[1] - attach_pt[1]) / dist;
            grad[2] = (_positions[0].position()[2] - attach_pt[2]) / dist;
        }
        
    }

    #ifdef HAVE_CUDA
    typedef GPUOffsetAttachmentConstraint GPUConstraintType;
    GPUConstraintType createGPUConstraint() const;
    #endif

    private:
    /** Helper for getting the attached position given what constructor was used */
    Vec3r _attachmentPosition() const
    {
        if (_attached_pos_ptr)
            return *_attached_pos_ptr;
        else if (_attached_vec_ptr)
            return _attached_vec_ptr->at(_attached_pos_ind);
        else
            return _attached_pos;
    }

    /** Option 1: static attached position */
    Vec3r _attached_pos;

    /** Option 2: pointer to attached position */
    const Vec3r* _attached_pos_ptr;

    /** Option 3: point to vector + index of attached position */
    const std::vector<Vec3r>* _attached_vec_ptr;
    int _attached_pos_ind;

};

} // namespace Solver

#endif // __ATTACHMENT_CONSTRAINT_HPP