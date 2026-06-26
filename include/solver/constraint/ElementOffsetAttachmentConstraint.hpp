#ifndef __ELEMENT_OFFSET_ATTACHMENT_CONSTRAINT_HPP
#define __ELEMENT_OFFSET_ATTACHMENT_CONSTRAINT_HPP

#include "solver/constraint/Constraint.hpp"

#include <iostream>

namespace Solver
{

class ElementOffsetAttachmentConstraint : public Constraint
{
    public:
    constexpr static int NUM_POSITIONS = 4;
    constexpr static int NUM_COORDINATES = 12;

    ElementOffsetAttachmentConstraint() = default;
    explicit ElementOffsetAttachmentConstraint(
        int v1, Real m1,
        int v2, Real m2,
        int v3, Real m3,
        int v4, Real m4,
        PositionReference::VecType* vec_ptr,
        const Vec4r& bary_coords,
        const Vec3r* attached_pos_ptr, const Vec3r& attachment_offset);

    virtual void serialize(std::vector<std::byte>& buf) const override
    {
        Constraint::serialize(buf);
        pack(buf, _bary_coords);
        pack(buf, _attached_pos_ptr);
        pack(buf, _attachment_offset);
    }
    
    virtual void deserialize(const std::byte*& buf) override
    {
        Constraint::deserialize(buf);
        unpack(buf, _bary_coords);
        unpack(buf, _attached_pos_ptr);
        unpack(buf, _attachment_offset);
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
        *C = ( _elementPoint() - (*_attached_pos_ptr + _attachment_offset) ).norm();
    }

    /** Computes the gradient of this constraint in vector form with pre-allocated memory.
     * i.e. returns delC(x)
     * 
     * @param grad (OUTPUT) - the pointer to the (currently empty) constraint gradient vector. Expects it to be _gradient_vector_size x 1.
     */
    inline void gradient(Real* grad) const override
    {
        Vec3r attach_pt = (*_attached_pos_ptr + _attachment_offset);
        Vec3r element_pt = _elementPoint();
        const Real dist = ( element_pt - attach_pt ).norm();

        if (dist < Real(1e-12))
        {
            grad[0] = 1; grad[3] = 1; grad[6] = 1; grad[9] = 1; 
            grad[1] = 0; grad[4] = 0; grad[7] = 0; grad[10] = 0;
            grad[2] = 0; grad[5] = 0; grad[8] = 0; grad[11] = 0;
        }
        else
        {
            grad[0] = _bary_coords[0] * (element_pt[0] - attach_pt[0]) / dist;
            grad[1] = _bary_coords[0] * (element_pt[1] - attach_pt[1]) / dist;
            grad[2] = _bary_coords[0] * (element_pt[2] - attach_pt[2]) / dist;

            grad[3] = _bary_coords[1] * (element_pt[0] - attach_pt[0]) / dist;
            grad[4] = _bary_coords[1] * (element_pt[1] - attach_pt[1]) / dist;
            grad[5] = _bary_coords[1] * (element_pt[2] - attach_pt[2]) / dist;

            grad[6] = _bary_coords[2] * (element_pt[0] - attach_pt[0]) / dist;
            grad[7] = _bary_coords[2] * (element_pt[1] - attach_pt[1]) / dist;
            grad[8] = _bary_coords[2] * (element_pt[2] - attach_pt[2]) / dist;

            grad[9] = _bary_coords[3] * (element_pt[0] - attach_pt[0]) / dist;
            grad[10] = _bary_coords[3] * (element_pt[1] - attach_pt[1]) / dist;
            grad[11] = _bary_coords[3] * (element_pt[2] - attach_pt[2]) / dist;
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
        Vec3r attach_pt = (*_attached_pos_ptr + _attachment_offset);
        Vec3r element_pt = _elementPoint();
        const Real dist = ( element_pt - attach_pt ).norm();
        *C = dist;

        if (dist < Real(1e-12))
        {
            grad[0] = 1; grad[3] = 1; grad[6] = 1; grad[9] = 1; 
            grad[1] = 0; grad[4] = 0; grad[7] = 0; grad[10] = 0;
            grad[2] = 0; grad[5] = 0; grad[8] = 0; grad[11] = 0;
        }
        else
        {
            grad[0] = _bary_coords[0] * (element_pt[0] - attach_pt[0]) / dist;
            grad[1] = _bary_coords[0] * (element_pt[1] - attach_pt[1]) / dist;
            grad[2] = _bary_coords[0] * (element_pt[2] - attach_pt[2]) / dist;

            grad[3] = _bary_coords[1] * (element_pt[0] - attach_pt[0]) / dist;
            grad[4] = _bary_coords[1] * (element_pt[1] - attach_pt[1]) / dist;
            grad[5] = _bary_coords[1] * (element_pt[2] - attach_pt[2]) / dist;

            grad[6] = _bary_coords[2] * (element_pt[0] - attach_pt[0]) / dist;
            grad[7] = _bary_coords[2] * (element_pt[1] - attach_pt[1]) / dist;
            grad[8] = _bary_coords[2] * (element_pt[2] - attach_pt[2]) / dist;

            grad[9] = _bary_coords[3] * (element_pt[0] - attach_pt[0]) / dist;
            grad[10] = _bary_coords[3] * (element_pt[1] - attach_pt[1]) / dist;
            grad[11] = _bary_coords[3] * (element_pt[2] - attach_pt[2]) / dist;
        }
        
    }

    private:
    Vec3r _elementPoint() const
    {
        Vec3r pt = Vec3r::Zero();
        for (int i = 0; i < 4; i++)
        {
            pt += _positions[i].position() * _bary_coords[i];
        }

        return pt;
    }

    Vec4r _bary_coords;
    const Vec3r* _attached_pos_ptr;
    Vec3r _attachment_offset;

};

} // namespace Solver

#endif // __ATTACHMENT_CONSTRAINT_HPP