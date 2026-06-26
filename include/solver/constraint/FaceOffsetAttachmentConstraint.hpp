#ifndef __ELEMENT_OFFSET_ATTACHMENT_CONSTRAINT_HPP
#define __ELEMENT_OFFSET_ATTACHMENT_CONSTRAINT_HPP

#include "solver/constraint/Constraint.hpp"

#include <iostream>

namespace Solver
{

class FaceOffsetAttachmentConstraint : public Constraint
{
    public:
    constexpr static int NUM_POSITIONS = 3;
    constexpr static int NUM_COORDINATES = 9;

    FaceOffsetAttachmentConstraint() = default;
    explicit FaceOffsetAttachmentConstraint(
        int v1, Real m1,
        int v2, Real m2,
        int v3, Real m3,
        PositionReference::VecType* vec_ptr,
        const Vec3r& bary_coords,
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
        *C = ( _elementPoint() - (*_attached_pos_ptr + _attachment_offset) ).squaredNorm();
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

        grad[0] = 2*_bary_coords[0] * (element_pt[0] - attach_pt[0]);
        grad[1] = 2*_bary_coords[0] * (element_pt[1] - attach_pt[1]);
        grad[2] = 2*_bary_coords[0] * (element_pt[2] - attach_pt[2]);

        grad[3] = 2*_bary_coords[1] * (element_pt[0] - attach_pt[0]);
        grad[4] = 2*_bary_coords[1] * (element_pt[1] - attach_pt[1]);
        grad[5] = 2*_bary_coords[1] * (element_pt[2] - attach_pt[2]);

        grad[6] = 2*_bary_coords[2] * (element_pt[0] - attach_pt[0]);
        grad[7] = 2*_bary_coords[2] * (element_pt[1] - attach_pt[1]);
        grad[8] = 2*_bary_coords[2] * (element_pt[2] - attach_pt[2]);
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
        std::cout << "Attach pt: " << attach_pt.transpose() << std::endl;
        std::cout << "attach pos ptr: " << _attached_pos_ptr << std::endl;
        std::cout << "element pt: " << element_pt.transpose() << std::endl;
        const Real dist_sq = ( element_pt - attach_pt ).squaredNorm();
        *C = dist_sq;

        grad[0] = 2*_bary_coords[0] * (element_pt[0] - attach_pt[0]);
        grad[1] = 2*_bary_coords[0] * (element_pt[1] - attach_pt[1]);
        grad[2] = 2*_bary_coords[0] * (element_pt[2] - attach_pt[2]);

        grad[3] = 2*_bary_coords[1] * (element_pt[0] - attach_pt[0]);
        grad[4] = 2*_bary_coords[1] * (element_pt[1] - attach_pt[1]);
        grad[5] = 2*_bary_coords[1] * (element_pt[2] - attach_pt[2]);

        grad[6] = 2*_bary_coords[2] * (element_pt[0] - attach_pt[0]);
        grad[7] = 2*_bary_coords[2] * (element_pt[1] - attach_pt[1]);
        grad[8] = 2*_bary_coords[2] * (element_pt[2] - attach_pt[2]);

        
        std::cout << "C = " << *C << "  diff=" << (element_pt - attach_pt).transpose() << std::endl;

        if (std::isnan(*C))
            throw std::runtime_error("nan");
        
    }

    private:
    Vec3r _elementPoint() const
    {
        Vec3r pt = Vec3r::Zero();
        for (int i = 0; i < 3; i++)
        {
            std::cout << "  iter " << i << " position: " << _positions[i].position().transpose() << "  ind: " << _positions[i].index << "  bary " << _bary_coords[i] << std::endl;
            pt += _positions[i].position() * _bary_coords[i];
        }

        return pt;
    }

    Vec3r _bary_coords;
    const Vec3r* _attached_pos_ptr;
    Vec3r _attachment_offset;

};

} // namespace Solver

#endif // __ATTACHMENT_CONSTRAINT_HPP