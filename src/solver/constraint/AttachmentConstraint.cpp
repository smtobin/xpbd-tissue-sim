#include "solver/constraint/AttachmentConstraint.hpp"

namespace Solver
{

AttachmentConstraint::AttachmentConstraint(int v_ind, PositionReference::VecType* vec_ptr, Real m, const Vec3r* attached_pos_ptr, const Vec3r& attachment_offset)
: Constraint(std::vector<PositionReference>({
    PositionReference(v_ind, vec_ptr, m)
    })), _attached_pos_ptr(attached_pos_ptr), _attachment_offset(attachment_offset)
{

}

#ifdef HAVE_CUDA
AttachmentConstraint::GPUConstraintType AttachmentConstraint::createGPUConstraint() const
{
    // TODO: figure out way to have a pointer to the attached position!
    // this will just have a constant attached position
    GPUConstraintType gpu_constraint = GPUConstraintType(_positions[0].index, _positions[0].inv_mass,
        *_attached_pos_ptr, _attachment_offset, _alpha);
    
    return gpu_constraint;
}
#endif

} // namespace Solver