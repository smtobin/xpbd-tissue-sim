#include "solver/constraint/FaceOffsetAttachmentConstraint.hpp"

namespace Solver
{

FaceOffsetAttachmentConstraint::FaceOffsetAttachmentConstraint(
        int v1, Real m1,
        int v2, Real m2,
        int v3, Real m3,
        PositionReference::VecType* vec_ptr,
        const Vec3r& bary_coords,
        const Vec3r* attached_pos_ptr, const Vec3r& attachment_offset,
        Real undershoot_frac)
: Constraint(std::vector<PositionReference>({
    PositionReference(v1, vec_ptr, m1),
    PositionReference(v2, vec_ptr, m2),
    PositionReference(v3, vec_ptr, m3)
    })),
    _undershoot_frac(undershoot_frac),
    _bary_coords(bary_coords),
     _attached_pos_ptr(attached_pos_ptr), _attachment_offset(attachment_offset)
{

}

} // namespace Solver