#include "solver/constraint/ElementOffsetAttachmentConstraint.hpp"

namespace Solver
{

ElementOffsetAttachmentConstraint::ElementOffsetAttachmentConstraint(
        int v1, Real m1,
        int v2, Real m2,
        int v3, Real m3,
        int v4, Real m4,
        PositionReference::VecType* vec_ptr,
        const Vec4r& bary_coords,
        const Vec3r* attached_pos_ptr, const Vec3r& attachment_offset)
: Constraint(std::vector<PositionReference>({
    PositionReference(v1, vec_ptr, m1),
    PositionReference(v2, vec_ptr, m2),
    PositionReference(v3, vec_ptr, m3),
    PositionReference(v4, vec_ptr, m4)
    })),
    _bary_coords(bary_coords),
     _attached_pos_ptr(attached_pos_ptr), _attachment_offset(attachment_offset)
{

}

} // namespace Solver