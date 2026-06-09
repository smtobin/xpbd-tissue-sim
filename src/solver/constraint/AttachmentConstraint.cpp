#include "solver/constraint/AttachmentConstraint.hpp"

namespace Solver
{

AttachmentConstraint::AttachmentConstraint(int v_ind, PositionReference::VecType* vec_ptr, Real m, const Vec3r* attached_pos_ptr)
: Constraint(std::vector<PositionReference>({
    PositionReference(v_ind, vec_ptr, m)
    })), _attached_pos_ptr(attached_pos_ptr)
{

}

} // namespace Solver