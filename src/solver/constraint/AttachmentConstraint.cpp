#include "solver/constraint/AttachmentConstraint.hpp"

namespace Solver
{

AttachmentConstraint::AttachmentConstraint(int v_ind, PositionReference::VecType* vec_ptr, Real m, const Vec3r& attached_pos)
: Constraint(std::vector<PositionReference>({
    PositionReference(v_ind, vec_ptr, m)
    })), _attached_pos(attached_pos), _attached_pos_ptr(nullptr), _attached_vec_ptr(nullptr), _attached_pos_ind(-1)
{

}

AttachmentConstraint::AttachmentConstraint(int v_ind, PositionReference::VecType* vec_ptr, Real m, const Vec3r* attached_pos_ptr)
: Constraint(std::vector<PositionReference>({
    PositionReference(v_ind, vec_ptr, m)
    })), _attached_pos(Vec3r::Zero()), _attached_pos_ptr(attached_pos_ptr), _attached_vec_ptr(nullptr), _attached_pos_ind(-1)
{

}

AttachmentConstraint::AttachmentConstraint(int v_ind, PositionReference::VecType* vec_ptr, Real m, int attached_ind, const std::vector<Vec3r>* attached_vec_ptr)
: Constraint(std::vector<PositionReference>({
    PositionReference(v_ind, vec_ptr, m)
    })), _attached_pos(Vec3r::Zero()), _attached_pos_ptr(nullptr), _attached_vec_ptr(attached_vec_ptr), _attached_pos_ind(attached_ind)
{

}

} // namespace Solver