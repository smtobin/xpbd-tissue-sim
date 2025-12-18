#include "solver/constraint/MidpointConstraint.hpp"

namespace Solver
{

MidpointConstraint::MidpointConstraint( int v1, PositionReference::VecType* vec_ptr1, Real m1,
                                        int v2, PositionReference::VecType* vec_ptr2, Real m2,
                                        int v3, PositionReference::VecType* vec_ptr3, Real m3)
    : Constraint(std::vector<PositionReference>({
        PositionReference(v1, vec_ptr1, m1),
        PositionReference(v2, vec_ptr2, m2),
        PositionReference(v3, vec_ptr3, m3)
    }))
{
    
}

} // namespace Solver