#ifndef __CONSTRAINT_REFERENCE_HPP
#define __CONSTRAINT_REFERENCE_HPP

#include "common/TombstoneVector.hpp"

#include <iostream>

namespace Solver
{

/** A consistent reference to a Constraint.
 * Constraints are stored in vectors that can change size. If a vector has to allocate more memory, any pointers or references
 * to its contents are invalidated. This becomes a problem for constraints that are dynamically added and removed (like collision constraints).
 * 
 * By storing a pointer to the container and its index, we can ensure that even if the vector changes sizes, we still have a valid
 * reference to the Constraint.
 * 
 * There is a performance hit by accessing the constraint through the constraint vector rather than with direct pointer, and since accessing
 * the constraint for its current value and gradient is done thousands of times every time step, this becomes slightly significant (~4-5% slower).
 * 
 * But, for now I think the flexibility (and not having to worry about pre-allocating enough space in the constraint vectors) outweighs the slight cost.
 * 
 * TODO: Generalize ConstraintReference (there is nothing here that is specific to constraints).
 * TODO: Have a monitored vector class that can have "references" registered to it. When it changes size it notifies each reference to update (somehow).
 */
template <typename Constraint>
class ConstraintReference
{
public:
    using constraint_type = Constraint;
    using vector_type = TombstoneVector<constraint_type>;//std::vector<constraint_type>;
    
    ConstraintReference(vector_type& vec, int index)
        : _vec(&vec), _index(index)
    {
        // _ptr = &_vec.at(_index);
    }

    ConstraintReference()
        : _vec(0), _index(0)
    {

    }

    const constraint_type* operator->() const
    {
        // return _ptr;
        return &(*_vec).at(_index);
    }

    constraint_type* operator->()
    {
        // return _ptr;
        return &(*_vec).at(_index);
    }

    const constraint_type& get() const
    {
        return (*_vec).at(_index);
    }

    constraint_type& get()
    {
        return (*_vec).at(_index);
    }

    private:
    vector_type* _vec;
    int _index;
    // constraint_type* _ptr;
};

} // namespace Solver

#endif // __CONSTRAINT_REFERENCE_HPP