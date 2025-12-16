#ifndef __POSITION_REFERENCE_HPP
#define __POSITION_REFERENCE_HPP

#include "common/types.hpp"
#include "common/TombstoneVector.hpp"

namespace Solver
{

/** Struct for storing references to positions in MeshObjects.
 * Used by Constraints to access node properties dynamically.
 * 
 * Direct pointers to vertex data are used to skip dereferencing multiple pointers, which is not only faster but more cache-friendly.
 */
struct PositionReference
{
    using VecType = TombstoneVector<Vec3r>;

    int index;             // the index of this position in the array of vertices
    VecType* vec_ptr;       // a direct pointer to the position - points to a data block owned by obj's vertices matrix
    Real inv_mass;            // store the inverse mass for quick lookup
    // Real num_constraints;     // number of constraints affect this position - stored here for quick lookup

    /** Default constructor */

    PositionReference()
        : index(0), vec_ptr(0), inv_mass(0)
    {}

    /** Constructor that initializes quantities from just an object pointer and index. */
    PositionReference(int index_, VecType* vec_ptr_, Real mass_)
        : index(index_), vec_ptr(vec_ptr_), inv_mass(1.0/mass_)
    {
    }

    Vec3r& position() const { return (*vec_ptr)[index]; }
    // const Vec3r& position() const { return (*vec_ptr)[index]; }

    Real* positionPtr() const { return (*vec_ptr)[index].data(); }
    // const Real* positionPtr() const { return (*vec_ptr)[index].data(); }

    /** Two PositionReferences are equal if they point to the same position in memory. */
    friend bool operator== (const PositionReference& lhs, const PositionReference& rhs)
    {
        return lhs.vec_ptr == rhs.vec_ptr && lhs.index == rhs.index;
    }
};

} // namespace Solver

#endif // __POSITION_REFERENCE_HPP