#ifndef __COLLISION_CONSTRAINT_HPP
#define __COLLISION_CONSTRAINT_HPP

#include "solver/constraint/Constraint.hpp"

namespace Solver
{

/** Defines a generic collision constraint that separates two colliding bodies. 
 * The main reason this class exists is to define the applyFriction() method which will apply equal and opposite friction forces to the colliding bodies,
 * in a direction tangential to the collision normal.
*/
class CollisionConstraint : public Constraint
{
    public:
    CollisionConstraint() = default;
    CollisionConstraint(const std::vector<PositionReference>& positions, const Vec3r& collision_normal)
        : Constraint(positions, 0), _collision_normal(collision_normal)
    {}

    virtual void serialize(std::vector<std::byte>& buf) const override
    {
        Constraint::serialize(buf);
        pack(buf, _collision_normal);
    }
    virtual void deserialize(const std::byte*& buf) override
    {
        Constraint::deserialize(buf);
        unpack(buf, _collision_normal);
    }

    /** Applies a frictional force to the two colliding bodies given the coefficients of friction and the Lagrange multiplier from this constraint.
     * @param lam - the Lagrange multiplier for this constraint after the XPBD update
     * @param mu_s - the coefficient of static friction between the two bodies
     * @param mu_k - the coefficient of kinetic friction between the two bodies
     */
    inline virtual void applyFriction(Real lam, Real mu_s, Real mu_k) const = 0;

    protected:
    Vec3r _collision_normal;  // the normal of the collision plane - also usually taken as the minimum separating vector
};

} // namespace Solver

#endif // __COLLISION_CONSTRAINT_HPP