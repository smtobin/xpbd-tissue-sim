#ifndef __RIGID_DEFORMABLE_COLLISION_CONSTRAINT_HPP
#define __RIGID_DEFORMABLE_COLLISION_CONSTRAINT_HPP

#include "solver/constraint/RigidBodyConstraint.hpp"
#include "solver/constraint/CollisionConstraint.hpp"
#include "geometry/SDF.hpp"

#ifdef HAVE_CUDA
#include "gpu/constraint/GPURigidDeformableCollisionConstraint.cuh"
#endif

namespace Solver
{

/** Defines a collision constraint between a rigid body and a deformable body.
 * The collision is defined to be between two points - one on the deformable body (p1) and one on the rigid body (p2) - these are obtained from collision detection.
 * (The point on the deformable body may be between vertices in the middle of an edge or a face, so we represent p1 in barycentric coordinates (u,v,w) )
 * The collision also has a collision normal (n), which is the direction of minimum separation - also obtained from collision detection.
 * We then define the constraint C(x) = (p1 - p2) * n, where * is the dot product. In other words, the constraint value is the penetration distance (negative if penetrating, positive if not).
 * 
 * If there is no penetration, then this constraint is not enforced (i.e it is an XPBD inequality constraint)
 * 
 * This collision constraint may be used for multiple frames, and thus the colliding bodies will evolve in time past the point of collision. It's possible that the original
 * collision parameters p1, p2, and n are no longer accurate after the object has moved, meaning that the expression (p1 - p2) * n may be negative even though the objects are no
 * longer penetrating. Instead, we can simply use the Signed Distance Function (SDF) that was used for collision detection to get the penetration distance, so that we
 * are completely certain when the objects are penetrating or not (and thus whether or not we should enforce this constraint).
 * 
 * As of right now, the points on the body that are penetrating (p1 and p2) and the collision normal (n) do not evolve in time and are taken to be fixed (despite the penetration
 * distance given by the SDF evolving in time). So far, this seems to be a reasonable assumption - i.e. if the bodies are still colliding after multiple frames, it is highly
 * likely that the penetrating points and collision normal are pretty similar to what they were when this collision constraint was created.
 */
class RigidDeformableCollisionConstraint : public CollisionConstraint, public RigidBodyConstraint
{
    public:
    constexpr static int NUM_POSITIONS = 3;
    constexpr static int NUM_COORDINATES = 9;
    constexpr static int NUM_RIGID_BODIES = 1;

    public:
    /** 
     * @param sdf : the SDF for the rigid object
     * @param rigid_obj : pointer to the rigid object
     * @param p : the point on the rigid object in global coordinates
     * @param n : the collision normal in global coordinates
     * @param deformable_obj : pointer to the deformable object
     * @param v1,v2,v3 : the vertex indices of the colliding face on the deformable object
     * @param u,v,w : the barycentric coordinates of the the colliding point on the colliding face of the deformable object
     */
    RigidDeformableCollisionConstraint(const Geometry::SDF* sdf, Sim::RigidObject* rigid_obj, const Vec3r& rigid_body_point, const Vec3r& collision_normal,
                                       int v1, PositionReference::VecType* vec_ptr1, Real m1,
                                       int v2, PositionReference::VecType* vec_ptr2, Real m2,
                                       int v3, PositionReference::VecType* vec_ptr3, Real m3,
                                        Real u, Real v, Real w);

    int numPositions() const override { return NUM_POSITIONS; }
    int numCoordinates() const override { return NUM_COORDINATES; }

    /** Evaluates the current value of this constraint with pre-allocated memory.
     * i.e. returns C(x)
     * 
     * @param C (OUTPUT) - the pointer to the (currently empty) value of the constraint
     */
    inline void evaluate(Real* C) const override;

    /** Computes the gradient of this constraint in vector form with pre-allocated memory.
     * i.e. returns delC(x)
     * 
     * @param grad (OUTPUT) - the pointer to the (currently empty) constraint gradient vector. Expects it to be _gradient_vector_size x 1.
     */
    inline void gradient(Real* delC) const override;

    /** Computes the value and gradient of this constraint with pre-allocated memory.
     * i.e. returns C(x) and delC(x) together.
     * 
     * This may be desirable when there would be duplicate work involved to evaluate constraint and its gradient separately.
     * 
     * @param C (OUTPUT) - the pointer to the (currently empty) value of the constraint
     * @param grad (OUTPUT) - the pointer to the (currently empty) constraint gradient vector. Expects it to be _gradient_vector_size x 1.
     */
    void evaluateWithGradient(Real* C, Real* grad) const override;

    #ifdef HAVE_CUDA
    typedef GPURigidDeformableCollisionConstraint GPUConstraintType;
    GPUConstraintType createGPUConstraint() const;
    #endif


    /** Collision constraints should be implemented as inequalities, i.e. as C(x) >= 0. */
    inline virtual bool isInequality() const override { return true; }

    /** Applies a frictional force to the two colliding bodies given the coefficients of friction and the Lagrange multiplier from this constraint.
     * @param lam - the Lagrange multiplier for this constraint after the XPBD update
     * @param mu_s - the coefficient of static friction between the two bodies
     * @param mu_k - the coefficient of kinetic friction between the two bodies
     */
    inline virtual void applyFriction(Real, Real, Real) const override;
    // inline virtual void applyFriction(Real lam, Real mu_s, Real mu_k) const override
    // {
    //     // since we are colliding with a rigid body, we need to apply frictional forces to both the deformable body and the rigid body

    //     // get Eigen vectors of positions and previous positions - just easier to work with
    //     const Vec3r p1 = Eigen::Map<Vec3r>(_positions[0].position_ptr);
    //     const Vec3r p2 = Eigen::Map<Vec3r>(_positions[1].position_ptr);
    //     const Vec3r p3 = Eigen::Map<Vec3r>(_positions[2].position_ptr);

    //     const Vec3r p1_prev = Eigen::Map<Vec3r>(_positions[0].prev_position_ptr);
    //     const Vec3r p2_prev = Eigen::Map<Vec3r>(_positions[1].prev_position_ptr);
    //     const Vec3r p3_prev = Eigen::Map<Vec3r>(_positions[2].prev_position_ptr);

    //     const Vec3r p_cur = _u*p1 + _v*p2 + _w*p3;    // current colliding point on the deformable body
    //     const Vec3r p_prev = _u*p1_prev + _v*p2_prev + _w*p3_prev;    // previous colliding point on the deformable body
    //     // get the movement of the colliding point in the directions tangent to the collision normal
    //     // this is directly related to the amount of tangential force felt by the colliding point
    //     const Vec3r dp = p_cur - p_prev;

    //     const Sim::RigidObject* obj = _rigid_bodies[0];
    //     const Vec3r rigid_p_cur = obj->bodyToGlobal(_point_on_rigid_body);    // current colliding point on the rigid body (global coords)
    //     const Vec3r rigid_p_prev = obj->prevPosition() + GeometryUtils::rotateVectorByQuat(_point_on_rigid_body, obj->prevOrientation()); // previous colliding point on the rigid body (global coords)
    //     // get the movement of the colliding point in the directions tangent to the collision normal
    //     const Vec3r rigid_dp = rigid_p_cur - rigid_p_prev;

    //     // get the relative movement of the two colliding points...
    //     const Vec3r relative_dp = dp - rigid_dp;
    //     const Vec3r dp_tan = relative_dp - (relative_dp.dot(_collision_normal))*_collision_normal;    // in the directions tangent to the collision normal

    //     // the mass at the deformable colliding point - weighted average of masses of the face vertices depending on barycentric coords
    //     const Real m = _u*(1.0/_positions[0].inv_mass) + _v*(1.0/_positions[1].inv_mass) + _w*(1.0/_positions[2].inv_mass);

    //     // we will apply a positional correction dx to vertices on the deformable body - the equivalent force on the rigid body is dx/(dt^2)
    //     // i.e. the force * dt * dt = dx
    //     Vec3r force_dtdt = Vec3r::Zero();

    //     // check to see if static friction should be enforced
    //     if (dp_tan.norm() < mu_s*lam/m)
    //     {
    //         // static friction is applied by undoing tangential movement this frame 
    //         // only do it for the vertices on the deformable object that have barycentric coordinates > 0

    //         if (_u>1e-4 && dp_tan.norm() < mu_s*lam*_positions[0].inv_mass)
    //         {
    //             const Vec3r dp1 = (p1 - p1_prev) - rigid_dp;
    //             const Vec3r dp1_tan = dp1 - (dp1.dot(_collision_normal))*_collision_normal;
    //             _positions[0].position_ptr[0] -= dp1_tan[0];
    //             _positions[0].position_ptr[1] -= dp1_tan[1];
    //             _positions[0].position_ptr[2] -= dp1_tan[2];
    //         }
                
    //         if (_v>1e-4 && dp_tan.norm() < mu_s*lam*_positions[1].inv_mass)
    //         {
    //             const Vec3r dp2 = (p2 - p2_prev) - rigid_dp;
    //             const Vec3r dp2_tan = dp2 - (dp2.dot(_collision_normal))*_collision_normal;
    //             _positions[1].position_ptr[0] -= dp2_tan[0];
    //             _positions[1].position_ptr[1] -= dp2_tan[1];
    //             _positions[1].position_ptr[2] -= dp2_tan[2];

    //         }

    //         if (_w>1e-4 && dp_tan.norm() < mu_s*lam*_positions[2].inv_mass)
    //         {
    //             const Vec3r dp3 = (p3 - p3_prev) - rigid_dp;
    //             const Vec3r dp3_tan = dp3 - (dp3.dot(_collision_normal))*_collision_normal;
    //             _positions[2].position_ptr[0] -= dp3_tan[0];
    //             _positions[2].position_ptr[1] -= dp3_tan[1];
    //             _positions[2].position_ptr[2] -= dp3_tan[2];
    //         }

    //         // the equivalent force * dt * dt oin the rigid body is simply just the tangential relative movement that we undid at the colliding point ont he deformable body
    //         force_dtdt = dp_tan;

    //     }
    //     // if not static friction, apply dynamic friction
    //     else
    //     {
    //         // calculate the positional correction for each vertex - never greater than the size of the tangential movement this time step
    //         const Vec3r corr_v1 = -_u * dp_tan * std::min(_positions[0].inv_mass*mu_k*lam/dp_tan.norm(), Real(1.0));
    //         const Vec3r corr_v2 = -_v * dp_tan * std::min(_positions[1].inv_mass*mu_k*lam/dp_tan.norm(), Real(1.0));
    //         const Vec3r corr_v3 = -_w * dp_tan * std::min(_positions[2].inv_mass*mu_k*lam/dp_tan.norm(), Real(1.0));
    //         _positions[0].position_ptr[0] += corr_v1[0];
    //         _positions[0].position_ptr[1] += corr_v1[1];
    //         _positions[0].position_ptr[2] += corr_v1[2];

    //         _positions[1].position_ptr[0] += corr_v2[0];
    //         _positions[1].position_ptr[1] += corr_v2[1];
    //         _positions[1].position_ptr[2] += corr_v2[2];

    //         _positions[2].position_ptr[0] += corr_v3[0];
    //         _positions[2].position_ptr[1] += corr_v3[1];
    //         _positions[2].position_ptr[2] += corr_v3[2];

    //         // the equivalent force * dt * dt is the opposite positional correction that we applied to the colliding point ont he deformable body
    //         force_dtdt = -(corr_v1/_positions[0].inv_mass + corr_v2/_positions[1].inv_mass + corr_v3/_positions[2].inv_mass);
    //     }


    //     // apply the force on the rigid body
    //     // the force is applied at the colliding point on the rigid body

    //     // update orientation of rigid body - from the torque caused by the applied force * dt * dt
    //     // these formulas are the same as when we apply an impulse to a rigid body during a collision
    //     const Vec3r torque = (rigid_p_cur - _rigid_bodies[0]->position()).cross(force_dtdt);
    //     const Vec3r body_torque = GeometryUtils::rotateVectorByQuat(torque, GeometryUtils::inverseQuat(_rigid_bodies[0]->orientation()));
    //     const Vec3r body_omega = _rigid_bodies[0]->invI() * body_torque;
    //     const Vec3r global_omega = GeometryUtils::rotateVectorByQuat(body_omega, _rigid_bodies[0]->orientation());
    //     const Vec4r w4({global_omega[0], global_omega[1], global_omega[2], 0.0});
    //     const Vec4r q_update = 0.5 * (GeometryUtils::quatMult(w4, _rigid_bodies[0]->orientation()));
    //     _rigid_bodies[0]->setOrientation(_rigid_bodies[0]->orientation() + q_update);

    //     // update position
    //     _rigid_bodies[0]->setPosition(_rigid_bodies[0]->position() + force_dtdt / _rigid_bodies[0]->mass() );
    // }

    private:

    protected:
    const Geometry::SDF* _sdf;  // the SDF of the rigid body - used to get accurate penetration distance
    Vec3r _point_on_rigid_body; // the colliding point on the rigid body, in global coordinates
    Real _u;  // barycentric coordinates of point on deformable body
    Real _v;  // the vertices themselves are stored in the _positions vector defined in the Constraint base class
    Real _w;


};

} // namespace Solver

#endif // __RIGID_DEFORMABLE_COLLISION_CONSTRAINT_HPP