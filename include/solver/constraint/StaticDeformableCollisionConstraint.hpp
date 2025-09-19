#ifndef __STATIC_DEFORMABLE_COLLISION_CONSTRAINT_HPP
#define __STATIC_DEFORMABLE_COLLISION_CONSTRAINT_HPP

#include "solver/constraint/CollisionConstraint.hpp"
#include "geometry/SDF.hpp"

#ifdef HAVE_CUDA
#include "gpu/constraint/GPUStaticDeformableCollisionConstraint.cuh"
#endif

namespace Solver
{

/** Collision constraint between a point on a static rigid body and a point on a deformable body. */
class StaticDeformableCollisionConstraint : public CollisionConstraint
{
    public:
    constexpr static int NUM_POSITIONS = 3;
    constexpr static int NUM_COORDINATES = 9;

    public:
    StaticDeformableCollisionConstraint(const Geometry::SDF* sdf, const Vec3r& p, const Vec3r& n,
                                        int v1, Real* p1, Real m1,
                                        int v2, Real* p2, Real m2,
                                        int v3, Real* p3, Real m3,
                                        Real u, Real v, Real w,
                                        int face_index=-1);

    int numPositions() const override { return NUM_POSITIONS; }
    int numCoordinates() const override { return NUM_COORDINATES; }

    int faceIndex() const { return _face_index; }

    /** Evaluates the current value of this constraint with pre-allocated memory.
     * i.e. returns C(x)
     * 
     * @param C (OUTPUT) - the pointer to the (currently empty) value of the constraint
     */
    void evaluate(Real* C) const override;

    /** Computes the gradient of this constraint in vector form with pre-allocated memory.
     * i.e. returns delC(x)
     * 
     * @param grad (OUTPUT) - the pointer to the (currently empty) constraint gradient vector. Expects it to be _gradient_vector_size x 1.
     */
    void gradient(Real* delC) const override;


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
    typedef GPUStaticDeformableCollisionConstraint GPUConstraintType;
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
    //     // since we are colliding with a static point/body, only need to apply frictional forces to the deformable body
    //     // also, the relative velocity between the two colliding points will just be the velocity of the point on the deformable body (duh)

    //     // get Eigen vectors of positions and previous positions - just easier to work with
    //     const Vec3r p1 = Eigen::Map<Vec3r>(_positions[0].position_ptr);
    //     const Vec3r p2 = Eigen::Map<Vec3r>(_positions[1].position_ptr);
    //     const Vec3r p3 = Eigen::Map<Vec3r>(_positions[2].position_ptr);

    //     const Vec3r p1_prev = Eigen::Map<Vec3r>(_positions[0].prev_position_ptr);
    //     const Vec3r p2_prev = Eigen::Map<Vec3r>(_positions[1].prev_position_ptr);
    //     const Vec3r p3_prev = Eigen::Map<Vec3r>(_positions[2].prev_position_ptr);

    //     const Vec3r p_cur = _u*p1 + _v*p2 + _w*p3;    // current colliding point on the deformable body
    //     const Vec3r p_prev = _u*p1_prev + _v*p2_prev + _w*p3_prev;    // previous colliding point on the deformable body
    //     const Vec3r dp = p_cur - p_prev; 
    //     // get the movement of the colliding point in the directions tangent to the collision normal
    //     // this is directly related to the amount of tangential force felt by the colliding point
    //     const Vec3r dp_tan = dp - (dp.dot(_collision_normal))*_collision_normal; 

    //     // the mass at the colliding point - weighted average of masses of the face vertices
    //     const Real m = _u*(1.0/_positions[0].inv_mass) + _v*(1.0/_positions[1].inv_mass) + _w*(1.0/_positions[2].inv_mass);

    //     // check to see if static friction should be enforced
    //     if (dp_tan.norm() < mu_s*lam/m)
    //     {
    //         // static friction is applied by undoing tangential movement this frame 
    //         // only do it for the vertices that have barycentric coordinates > 0

    //         if (_u>1e-4 && dp_tan.norm() < mu_s*lam*_positions[0].inv_mass)
    //         {
    //             const Vec3r dp1 = p1 - p1_prev;
    //             const Vec3r dp1_tan = dp1 - (dp1.dot(_collision_normal))*_collision_normal;
    //             _positions[0].position_ptr[0] -= dp1_tan[0];
    //             _positions[0].position_ptr[1] -= dp1_tan[1];
    //             _positions[0].position_ptr[2] -= dp1_tan[2];
    //         }
                
    //         if (_v>1e-4 && dp_tan.norm() < mu_s*lam*_positions[1].inv_mass)
    //         {
    //             const Vec3r dp2 = p2 - p2_prev;
    //             const Vec3r dp2_tan = dp2 - (dp2.dot(_collision_normal))*_collision_normal;
    //             _positions[1].position_ptr[0] -= dp2_tan[0];
    //             _positions[1].position_ptr[1] -= dp2_tan[1];
    //             _positions[1].position_ptr[2] -= dp2_tan[2];

    //         }

    //         if (_w>1e-4 && dp_tan.norm() < mu_s*lam*_positions[2].inv_mass)
    //         {
    //             const Vec3r dp3 = p3 - p3_prev;
    //             const Vec3r dp3_tan = dp3 - (dp3.dot(_collision_normal))*_collision_normal;
    //             _positions[2].position_ptr[0] -= dp3_tan[0];
    //             _positions[2].position_ptr[1] -= dp3_tan[1];
    //             _positions[2].position_ptr[2] -= dp3_tan[2];
    //         }
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
    //     }
    // }

    protected:
    const Geometry::SDF* _sdf;
    Vec3r _p;
    Real _u;
    Real _v;
    Real _w;
    int _face_index;

};

} // namespace Solver

#endif // __STATIC_DEFORMABLE_COLLISION_CONSTRAINT_HPP