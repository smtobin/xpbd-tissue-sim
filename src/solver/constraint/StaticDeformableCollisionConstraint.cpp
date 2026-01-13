#include "solver/constraint/StaticDeformableCollisionConstraint.hpp"
#include <iostream>

namespace Solver
{
StaticDeformableCollisionConstraint::StaticDeformableCollisionConstraint(const Geometry::SDF* sdf, const Vec3r& p, const Vec3r& n,
                                                                        int v1, PositionReference::VecType* vec_ptr1, Real m1,
                                                                        int v2, PositionReference::VecType* vec_ptr2, Real m2,
                                                                        int v3, PositionReference::VecType* vec_ptr3, Real m3,
                                                                        Real u, Real v, Real w,
                                                                        int face_index)
    : CollisionConstraint(std::vector<PositionReference>({
    PositionReference(v1, vec_ptr1, m1),
    PositionReference(v2, vec_ptr2, m2),
    PositionReference(v3, vec_ptr3, m3)}), n),
    _sdf(sdf), _p(p), _u(u), _v(v), _w(w), _face_index(face_index)
{

}

void StaticDeformableCollisionConstraint::evaluate(Real* C) const
{
    const Vec3r a = _u*_positions[0].position() + _v*_positions[1].position() + _w*_positions[2].position();
    *C = _collision_normal.dot(a - _p);

    // if (_positions[0].index == 337 || _positions[1].index == 337 || _positions[2].index == 337)
    // {
    //     std::cout << "Index 337 collision constraint violation: " << *C << std::endl;
    // }

    // std::cout << "C: " << *C << std::endl;
}

void StaticDeformableCollisionConstraint::gradient(Real* delC) const
{
    delC[0] = _u*_collision_normal[0];
    delC[1] = _u*_collision_normal[1];
    delC[2] = _u*_collision_normal[2];

    delC[3] = _v*_collision_normal[0];
    delC[4] = _v*_collision_normal[1];
    delC[5] = _v*_collision_normal[2];
    
    delC[6] = _w*_collision_normal[0];
    delC[7] = _w*_collision_normal[1];
    delC[8] = _w*_collision_normal[2];

    // std::cout << "delC: " << delC[0] << ", " << delC[1] << ", " << delC[2] << ", " << delC[3] << ", " << delC[4] << ", " << delC[5] << ", " << delC[6] << ", " << delC[7] << ", " << delC[8] << std::endl;

}

void StaticDeformableCollisionConstraint::evaluateWithGradient(Real* C, Real* grad) const
{
    evaluate(C);
    gradient(grad);
}

#ifdef HAVE_CUDA
StaticDeformableCollisionConstraint::GPUConstraintType StaticDeformableCollisionConstraint::createGPUConstraint() const
{
    GPUConstraintType gpu_constraint = GPUConstraintType(_positions[0].index, _positions[0].inv_mass,
                                                            _positions[1].index, _positions[1].inv_mass,
                                                            _positions[2].index, _positions[2].inv_mass,
                                                            _u, _v, _w,
                                                            _p, _collision_normal);
    return gpu_constraint;
}
#endif

void StaticDeformableCollisionConstraint::applyFriction(Real, Real, Real) const {}
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

} // namespace Solver