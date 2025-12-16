#include "solver/constraint/DeformableDeformableCollisionConstraint.hpp"

#include "utils/MathUtils.hpp"
#include <iostream>

namespace Solver
{
DeformableDeformableCollisionConstraint::DeformableDeformableCollisionConstraint(int v, PositionReference::VecType* vec_ptr, Real m,
                                                                        int fv1, PositionReference::VecType* fvec_ptr1, Real fm1,
                                                                        int fv2, PositionReference::VecType* fvec_ptr2, Real fm2,
                                                                        int fv3, PositionReference::VecType* fvec_ptr3, Real fm3)
    : Constraint(std::vector<PositionReference>({
    PositionReference(v, vec_ptr, m),
    PositionReference(fv1, fvec_ptr1, fm1),
    PositionReference(fv2, fvec_ptr2, fm2),
    PositionReference(fv3, fvec_ptr3, fm3)}), 1e-8)
{

}

void DeformableDeformableCollisionConstraint::evaluate(Real* C) const
{
    const Vec3r& q = _positions[0].position();
    const Vec3r& p1= _positions[1].position();
    const Vec3r& p2= _positions[2].position();
    const Vec3r& p3= _positions[3].position();

    const Vec3r a = (p2 - p1).cross(p3 - p1);
    Real a_norm = a.norm();


    *C = (q - p1).dot(a) / a_norm + 1e-5;
}

void DeformableDeformableCollisionConstraint::gradient(Real* delC) const
{
    const Vec3r& q  = _positions[0].position();
    const Vec3r& p1 = _positions[1].position();
    const Vec3r& p2 = _positions[2].position();
    const Vec3r& p3 = _positions[3].position();

    const Vec3r a = (p2 - p1).cross(p3 - p1);
    Real a_norm = a.norm();

    const Vec3r gq = a/a_norm;

    const Mat3r I_aaT = Mat3r::Identity()/a_norm - a*a.transpose() / (a_norm * a_norm * a_norm);
    const Vec3r gp1 = -a.transpose()/a_norm + (q-p1).transpose() * I_aaT * MathUtils::Skew3(p3 - p2);
    const Vec3r gp2 = (q-p1).transpose() * I_aaT * MathUtils::Skew3(p1 - p3);
    const Vec3r gp3 = (q-p2).transpose() * I_aaT * MathUtils::Skew3(p2 - p1);

    delC[0] = gq[0];
    delC[1] = gq[1];
    delC[2] = gq[2];

    delC[3] = gp1[0];
    delC[4] = gp1[1];
    delC[5] = gp1[2];
    
    delC[6] = gp2[0];
    delC[7] = gp2[1];
    delC[8] = gp2[2];

    delC[9] = gp3[0];
    delC[10] = gp3[1];
    delC[11] = gp3[2];
}

void DeformableDeformableCollisionConstraint::evaluateWithGradient(Real* C, Real* grad) const
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

// void DeformableDeformableCollisionConstraint::applyFriction(Real, Real, Real) const {}
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