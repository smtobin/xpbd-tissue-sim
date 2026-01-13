#include "solver/constraint/DeviatoricConstraint.hpp"

namespace Solver
{
DeviatoricConstraint::DeviatoricConstraint()
    : ElementConstraint()
{}

DeviatoricConstraint::DeviatoricConstraint(int v1, PositionReference::VecType* vec_ptr1, Real m1,
                        int v2, PositionReference::VecType* vec_ptr2, Real m2,
                        int v3, PositionReference::VecType* vec_ptr3, Real m3,
                        int v4, PositionReference::VecType* vec_ptr4, Real m4,
                        const ElasticMaterial& material)
    : ElementConstraint(v1, vec_ptr1, m1, v2, vec_ptr2, m2, v3, vec_ptr3, m3, v4, vec_ptr4, m4)
{
    _alpha = 1/(material.mu() * _volume); // set alpha after the ElementConstraint constructor because we need the element volume
}

DeviatoricConstraint::DeviatoricConstraint(int v1, PositionReference::VecType* vec_ptr1, Real m1,
                        int v2, PositionReference::VecType* vec_ptr2, Real m2,
                        int v3, PositionReference::VecType* vec_ptr3, Real m3,
                        int v4, PositionReference::VecType* vec_ptr4, Real m4,
                        const ElasticMaterial& material, const Mat3r& Q, Real volume)
    : ElementConstraint(v1, vec_ptr1, m1, v2, vec_ptr2, m2, v3, vec_ptr3, m3, v4, vec_ptr4, m4, Q, volume)
{
    _alpha = 1/(material.mu() * _volume); // set alpha after the ElementConstraint constructor because we need the element volume
}


void DeviatoricConstraint::evaluate(Real* C) const
{
    Real F[9];
    Real X[9];
    _computeF(F, X);
    _evaluate(C, F);
}

void DeviatoricConstraint::gradient(Real* grad) const
{
    Real F[9];
    Real X[9];
    _computeF(F, X);
    Real C;
    _evaluate(&C, F);                   // we need C(x) since it is used in the gradient calculation
    _gradient(grad, &C, F);
}

DeviatoricConstraint::HessianMatType DeviatoricConstraint::hessian() const
{
    // compute the deformation gradient
    Real F[9];
    Real X[9];
    _computeF(F, X);

    // compute the constraint itself
    Real C;
    _evaluate(&C, F);

    // initialize Hessian
    HessianMatType hessian_mat = HessianMatType::Zero();
    
    Eigen::Map<const Mat3r> F_mat(F);

    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            const Vec3r qir = _Q.row(i);
            const Vec3r qjr = _Q.row(j);

            const Vec3r Fqir = F_mat * qir;
            const Vec3r Fqjr = F_mat * qjr;
            
            // 2 terms from product rule
            Mat3r term1 = -1/(C*C*C) * (Fqir * Fqjr.transpose());
            Mat3r term2 = 1/C * qir.dot(qjr) * Mat3r::Identity();
            Mat3r hess_3x3_block =  term1 + term2;
            hessian_mat.block<3,3>(3*i, 3*j) = hess_3x3_block;

            // subtract the 3x3 block from the part of the Hessian that's w.r.t. the 4th vertex
            hessian_mat.block<3,3>(3*i, 9) -= hess_3x3_block;
            hessian_mat.block<3,3>(9, 3*j) -= hess_3x3_block;
            hessian_mat.block<3,3>(9, 9) += hess_3x3_block; // note the plus sign - minus signs cancel out in this case
        }
    }

    return hessian_mat;
}

#ifdef HAVE_CUDA
DeviatoricConstraint::GPUConstraintType DeviatoricConstraint::createGPUConstraint() const
{
    GPUConstraintType gpu_constraint = GPUConstraintType(_positions[0].index, _positions[0].inv_mass,
                                                            _positions[1].index, _positions[1].inv_mass,
                                                            _positions[2].index, _positions[2].inv_mass,
                                                            _positions[3].index, _positions[3].inv_mass,
                                                            _Q, _alpha);
    return gpu_constraint;
}
#endif

} // namespace Solver