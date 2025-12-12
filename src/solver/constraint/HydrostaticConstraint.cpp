#include "solver/constraint/HydrostaticConstraint.hpp"

#include <iostream>

namespace Solver
{

HydrostaticConstraint::HydrostaticConstraint(int v1, Real* p1, Real m1,
                          int v2, Real* p2, Real m2,
                          int v3, Real* p3, Real m3,
                          int v4, Real* p4, Real m4,
                          const ElasticMaterial& material)
    : ElementConstraint(v1, p1, m1, v2, p2, m2, v3, p3, m3, v4, p4, m4)
{
    _alpha = 1/(material.lambda() * _volume);            // set alpha after the ElementConstraint constructor because we need the element volume
    _gamma = material.mu() / material.lambda();  
}

void HydrostaticConstraint::evaluate(Real* C) const
{
    Real F[9];
    Real X[9];

    _computeF(F, X);
    _evaluate(C, F);
}

void HydrostaticConstraint::gradient(Real* grad) const
{
    Real F[9];
    Real X[9];
    _computeF(F, X);
    _gradient(grad, F);
}

HydrostaticConstraint::HessianMatType HydrostaticConstraint::hessian() const
{
    // compute the deformation gradient
    Real F[9];
    Real X[9];
    _computeF(F, X);

    // initialize Hessian
    HessianMatType hessian_mat = HessianMatType::Zero();

    // extract rows of deformation gradient
    const Vec3r f1r(F[0], F[3], F[6]);
    const Vec3r f2r(F[1], F[4], F[7]);
    const Vec3r f3r(F[2], F[5], F[8]);

    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            if (i == j)
                continue;

            const Vec3r qir = _Q.row(i);
            const Vec3r qjr = _Q.row(j);

            const Real qir_f1r_qjr = qir.dot( f1r.cross(qjr) );
            const Real qir_f2r_qjr = qir.dot( f2r.cross(qjr) );
            const Real qir_f3r_qjr = qir.dot( f3r.cross(qjr) );
            
            Mat3r hess_3x3_block;
            hess_3x3_block << 0, -qir_f3r_qjr, qir_f2r_qjr,
                              qir_f3r_qjr, 0, -qir_f1r_qjr,
                              -qir_f2r_qjr, qir_f1r_qjr, 0;
            
            hessian_mat.block<3,3>(3*i, 3*j) = hess_3x3_block;

            // subtract the 3x3 block from the part of the Hessian that's w.r.t. the 4th vertex
            hessian_mat.block<3,3>(3*i, 9) -= hess_3x3_block;
            hessian_mat.block<3,3>(9, 3*j) -= hess_3x3_block;
        }
    }

    return hessian_mat;
}

#ifdef HAVE_CUDA
HydrostaticConstraint::GPUConstraintType HydrostaticConstraint::createGPUConstraint() const
{
    GPUConstraintType gpu_constraint = GPUConstraintType(_positions[0].index, _positions[0].inv_mass,
                                                            _positions[1].index, _positions[1].inv_mass,
                                                            _positions[2].index, _positions[2].inv_mass,
                                                            _positions[3].index, _positions[3].inv_mass,
                                                            _Q, _alpha, _gamma);
    return gpu_constraint;
}
#endif

} // namespace Solver