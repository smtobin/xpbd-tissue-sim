#include "solver/constraint/DeviatoricConstraint.hpp"

namespace Solver
{

DeviatoricConstraint::DeviatoricConstraint(int v1, PositionReference::VecType* vec_ptr1, Real m1,
                        int v2, PositionReference::VecType* vec_ptr2, Real m2,
                        int v3, PositionReference::VecType* vec_ptr3, Real m3,
                        int v4, PositionReference::VecType* vec_ptr4, Real m4,
                        const ElasticMaterial& material)
    : ElementConstraint(v1, vec_ptr1, m1, v2, vec_ptr2, m2, v3, vec_ptr3, m3, v4, vec_ptr4, m4)
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