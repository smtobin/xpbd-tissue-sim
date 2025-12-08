#include "solver/constraint/HydrostaticConstraint.hpp"

#include <iostream>

namespace Solver
{

HydrostaticConstraint::HydrostaticConstraint(int v1, Real* p1, Real m1,
                          int v2, Real* p2, Real m2,
                          int v3, Real* p3, Real m3,
                          int v4, Real* p4, Real m4,
                          const ElasticMaterial& material,
                        Real pressure_correction)
    : ElementConstraint(v1, p1, m1, v2, p2, m2, v3, p3, m3, v4, p4, m4)
{
    _alpha = 1/(material.lambda() * _volume);            // set alpha after the ElementConstraint constructor because we need the element volume
    _gamma = -pressure_correction + material.mu() / material.lambda();  
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