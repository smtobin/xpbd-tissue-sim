#include "solver/xpbd_projector/CombinedNeohookeanConstraintProjector.hpp"

#include <iostream>

namespace Solver
{

// template<>
// template class CombinedConstraintProjector<DeviatoricConstraint, HydrostaticConstraint>;

// CombinedConstraintProjector<DeviatoricConstraint, HydrostaticConstraint>::CombinedConstraintProjector(Real dt, DeviatoricConstraint* dev_constraint, HydrostaticConstraint* hyd_constraint)
// {

// }

template<>
void CombinedConstraintProjector<true, DeviatoricConstraint, HydrostaticConstraint>::project(CoordinateUpdate* coordinate_updates_ptr)
{
    Real C[2];
    Real delC[DeviatoricConstraint::NUM_COORDINATES + HydrostaticConstraint::NUM_COORDINATES];

    // compute F
    Real F[9];
    Real X[9];
    _constraint1->_computeF(F, X);

    // evaluate constraint and gradients for each constraint
    _constraint1->_evaluate(C, F);
    _constraint1->_gradient(delC, C, F);

    _constraint2->_evaluate(C+1, F);
    _constraint2->_gradient(delC + DeviatoricConstraint::NUM_COORDINATES, F);

    // rest of projection is exactly the same...
    Real alpha_tilde[2] = { _constraint1->alpha() / (_dt), _constraint2->alpha() / (_dt) };
    Real LHS[4];
    for (int ci = 0; ci < 2; ci++)
    {
        for (int cj = 0; cj < 2; cj++)
        {
            if (ci == cj) LHS[cj*2 + ci] = alpha_tilde[ci];
            else LHS[cj*2 + ci] = 0;
        }
    }

    for (int ci = 0; ci < 2; ci++)
    {
        Real* delC_i = delC + ci*DeviatoricConstraint::NUM_COORDINATES;
        for (int cj = ci; cj < 2; cj++)
        {
            Real* delC_j = delC + cj*DeviatoricConstraint::NUM_COORDINATES;

            for (int i = 0; i < DeviatoricConstraint::NUM_POSITIONS; i++)
            {
                LHS[cj*2 + ci] += _constraint1->positions()[i].inv_mass * (delC_i[3*i]*delC_j[3*i] + delC_i[3*i+1]*delC_j[3*i+1] + delC_i[3*i+2]*delC_j[3*i+2]);
            }

            LHS[ci*2 + cj] = LHS[cj*2 + ci];
        }
        
    }

    // compute RHS of lambda update: -C - alpha_tilde * lambda
    Real RHS[2];
    for (int ci = 0; ci < 2; ci++)
    {
        RHS[ci] = -C[ci] - alpha_tilde[ci] * _lambda[ci];
    }

    // compute lambda update - solve 2x2 system
    Real dlam[2];
    const Real det = LHS[0]*LHS[3] - LHS[1]*LHS[2];

    dlam[0] = (RHS[0]*LHS[3] - RHS[1]*LHS[2]) / det;
    dlam[1] = (RHS[1]*LHS[0] - RHS[0]*LHS[1]) / det;

    // update lambdas
    _lambda[0] += dlam[0];
    _lambda[1] += dlam[1];

    // compute position updates
    Real* delC_c2 = delC + HydrostaticConstraint::NUM_COORDINATES;
    for (int i = 0; i < DeviatoricConstraint::NUM_POSITIONS; i++)
    {
        Real update_x = _constraint1->positions()[i].inv_mass * (delC[3*i] * dlam[0] + delC_c2[3*i] * dlam[1]);
        Real update_y = _constraint1->positions()[i].inv_mass * (delC[3*i+1] * dlam[0] + delC_c2[3*i+1] * dlam[1]);
        Real update_z = _constraint1->positions()[i].inv_mass * (delC[3*i+2] * dlam[0] + delC_c2[3*i+2] * dlam[1]);
        
        coordinate_updates_ptr[3*i].ptr = _constraint1->positions()[i].positionPtr();
        coordinate_updates_ptr[3*i].update = update_x;
        coordinate_updates_ptr[3*i+1].ptr = _constraint1->positions()[i].positionPtr()+1;
        coordinate_updates_ptr[3*i+1].update = update_y;
        coordinate_updates_ptr[3*i+2].ptr = _constraint1->positions()[i].positionPtr()+2;
        coordinate_updates_ptr[3*i+2].update = update_z;
    }
}

template<>
void CombinedConstraintProjector<false, DeviatoricConstraint, HydrostaticConstraint>::project(CoordinateUpdate* coordinate_updates_ptr)
{
    Real C[2];
    Real delC[DeviatoricConstraint::NUM_COORDINATES + HydrostaticConstraint::NUM_COORDINATES];

    // compute F
    Real F[9];
    Real X[9];
    _constraint1->_computeF(F, X);

    // evaluate constraint and gradients for each constraint
    _constraint1->_evaluate(C, F);
    _constraint1->_gradient(delC, C, F);

    _constraint2->_evaluate(C+1, F);
    _constraint2->_gradient(delC + DeviatoricConstraint::NUM_COORDINATES, F);

    // rest of projection is exactly the same...
    Real alpha_tilde[2] = { _constraint1->alpha() / (_dt * _dt), _constraint2->alpha() / (_dt * _dt) };
    Real LHS[4];
    for (int ci = 0; ci < 2; ci++)
    {
        for (int cj = 0; cj < 2; cj++)
        {
            if (ci == cj) LHS[cj*2 + ci] = alpha_tilde[ci];
            else LHS[cj*2 + ci] = 0;
        }
    }

    for (int ci = 0; ci < 2; ci++)
    {
        Real* delC_i = delC + ci*DeviatoricConstraint::NUM_COORDINATES;
        for (int cj = ci; cj < 2; cj++)
        {
            Real* delC_j = delC + cj*DeviatoricConstraint::NUM_COORDINATES;

            for (int i = 0; i < DeviatoricConstraint::NUM_POSITIONS; i++)
            {
                LHS[cj*2 + ci] += _constraint1->positions()[i].inv_mass * (delC_i[3*i]*delC_j[3*i] + delC_i[3*i+1]*delC_j[3*i+1] + delC_i[3*i+2]*delC_j[3*i+2]);
            }

            LHS[ci*2 + cj] = LHS[cj*2 + ci];
        }
        
    }
    // compute RHS of lambda update: -C - alpha_tilde * lambda
    Real RHS[2];
    for (int ci = 0; ci < 2; ci++)
    {
        RHS[ci] = -C[ci] - alpha_tilde[ci] * _lambda[ci];
    }

    // compute lambda update - solve 2x2 system
    Real dlam[2];
    const Real det = LHS[0]*LHS[3] - LHS[1]*LHS[2];

    dlam[0] = (RHS[0]*LHS[3] - RHS[1]*LHS[2]) / det;
    dlam[1] = (RHS[1]*LHS[0] - RHS[0]*LHS[1]) / det;

    // std::cout << "dlam[0]: " << dlam[0] << ", dlam[1]: " << dlam[1] << std::endl;

    // update lambdas
    _lambda[0] += dlam[0];
    _lambda[1] += dlam[1];

    // compute position updates
    Real* delC_c2 = delC + HydrostaticConstraint::NUM_COORDINATES;
    for (int i = 0; i < DeviatoricConstraint::NUM_POSITIONS; i++)
    {
        Real update_x = _constraint1->positions()[i].inv_mass * (delC[3*i] * dlam[0] + delC_c2[3*i] * dlam[1]);
        Real update_y = _constraint1->positions()[i].inv_mass * (delC[3*i+1] * dlam[0] + delC_c2[3*i+1] * dlam[1]);
        Real update_z = _constraint1->positions()[i].inv_mass * (delC[3*i+2] * dlam[0] + delC_c2[3*i+2] * dlam[1]);
        
        coordinate_updates_ptr[3*i].ptr = _constraint1->positions()[i].positionPtr();
        coordinate_updates_ptr[3*i].update = update_x;
        coordinate_updates_ptr[3*i+1].ptr = _constraint1->positions()[i].positionPtr()+1;
        coordinate_updates_ptr[3*i+1].update = update_y;
        coordinate_updates_ptr[3*i+2].ptr = _constraint1->positions()[i].positionPtr()+2;
        coordinate_updates_ptr[3*i+2].update = update_z;
    }
}


} // namespace Solver