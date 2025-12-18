#ifndef __HYDROSTATIC_CONSTRAINT_HPP
#define __HYDROSTATIC_CONSTRAINT_HPP

#include "solver/constraint/ElementConstraint.hpp"
#include "simobject/ElasticMaterial.hpp"

#ifdef HAVE_CUDA
#include "gpu/constraint/GPUHydrostaticConstraint.cuh"
#endif


namespace Solver
{

// TODO: systematic way for forward declarations...
template<bool IsFirstOrder, typename Constraint1, typename Constraint2>
class CombinedConstraintProjector;

class DeviatoricConstraint;

/** Represents the hydrostatic constraint derived from the Stable Neo-hookean strain energy, proposed by Macklin et. al: https://mmacklin.com/neohookean.pdf
 */
class HydrostaticConstraint : public ElementConstraint
{
    friend class CombinedConstraintProjector<true, DeviatoricConstraint, HydrostaticConstraint>;
    friend class CombinedConstraintProjector<false, DeviatoricConstraint, HydrostaticConstraint>;

    constexpr static int _NUM_TAYLOR_SERIES_TERMS = 7;  // number of Taylor series terms to be used for approximation of log(J)

    public:
    constexpr static int NUM_POSITIONS = 4; 
    constexpr static int NUM_COORDINATES = 12;
    
    public:
    /** Creates the hydrostatic constraint from a MeshObject and the 4 vertices that make up the tetrahedral element. */
    HydrostaticConstraint(int v1, PositionReference::VecType* vec_ptr1, Real m1,
                          int v2, PositionReference::VecType* vec_ptr2, Real m2,
                          int v3, PositionReference::VecType* vec_ptr3, Real m3,
                          int v4, PositionReference::VecType* vec_ptr4, Real m4,
                          const ElasticMaterial& material);

    HydrostaticConstraint(int v1, PositionReference::VecType* vec_ptr1, Real m1,
                          int v2, PositionReference::VecType* vec_ptr2, Real m2,
                          int v3, PositionReference::VecType* vec_ptr3, Real m3,
                          int v4, PositionReference::VecType* vec_ptr4, Real m4,
                          const ElasticMaterial& material, const Mat3r& Q, Real rest_volume);

    HydrostaticConstraint();

    int numPositions() const override { return NUM_POSITIONS; }
    int numCoordinates() const override { return NUM_COORDINATES; }
    bool isInequality() const override { return false; }


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
    void gradient(Real* grad) const override;


    /** Computes the value and gradient of this constraint with pre-allocated memory.
     * i.e. returns C(x) and delC(x) together.
     * 
     * This may be desirable when there would be duplicate work involved to evaluate constraint and its gradient separately.
     * 
     * Inline for performance reasons.
     * 
     * @param C (OUTPUT) - the pointer to the (currently empty) value of the constraint
     * @param grad (OUTPUT) - the pointer to the (currently empty) constraint gradient vector. Expects it to be _gradient_vector_size x 1.
     */
    void evaluateWithGradient(Real* C, Real* grad) const override
    {
        Real F[9];
        Real X[9];
        _computeF(F, X);
        _evaluate(C, F);

        // if (*C > 1000)
        // {
        //     std::cout << "C > 1000! F=\n" << F[0] << ", " << F[1] << ", " << F[2] << "\n" << F[3] << ", " << F[4] << ", " << F[5] << "\n" << F[6] << ", " << F[7] << ", " << F[8] << std::endl;
        // }
        _gradient(grad, F);
    }

    Vec3r elasticForce(int index) const
    {
        Real C;
        Real delC[12];
        evaluateWithGradient(&C, delC);

        // compute elastic force for index
        int pos_index = -1;
        for (unsigned i = 0; i < _positions.size(); i++)
        {
            if (index == _positions[i].index)
            {
                pos_index = i;
                break;
            }
        }
        if (pos_index == -1)
            assert(0 && "This constraint does not affect passed position!");
        
        Vec3r grad = Eigen::Map<Vec3r>(delC + 3*pos_index);

        // std::cout << "grad: " << grad[0] << ", " << grad[1] << ", " << grad[2] << "  1/alpha: " << 1.0/_alpha << "  C: " << C << std::endl;
        return -grad * (1.0/_alpha) * C;
    }

    #ifdef HAVE_CUDA
    typedef GPUHydrostaticConstraint GPUConstraintType;
    GPUConstraintType createGPUConstraint() const;
    #endif

    private:

    /** Note - these are inline for performance reasons. (~7% speedup) */

    /** Helper method to evaluate the constraint given the deformation gradient, F, using pre-allocated memory.
     * Avoids the need to recompute F if we already have it.
     */
    void _evaluate(Real* C, Real* F) const
    {
        // compute C(x) = det(F) - (1 + gamma)
        Real detF = F[0]*F[4]*F[8] - F[0]*F[7]*F[5] - F[3]*F[1]*F[8] + F[3]*F[7]*F[2] + F[6]*F[1]*F[5] - F[6]*F[4]*F[2];

        // *C = detF - 1 - _gamma;
        if (detF >= 1)
        {
            // when J >= 1, just log(J)
            // -gamma is a term needed for rest-stability
            *C = -_gamma + std::log(detF);
            // *C = detF - 1 - _gamma;
        }
        else
        {
            // when J < 1, approximate log(J) with its Taylor series
            // log(J) = (J-1) - 1/2*(J-1)^2 + 1/3*(J-1)^3 -...
            *C = -_gamma + (detF-1) - (detF-1)*(detF-1)/2.0 + (detF-1)*(detF-1)*(detF-1)/3.0;
            // variable to track (J-1)^n
            // Real detF_min_1_n = 1;
            // for (int i = 0; i < _NUM_TAYLOR_SERIES_TERMS; i++)
            // {
            //     // when i = 0,2,4... we want the terms to be positive
            //     // when i = 1,3,5... we want the terms to be negative
            //     int sign = (i%2 == 0) ? 1 : -1;
            //     detF_min_1_n *= (detF - 1);
            //     // add the next term in the series to *C
            //     *C += sign * (detF_min_1_n/(i+1));
            // }
        }
    }

    /** Helper method to evaluate the constraint gradient given the deformation gradient, F, useing pre-allocated memory.
     * Avoids the need to recompute F if we already have it.
     */
    void _gradient(Real* delC, Real* F) const
    {
        // F_cross = [f2 x f3, f3 x f1, f1 x f2] where f_i are the columns of F
        // F_cross and F are both column-major
        // see supplementary material of Macklin paper for more details
        Real F_cross[9];
        _cross3(F+3, F+6, F_cross);             // 2nd column of F crossed with 3rd column
        _cross3(F+6, F, F_cross+3);             // 3rd column of F crossed with 1st column
        _cross3(F, F+3, F_cross+6);             // 1st column of F crossed with 2nd column

        // for A = F_cross * Q^T,
        // 1st column of A is delC wrt 1st position
        // 2nd column of A is delC wrt 2nd position
        // 3rd column of A is delC wrt 3rd position
        // delC wrt 4th position is (-1st column - 2nd column - 3rd column)
        // see supplementary material of Macklin paper for more details

        // the factor out front of the old gradient F_cross * Q^T
        Real fac;
        Real detF = F[0]*F[4]*F[8] - F[0]*F[7]*F[5] - F[3]*F[1]*F[8] + F[3]*F[7]*F[2] + F[6]*F[1]*F[5] - F[6]*F[4]*F[2];
        
        // fac = 1;
        if (detF >= 1.0)
        {
            // when J >= 1, this factor is just the derivative of log(J) = 1/J
            fac = 1/detF;
            // fac = 1;
        }
        else
        {
            // when J < 1, this factor is the derivative of the Taylor series
            // i.e. 1 - (J-1) + (J-1)^2 -...
            // in other words, the sum of (1-J)^n for n=0...N-1
            // fac = 0;
            // Real _1_min_detF_n = 1;
            // for (int i = 0; i < _NUM_TAYLOR_SERIES_TERMS; i++)
            // {
            //     fac += _1_min_detF_n;
            //     _1_min_detF_n *= (1 - detF);
            // }
            fac = 1 - (detF-1) + (detF-1)*(detF-1);
            // fac = 1;
        }

        // calculation of delC wrt 1st position
        delC[0] = fac*(F_cross[0]*_Q(0,0) + F_cross[3]*_Q(0,1) + F_cross[6]*_Q(0,2));
        delC[1] = fac*(F_cross[1]*_Q(0,0) + F_cross[4]*_Q(0,1) + F_cross[7]*_Q(0,2));
        delC[2] = fac*(F_cross[2]*_Q(0,0) + F_cross[5]*_Q(0,1) + F_cross[8]*_Q(0,2));

        // calculation of delC wrt 2nd postion
        delC[3] = fac*(F_cross[0]*_Q(1,0) + F_cross[3]*_Q(1,1) + F_cross[6]*_Q(1,2));
        delC[4] = fac*(F_cross[1]*_Q(1,0) + F_cross[4]*_Q(1,1) + F_cross[7]*_Q(1,2));
        delC[5] = fac*(F_cross[2]*_Q(1,0) + F_cross[5]*_Q(1,1) + F_cross[8]*_Q(1,2));

        // calculation of delC wrt 3rd position
        delC[6] = fac*(F_cross[0]*_Q(2,0) + F_cross[3]*_Q(2,1) + F_cross[6]*_Q(2,2));
        delC[7] = fac*(F_cross[1]*_Q(2,0) + F_cross[4]*_Q(2,1) + F_cross[7]*_Q(2,2));
        delC[8] = fac*(F_cross[2]*_Q(2,0) + F_cross[5]*_Q(2,1) + F_cross[8]*_Q(2,2));

        // calculation of delC wrt 4th position
        delC[9]  = -delC[0] - delC[3] - delC[6];
        delC[10] = -delC[1] - delC[4] - delC[7];
        delC[11] = -delC[2] - delC[5] - delC[8];
    }

    /** Helper method for cross product between two 3-vectors v1 and v2, store the result in v3 */
    inline void _cross3(const Real* v1, const Real* v2, Real* v3) const
    {
        v3[0] = v1[1]*v2[2] - v1[2]*v2[1];
        v3[1] = v1[2]*v2[0] - v1[0]*v2[2];
        v3[2] = v1[0]*v2[1] - v1[1]*v2[0];
    }

    protected:
    Real _gamma;  // the ratio mu/lambda - part of C(x)
};

} // namespace Solver

#endif // __HYDROSTATIC_CONSTRAINT_HPP
