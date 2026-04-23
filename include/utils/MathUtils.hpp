#ifndef __MATH_UTILS_HPP
#define __MATH_UTILS_HPP

#include "common/types.hpp"

#include <functional>

struct MathUtils
{

/** General RK4 integration
 * @param state0 - the initial state at t[0]
 * @param t - a vector of integration points (t[0] corresponds to the time at state0)
 * @param params - additional constant parameters for the integration
 * @param ode_func - a function that, given (time, state, params) returns the time derivative of the state
 * @returns the states at each of the integration points
 */
template<typename StateType, typename ParamType>
static std::vector<StateType> RK4(const StateType& state0, const std::vector<Real>& t, const ParamType& params, std::function<StateType (Real, const StateType&, const ParamType&)> ode_func)
{
    std::vector<StateType> states(t.size());
    states[0] = state0;

    // run through each integration point
    for (unsigned i = 1; i < t.size(); i++)
    {
        const Real h = t[i] - t[i-1];
        
        // RK4 algorithm
        const StateType k1 = ode_func(t[i], states[i-1], params);
        const StateType k2 = ode_func(t[i] + 0.5*h, states[i-1] + 0.5*h*k1, params);
        const StateType k3 = ode_func(t[i] + 0.5*h, states[i-1] + 0.5*h*k2, params);
        const StateType k4 = ode_func(t[i] + h, states[i-1] + h*k3, params);

        states[i] = states[i-1] + h*(k1 + 2*k2 + 2*k3 + k4) / 6.0;
    }

    return states;

}

/** General RK4 integration - but with iterator ranges
 * @param state0 - the initial state at t_start
 * @param t_start - starting iterator of the times t - *t_start should correspond to the time at state0
 * @param t_end - ending iterator of the times t - IT IS NOT INCLUDED in t
 * @param params - additional constant parameters for the integration
 * @param ode_func - a function that, given (time, state, params) returns the time derivative of the state
 * @param state_output_start - (OUTPUT) an iterator pointing to the start of an output states vector (state0 will go here)
 */
template<typename StateType, typename ParamType, typename TIterator, typename StateIterator>
static void RK4(const StateType& state0, TIterator t_start, TIterator t_end,
    const ParamType& params, std::function<StateType (Real, const StateType&, const ParamType&)> ode_func,
    StateIterator state_output_start)
{
    int num_states = std::distance(t_start, t_end);
    std::vector<StateType> states(num_states);
    *(state_output_start) = state0;

    // run through each integration point
    for (int i = 1; i < num_states; i++)
    {
        const Real cur_t = *(t_start + i);
        const Real last_t = *(t_start + i-1);
        const Real h = cur_t - last_t;
        
        // RK4 algorithm
        const StateType k1 = ode_func(cur_t, *(state_output_start + i-1), params);
        const StateType k2 = ode_func(cur_t + 0.5*h, *(state_output_start + i-1) + 0.5*h*k1, params);
        const StateType k3 = ode_func(cur_t + 0.5*h, *(state_output_start + i-1) + 0.5*h*k2, params);
        const StateType k4 = ode_func(cur_t + h, *(state_output_start + i-1) + h*k3, params);

        *(state_output_start + i) = *(state_output_start + i-1) + h*(k1 + 2*k2 + 2*k3 + k4) / 6.0;
    }

}

static Mat3r quatToMat(const Vec4r& quat)
{
    Mat3r mat;
    mat(0,0) = 1 - 2*quat[1]*quat[1] - 2*quat[2]*quat[2];
    mat(0,1) = 2*quat[0]*quat[1] - 2*quat[3]*quat[2];
    mat(0,2) = 2*quat[0]*quat[2] + 2*quat[3]*quat[1];
    mat(1,0) = 2*quat[0]*quat[1] + 2*quat[3]*quat[2];
    mat(1,1) = 1 - 2*quat[0]*quat[0] - 2*quat[2]*quat[2];
    mat(1,2) = 2*quat[1]*quat[2] - 2*quat[3]*quat[0];
    mat(2,0) = 2*quat[0]*quat[2] - 2*quat[3]*quat[1];
    mat(2,1) = 2*quat[1]*quat[2] + 2*quat[3]*quat[0];
    mat(2,2) = 1 - 2*quat[0]*quat[0] - 2*quat[1]*quat[1];

    return mat;
}

static Vec4r matToQuat(const Mat3r& mat)
{
    Vec4r q;

    // code adapted from https://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToQuaternion/
    const Real trace = mat.trace();
    if( trace > 0 )
    {
        Real s = Real(0.5) / std::sqrt(trace + 1);
        q[3] = Real(0.25) / s;
        q[0] = ( mat(2,1) - mat(1,2) ) * s;
        q[1] = ( mat(0,2) - mat(2,0) ) * s;
        q[2] = ( mat(1,0) - mat(0,1) ) * s;
    } 
    else
    {
        if ( mat(0,0) > mat(1,1) && mat(0,0) > mat(2,2) ) {
            Real s = 2 * std::sqrt( 1 + mat(0,0) - mat(1,1) - mat(2,2));
            q[3] = (mat(2,1) - mat(1,2) ) / s;
            q[0] = Real(0.25) * s;
            q[1] = (mat(0,1) + mat(1,0) ) / s;
            q[2] = (mat(0,2) + mat(2,0) ) / s;
        } else if (mat(1,1) > mat(2,2)) {
            Real s = 2 * std::sqrt( 1 + mat(1,1) - mat(0,0) - mat(2,2));
            q[3] = (mat(0,2) - mat(2,0) ) / s;
            q[0] = (mat(0,1) + mat(1,0) ) / s;
            q[1] = 0.25f * s;
            q[2] = (mat(1,2) + mat(2,1) ) / s;
        } else {
            Real s = 2 * std::sqrt( 1 + mat(2,2) - mat(0,0) - mat(1,1) );
            q[3] = (mat(1,0) - mat(0,1) ) / s;
            q[0] = (mat(0,2) + mat(2,0) ) / s;
            q[1] = (mat(1,2) + mat(2,1) ) / s;
            q[2] = 0.25f * s;
        }
    }
    
    return q;
}

static Mat3r Skew3(const Vec3r& vec)
{
    Mat3r mat;
    mat << 0,       -vec(2),    vec(1),
           vec(2),  0,          -vec(0),
           -vec(1), vec(0),     0;
    return mat;
}

static Vec3r Vee3(const Mat3r& mat)
{
    return Vec3r(mat(2,1), mat(0,2), mat(1,0));
}

static Mat3r Exp_so3(const Vec3r& vec)
{
    const Mat3r skew = Skew3(vec);
    Real mag = vec.norm();

    if (mag < Real(1e-8))
        return Mat3r::Identity() + skew;
    
    return Mat3r::Identity() + std::sin(mag) / mag * skew + (1 - std::cos(mag)) / (mag * mag) * skew * skew;
}

static Vec3r Log_SO3(const Mat3r& mat)
{
    // std::cout << "\n===Log_SO3===" << std::endl;
    // std::cout << "  mat:\n" << mat << std::endl;
    // std::cout << "  mat.trace()-3: " << mat.trace()-3 << std::endl;
    Real theta = std::acos( std::min(0.5 * mat.trace() - 0.5, Real(1.0)));  // make sure 1/2 tr(mat) - 1/2 is not >1, will get NaNs. This may happen due to numerical drift
    // std::cout << "  theta: " << theta << std::endl;

    if (std::abs(theta) < Real(1e-14))
    {
        return Vec3r::Zero();
    }

    const Vec3r skew_vec3 = Vee3(mat - mat.transpose());
    // std::cout << "  skew_vec3: " << skew_vec3 << std::endl;

    if (std::abs(mat.trace()) < Real(1e-8))
    {
        return 0.5 * (1 + theta*theta/6.0 + 7*theta*theta*theta*theta/360.0) * skew_vec3;
    }

    // std::cout << " 2*std::sin(theta): " << 2*std::sin(theta) << std::endl;
    return theta / ( 2*std::sin(theta)) * skew_vec3;
}

static Vec3r Minus_SO3(const Mat3r& mat1, const Mat3r& mat2)
{
    return Log_SO3(mat2.transpose() * mat1);
}

};

#endif // __MATH_UTILS_HPP