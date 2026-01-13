#ifndef __XPBD_TYPEDEFS_HPP
#define __XPBD_TYPEDEFS_HPP

#include "solver/xpbd_projector/ConstraintProjector.hpp"
#include "solver/xpbd_projector/RigidBodyConstraintProjector.hpp"
#include "solver/xpbd_projector/CombinedConstraintProjector.hpp"
#include "solver/xpbd_projector/CombinedNeohookeanConstraintProjector.hpp"

#include "solver/constraint/HydrostaticConstraint.hpp"
#include "solver/constraint/DeviatoricConstraint.hpp"
#include "solver/constraint/StaticDeformableCollisionConstraint.hpp"
#include "solver/constraint/RigidDeformableCollisionConstraint.hpp"
#include "solver/constraint/DeformableDeformableCollisionConstraint.hpp"
#include "solver/constraint/AttachmentConstraint.hpp"
#include "solver/constraint/MidpointConstraint.hpp"

#include "solver/xpbd_solver/XPBDGaussSeidelSolver.hpp"
#include "solver/xpbd_solver/XPBDJacobiSolver.hpp"
#include "solver/xpbd_solver/XPBDParallelJacobiSolver.hpp"

#include "common/TypeList.hpp"

#include <variant>
#include <type_traits>

template<typename ...Projectors>
struct XPBDMeshObjectConstraintConfiguration
{
    using projector_type_list = TypeList<Projectors...>;
    using constraint_type_list = typename ConcatenateTypeLists<typename Projectors::constraint_type_list...>::type;

    // two ConstraintTypes are equal if their Projector types are equal
    template<typename ...OtherProjectors>
    bool operator==(const XPBDMeshObjectConstraintConfiguration<OtherProjectors...>&) const
    {
        return std::is_same_v<TypeList<Projectors...>, TypeList<OtherProjectors...>>;
    }
};

// Declare XPBDMeshObject constraint configurations
template <bool IsFirstOrder>
struct XPBDMeshObjectConstraintConfigurations
{
    // typedefs for readability
    private:
    using DevProjector = Solver::ConstraintProjector<IsFirstOrder, Solver::DeviatoricConstraint>;
    using HydProjector = Solver::ConstraintProjector<IsFirstOrder, Solver::HydrostaticConstraint>;
    using DevHydProjector = Solver::CombinedConstraintProjector<IsFirstOrder, Solver::DeviatoricConstraint, Solver::HydrostaticConstraint>;
    using StatCollProjector = Solver::ConstraintProjector<IsFirstOrder, Solver::StaticDeformableCollisionConstraint>;
    using DefCollProjector = Solver::ConstraintProjector<IsFirstOrder, Solver::DeformableDeformableCollisionConstraint>;
    using RigiCollProjector = Solver::RigidBodyConstraintProjector<IsFirstOrder, Solver::RigidDeformableCollisionConstraint>;
    using AttProjector = Solver::ConstraintProjector<IsFirstOrder, Solver::AttachmentConstraint>;
    using MidProjector = Solver::ConstraintProjector<IsFirstOrder, Solver::MidpointConstraint>;

    // public typedefs represent XPBDMeshObject constraint configurations
    public:
    using StableNeohookean = XPBDMeshObjectConstraintConfiguration<DevProjector, HydProjector, MidProjector, StatCollProjector, DefCollProjector, RigiCollProjector, AttProjector>;
    using StableNeohookeanCombined = XPBDMeshObjectConstraintConfiguration<DevHydProjector, MidProjector, StatCollProjector, DefCollProjector, RigiCollProjector, AttProjector>;

    using type_list = TypeList<StableNeohookean, StableNeohookeanCombined>;
    using variant_type = std::variant<StableNeohookean, StableNeohookeanCombined>;

    constexpr static StableNeohookean STABLE_NEOHOOKEAN{};
    constexpr static StableNeohookeanCombined STABLE_NEOHOOKEAN_COMBINED{};
};

// Declare XPBDMeshObject constraint configurations
// struct FirstOrderXPBDMeshObjectConstraintConfigurations
// {
//     // typedefs for readability
//     private:
//     using DevProjector = Solver::ConstraintProjector<true, Solver::DeviatoricConstraint>;
//     using HydProjector = Solver::ConstraintProjector<true, Solver::HydrostaticConstraint>;
//     using DevHydProjector = Solver::CombinedConstraintProjector<true, Solver::DeviatoricConstraint, Solver::HydrostaticConstraint>;
//     using StatCollProjector = Solver::ConstraintProjector<true, Solver::StaticDeformableCollisionConstraint>;
//     using RigiCollProjector = Solver::RigidBodyConstraintProjector<true, Solver::RigidDeformableCollisionConstraint>;
//     using AttProjector = Solver::ConstraintProjector<true, Solver::AttachmentConstraint>;

//     // public typedefs represent XPBDMeshObject constraint configurations
//     public:
//     using StableNeohookean = XPBDMeshObjectConstraintConfiguration<DevProjector, HydProjector, StatCollProjector, RigiCollProjector, AttProjector>;
//     using StableNeohookeanCombined = XPBDMeshObjectConstraintConfiguration<DevHydProjector, StatCollProjector, RigiCollProjector, AttProjector>;

//     using type_list = TypeList<StableNeohookean, StableNeohookeanCombined>;
//     using variant_type = std::variant<StableNeohookean, StableNeohookeanCombined>;

//     constexpr static StableNeohookean STABLE_NEOHOOKEAN{};
//     constexpr static StableNeohookeanCombined STABLE_NEOHOOKEAN_COMBINED{};
// };

template<bool IsFirstOrder, typename ProjectorTypeList> struct XPBDObjectSolverTypes;

template<bool IsFirstOrder, typename ...Projectors>
struct XPBDObjectSolverTypes<IsFirstOrder, TypeList<Projectors...>>
{
    // check and make sure all projector types are NOT 1st order projectors
    static_assert( ( (Projectors::is_first_order == IsFirstOrder) && ...) );
    using GaussSeidel = Solver::XPBDGaussSeidelSolver<IsFirstOrder, Projectors...>;
    using Jacobi = Solver::XPBDJacobiSolver<IsFirstOrder, Projectors...>;
    using ParallelJacobi = Solver::XPBDParallelJacobiSolver<IsFirstOrder, Projectors...>;

    using type_list = TypeList<GaussSeidel, Jacobi, ParallelJacobi>;
    using variant_type = std::variant<GaussSeidel, Jacobi, ParallelJacobi>;
};

// template<typename ProjectorTypeList> struct FirstOrderXPBDObjectSolverTypes;

// template<typename ...Projectors>
// struct FirstOrderXPBDObjectSolverTypes<TypeList<Projectors...>>
// {
//     // check and make sure all projector types are 1st order projectors
//     static_assert( (Projectors::is_first_order && ...) );
//     using GaussSeidel = Solver::XPBDGaussSeidelSolver<true, Projectors...>;
//     using Jacobi = Solver::XPBDJacobiSolver<true, Projectors...>;
//     using ParallelJacobi = Solver::XPBDParallelJacobiSolver<true, Projectors...>;

//     using type_list = TypeList<GaussSeidel, Jacobi, ParallelJacobi>;
//     using variant_type = std::variant<GaussSeidel, Jacobi, ParallelJacobi>;
// };



// Declare XPBDMeshObject solver types



#endif // __XPBD_TYPEDEFS_HPP