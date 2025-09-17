#ifndef __XPBD_SOLVER_HPP
#define __XPBD_SOLVER_HPP

#include "simobject/XPBDMeshObjectBase.hpp"
#include "common/XPBDEnumTypes.hpp"

#include "solver/constraint/Constraint.hpp"
#include "solver/constraint/ConstraintReference.hpp"
#include "solver/xpbd_projector/ConstraintProjector.hpp"
#include "solver/xpbd_projector/CombinedConstraintProjector.hpp"
#include "solver/xpbd_projector/ConstraintProjectorReference.hpp"
#include "solver/xpbd_projector/ConstraintProjectorTraits.hpp"
#include "solver/xpbd_solver/XPBDSolverUpdates.hpp"

#include "common/TypeList.hpp"
#include "common/VariadicVectorContainer.hpp"

namespace Solver
{

/** XPBD Solver base class responsible for iterating over constraint projectors and applying their position updates.
 * Different derived classes will have different ways of doing this:
 *   i.e. Gauss Seidel solver will apply position updates immediately, Jacobi solver will accumulate position updates and apply them after iterating through all constraints
 * 
 * IsFirstOrder template parameter indicates whether the 1st-Order XPBD method is being used.
 * 
 * ConstraintProjectors variadic template parameter lists all the types of constraint projectors this solver can project.
 *   Note that this must be known at compile time - see XPBDTypedefs.hpp for grouped lists of constraint projectors
 * 
 * 
 */
template<bool IsFirstOrder, typename ...ConstraintProjectors>
class XPBDSolver
{
    public:
    /** List of projector types */
    using projector_type_list = TypeList<ConstraintProjectors...>;
    /** List of constraint types */
    using constraint_type_list = typename ConcatenateTypeLists<typename ConstraintProjectors::constraint_type_list...>::type;
    /** Container for projectors */
    using projector_container_type = VariadicVectorContainer<ConstraintProjectors...>;
    /** Container for projector references */
    using projector_reference_container_type = VariadicVectorContainer<ConstraintProjectorReference<ConstraintProjectors>...>;
    /** Whether or not solver is using 1st-Order projection */
    constexpr static bool is_first_order = IsFirstOrder;

    explicit XPBDSolver(Sim::XPBDMeshObject_Base_<IsFirstOrder>* obj, int num_iter, XPBDSolverResidualPolicyEnum residual_policy)
        : _obj(obj), _num_iter(num_iter), _residual_policy(residual_policy), _constraints_using_primary_residual(false), _num_constraints(0)
    {
        _rigid_body_updates.resize(14); // 14 doubles is enough to store 2 rigid body updates (no more than 2 rigid bodies will be involved in a single constraint projection)
    }

    ~XPBDSolver() = default;

    /** Performs any one-time setup for the solver. */
    void setup()
    {
        // resize the primary residual vector according to the MeshObject's number of vertices
        // TODO: should there be a generalized numCoordinates() method for XPBDObjects?
        _primary_residual.resize(3*_obj->mesh()->numVertices());
    }   

    /** Project the constraints and apply the position updates.
     * Generally consists of constraint initialization, a solver loop that will run num_iter times, and optional residual calculation.
     */
    void solve()
    {
        /** TODO: uncomment this when ready */
        // _inertial_positions = _obj->mesh()->vertices();

        // initialize all the constraints
        _constraint_projectors.for_each_element([&](auto& proj)
        {
            if (proj.isValid())
            {
                proj.initialize();
            }
        });

        for (int i = 0; i < _num_iter; i++)
        {
            // if any of the constraint proectors require the primary residual, we need to calculate it
            if (_constraints_using_primary_residual)
            {
                _calculatePrimaryResidual();
            }

            // iterate through the constraints and solve them
            _iterateConstraints();
        }

        // calculate the residual after this step if required by the residual policy
        if (_residual_policy == XPBDSolverResidualPolicyEnum::EVERY_SUBSTEP)
        {
            _calculatePrimaryResidual();
            _calculateConstraintResidual();
        }
    }

    /** Project the constraints and apply the position updates.
     * This will only project the constraints associated with the projectors passed in with the project_references parameter.
     * This is useful for only projecting a subset of the overall constraints, such as when we want to do local iterations for collision constraints.
     * @param projector_references - a heterogenous container of ConstraintProjectorReferences which are the constraints to project
     * @param num_iter - the number of times to go through the constraints
     * @param initialize - whether or not to initialize the projectors
     */
    void solve(projector_reference_container_type& projector_references, int num_iter=1, bool initialize=false)
    {
        // if initializing, initialize the projectors
        if (initialize)
        {
            projector_references.for_each_element([&](auto& proj_ref)
            {
                if (proj_ref->isValid())
                {
                    proj_ref->initialize();
                }
            });
        }

        // iterate repeatedly through the specified constraint projectors
        for (int i = 0; i < num_iter; i++)
        {
            _iterateConstraints(projector_references);
        }
    }

    /** Returns the current primary residual (Equation (8) from XPBD paper).
     * Does NOT recalculate if stale.
     */
    const VecXr primaryResidual() const { return Eigen::Map<const VecXr>(_primary_residual.data(), _primary_residual.size()); };

    /** Returns the current constraint residual (Equation (9) from XPBD paper).
     * Does NOT recalculate if stale.
     */
    const VecXr constraintResidual() const { return Eigen::Map<const VecXr>(_constraint_residual.data(), _constraint_residual.size()); };

    /** Returns the number of solver iterations. */
    int numIterations() const { return _num_iter; }

    /** Allocate space for a specified number of constraint projectors of a specified type.
     * @param size - the number of constraint projectors to allocate space for
    */
    template<class ProjectorType>
    void setNumProjectorsOfType(int size)
    {
        _constraint_projectors.template get<ProjectorType>().resize(size);
        
    }

    /** Creates a projector for the passed in constraints and adds it to the associated projector array.
     * @param dt - the time step used by the constraint projector (i.e. the simulation time step)
     * @param constraints - variadic list of constraint pointers (right now, only 1 or 2 constraints are supported)
     * @returns a reference to the added projector as a ConstraintProjectorReference
     */
    template<class... Constraints>
    ConstraintProjectorReference<typename ConstraintProjectorTraits<IsFirstOrder, Constraints...>::type> 
    addConstraintProjector(Real dt, ConstraintReference<Constraints>&&... constraints)
    {
        using ProjectorType = typename ConstraintProjectorTraits<IsFirstOrder, Constraints...>::type;
        ProjectorType projector(dt, std::forward<ConstraintReference<Constraints>>(constraints)...);
    
        // make sure that the data buffer of the coordinate updates vector is large enough to accommodate the new projector
        if (static_cast<unsigned>(projector.numCoordinates()) > _coordinate_updates.size())
        {
            _coordinate_updates.resize(projector.numCoordinates());
        }

        // make sure that the data buffer of the rigid body updates vector is large eneough to accommodate the new projector
        if (static_cast<unsigned>(ProjectorType::NUM_RIGID_BODIES) > _rigid_body_updates.size())
        {
            _rigid_body_updates.resize(ProjectorType::NUM_RIGID_BODIES);
        }
    
        // increase the total number of constraints (needed for the constraint residual size)
        _num_constraints += ProjectorType::NUM_CONSTRAINTS;
            
        _constraint_projectors.template push_back<ProjectorType>(std::move(projector));

        std::vector<ProjectorType>& proj_vec = _constraint_projectors.template get<ProjectorType>();
        return ConstraintProjectorReference<ProjectorType>(proj_vec, proj_vec.size()-1);
    }

    /** Creates a projector for the passed in constraints and adds it to the associated projector array.
     * @param dt - the time step used by the constraint projector (i.e. the simulation time step)
     * @param B_e_inv - inverse damping matrix for the element
     * @param constraints - variadic list of constraint pointers (right now, only 1 or 2 constraints are supported)
     * @returns a reference to the added projector as a ConstraintProjectorReference
     */
    template<class... Constraints>
    ConstraintProjectorReference<typename ConstraintProjectorTraits<IsFirstOrder, Constraints...>::type> 
    addConstraintProjector(Real dt, const Eigen::Matrix<Real,12,12>& B_e_inv, ConstraintReference<Constraints>&&... constraints)
    {
        using ProjectorType = typename ConstraintProjectorTraits<IsFirstOrder, Constraints...>::type;
        ProjectorType projector(dt, std::forward<ConstraintReference<Constraints>>(constraints)..., B_e_inv);
    
        // make sure that the data buffer of the coordinate updates vector is large enough to accommodate the new projector
        if (static_cast<unsigned>(projector.numCoordinates()) > _coordinate_updates.size())
        {
            _coordinate_updates.resize(projector.numCoordinates());
        }

        // make sure that the data buffer of the rigid body updates vector is large eneough to accommodate the new projector
        if (static_cast<unsigned>(ProjectorType::NUM_RIGID_BODIES) > _rigid_body_updates.size())
        {
            _rigid_body_updates.resize(ProjectorType::NUM_RIGID_BODIES);
        }
    
        // increase the total number of constraints (needed for the constraint residual size)
        _num_constraints += ProjectorType::NUM_CONSTRAINTS;
            
        _constraint_projectors.template push_back<ProjectorType>(std::move(projector));

        std::vector<ProjectorType>& proj_vec = _constraint_projectors.template get<ProjectorType>();
        return ConstraintProjectorReference<ProjectorType>(proj_vec, proj_vec.size()-1);
    }

    /** Creates a projector for the passed in constraints and puts it at the specified index.
     * @param index - the index of the new projector in the associated projector array
     * @param dt - the time step used by the constraint projector (i.e. the simulation time step)
     * @param constraints - variadic list of constraint pointers (right now, only 1 or 2 constraints are supported)
     */
    template<class... Constraints>
    ConstraintProjectorReference<typename ConstraintProjectorTraits<IsFirstOrder, Constraints...>::type>
    setConstraintProjector(int index, Real dt, Constraints* ... constraints)
    {
        using ProjectorType = typename ConstraintProjectorTraits<IsFirstOrder, Constraints...>::type;
        ProjectorType projector(dt, constraints...);
            
        // if current constraint projector at this index is invalid, increase total number of constraints
        if (!_constraint_projectors.template get<ProjectorType>()[index].isValid())
            _num_constraints += ProjectorType::NUM_CONSTRAINTS;
        
        // set constraint projector at index
        _constraint_projectors.template set<ProjectorType>(index, std::move(projector));

        std::vector<ProjectorType>& proj_vec = _constraint_projectors.template get<ProjectorType>();
        return ConstraintProjectorReference<ProjectorType>(proj_vec, index);
    }

    /** Returns the projector at the specified index. */
    template<class Projector>
    const Projector& getConstraintProjector(int index) const
    {
        return _constraint_projectors.template get<Projector>()[index];
    }

    /** Returns all projectors of a specified type. */
    template<class Projector>
    const std::vector<Projector>& getConstraintProjectorsOfType() const
    {
        return _constraint_projectors.template get<Projector>();
    }

    /** Returns all projectors of a specified type. */
    template<class Projector>
    std::vector<Projector>& getConstraintProjectorsOfType()
    {
        return _constraint_projectors.template get<Projector>();
    }

    /** Sets the validity of a single projector.
     * @param index - the index of the projector
     * @param is_valid - whether the proejctor should be marked valid or invalid
     */
    template<class Projector>
    void setProjectorValidity(int index, bool is_valid)
    {
        Projector& projector = _constraint_projectors.template get<Projector>()[index];

        // decrease the total number of constraints if projector is valid
        if (projector.isValid() && !is_valid)
            _num_constraints -= Projector::NUM_CONSTRAINTS;
        else if (!projector.isValid() && is_valid)
            _num_constraints += Projector::NUM_CONSTRAINTS;

        // set validity of constraint projector
        projector.setValidity(is_valid);
    }

    /** Marks all projectors of a given type as invalid. */
    template<class Projector>
    void setAllProjectorsOfTypeInvalid()
    {
        _constraint_projectors.template for_each_element<Projector>([&](auto& element)
        {
            // decrease the total number of constraints if projector is valid
            if (element.isValid())
                _num_constraints -= Projector::NUM_CONSTRAINTS;

            element.setValidity(false);
        });
    }

    /** Clears all projectors of a given type. */
    template<class Projector>
    void clearProjectorsOfType()
    {
        _constraint_projectors.template for_each_element<Projector>([&](auto& element)
        {
            // decrease the total number of constraints if projector is valid
            if (element.isValid())
                _num_constraints -= Projector::NUM_CONSTRAINTS;
        });

        _constraint_projectors.template clear<Projector>();
    }

    protected:

    /** Helper function that will perform 1 iteration of the solver. 
     * This method is pure virtual because its implementation depends on the solver type (Gauss-Seidel, Jacobi, etc.) to know what to do with the position updates given by the ConstraintProjectors.
     */
    virtual void _iterateConstraints() = 0;

    /** Helper function that will perform 1 iteration of the solver for the specified list of projectors.
     * This method is pure virtual because its implementation depends on the solver type (Gauss-Seidel, Jacobi, etc.) to know what to do with the position updates given by the ConstraintProjectors.
     */
    virtual void _iterateConstraints(projector_reference_container_type& projector_references) = 0;
    
    /** Calculates the primary residual (Equation (8) from XPBD paper). */
    // TODO: Actually implement this
    void _calculatePrimaryResidual() {}

    /** Calculates the constraint residual (Equation (9) from XPBD paper).
     * This should be called AFTER solve() so that the constraint projectors have nonzero lambdas.
     */
    void _calculateConstraintResidual()
    {
        // resize constraint residual to the number of constraints
        // due to inequality constraints being ignored, it may not be this large, but at least we guarantee we have enough space
        _constraint_residual.resize(_num_constraints);

        int constraint_index = 0;
        this->_constraint_projectors.for_each_element([&](auto& proj)
        {
            if (!proj.isValid())
                return;

            using ProjectorType = std::remove_reference_t<decltype(proj)>;
            // CombinedConstraintProjector case
            if constexpr (ProjectorType::NUM_CONSTRAINTS == 2)
            {
                const auto& constraint1 = proj.constraint1();
                const auto& constraint2 = proj.constraint2();
                Real C1, C2;
                constraint1->evaluate(&C1);
                constraint2->evaluate(&C2);
                Real alpha1 = constraint1->alpha();
                Real alpha2 = constraint2->alpha();
                Vec2r lambda = proj.lambda();

                if constexpr (IsFirstOrder)
                {
                    _constraint_residual[constraint_index++] = C1 + alpha1/proj.dt() * lambda[0];
                    _constraint_residual[constraint_index++] = C2 + alpha2/proj.dt() * lambda[1];
                }
                else
                {
                    _constraint_residual[constraint_index++] = C1 + alpha1/(proj.dt()*proj.dt()) * lambda[0];
                    _constraint_residual[constraint_index++] = C2 + alpha2/(proj.dt()*proj.dt()) * lambda[1];
                }
                
            }

            // single constraint projector case
            else
            {
                const auto& constraint = proj.constraint();

                // skip inequality constraints (for now)
                if (constraint->isInequality())
                    return;
                
                Real C;
                constraint->evaluate(&C);
                Real alpha = constraint->alpha();
                Real lambda = proj.lambda();

                if constexpr (IsFirstOrder)
                    _constraint_residual[constraint_index++] = C + alpha/proj.dt() * lambda;
                else
                    _constraint_residual[constraint_index++] = C + alpha/(proj.dt()*proj.dt()) * lambda;
            }

        });

        // resize the constraint residual down to the actual number of constraints included in the residual calculation
        // this should just truncate the junk indices in the vector that we didn't fill out
        _constraint_residual.resize(constraint_index);
        
    }

    protected:
    VariadicVectorContainer<ConstraintProjectors...> _constraint_projectors;    // stores the constraint projectors of different types

    Sim::XPBDMeshObject_Base_<IsFirstOrder>* _obj;                                 // pointer to the XPBDMeshObject that owns this Solver and is updated by the solver loop
    int _num_iter;                                         // number of solver iterations per solve() call
    Geometry::Mesh::vertices_vec_type _inertial_positions;                // stores the positions after the inertial update - useful for primary residual calculation

    XPBDSolverResidualPolicyEnum _residual_policy;                        // determines how often to calculate the residuals - this varies depending on the information needs of the user

    bool _constraints_using_primary_residual;                   // whether or not any of the ConstraintProjectors require the primary residual to do the position update

    int _num_constraints;                                  // total number of constraints projected (note this may be different from number of ConstraintProjectors if some constraints are solved simultaneously)

    std::vector<Real> _primary_residual;                      // primary residual vector
    std::vector<Real> _constraint_residual;                   // constraint residual vector - use std::vector instead of VecXr to minimize dynamic reallocations as number of constraints change

    std::vector<CoordinateUpdate> _coordinate_updates;                    // stores updates to the coordinates determined by the constraint projections - also pre-allocated. Needs to be have a size >= the max number of positions affected by a single constraint projection.
    std::vector<RigidBodyUpdate> _rigid_body_updates;                    // stores updates to any rigid bodies involved in the constraint projections - also pre-allocated. Each rigid body update consists of a position update and an orientation update, which is 7 doubles.
};

} // namespace Solver

#endif // __XPBD_SOLVER_HPP