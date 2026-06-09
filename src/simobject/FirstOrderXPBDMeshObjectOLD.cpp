#include "simobject/FirstOrderXPBDMeshObject.hpp"
#include "simulation/Simulation.hpp"

#include "geometry/DeformableMeshSDF.hpp"

namespace Sim
{

template<typename SolverType, typename... ConstraintTypes>
FirstOrderXPBDMeshObject<SolverType, TypeList<ConstraintTypes...>>::FirstOrderXPBDMeshObject(const Simulation* sim, const ConfigType* config)
    : XPBDMeshObject<SolverType, TypeList<ConstraintTypes...>>(sim, config)
{
    std::cout << "FirstOrderXPBDMeshObject constructor! " << std::endl;
    _damping_multiplier = config->dampingMultiplier();

    std::cout << "Damping Multiplier: " << _damping_multiplier << std::endl;
}

template<typename SolverType, typename... ConstraintTypes>
void FirstOrderXPBDMeshObject<SolverType, TypeList<ConstraintTypes...>>::_calculatePerVertexQuantities()
{
    XPBDMeshObject<SolverType, TypeList<ConstraintTypes...>>::_calculatePerVertexQuantities();

    _inv_B.resize(this->_mesh->numVertices());
    for (int i = 0; i < this->_mesh->numVertices(); i++)
    {
        _inv_B[i] = 1.0 / (this->_vertex_volumes[i] * _damping_multiplier);
    }
}

template<typename SolverType, typename... ConstraintTypes>
void FirstOrderXPBDMeshObject<SolverType, TypeList<ConstraintTypes...>>::setup()
{
    // TODO: lots of duplicated code here...

    this->loadAndConfigureMesh();

    this->_solver.setup();

    // set size of collision constraint projector vectors
    using StaticCollisionConstraintType = Solver::ConstraintProjector<SolverType::is_first_order, Solver::StaticDeformableCollisionConstraint>;
    using RigidCollisionConstraintType = Solver::RigidBodyConstraintProjector<SolverType::is_first_order, Solver::RigidDeformableCollisionConstraint>;
    // this->_solver.template setNumProjectorsOfType<StaticCollisionConstraintType>(4*this->_mesh->numFaces() + this->_mesh->numVertices());
    this->_solver.template setNumProjectorsOfType<RigidCollisionConstraintType>(this->_mesh->numFaces());

    // initialize the previous vertices matrix once we've loaded the mesh
    this->_previous_vertices = this->_mesh->vertices();
    this->_vertex_velocities = Geometry::Mesh::VerticesMat::Zero(3, this->_mesh->numVertices());
    this->_vertices_in_collision.resize(this->_mesh->numVertices(), false);

    _calculatePerVertexQuantities();
    // _createSolver(_solver_type, _num_solver_iters, _residual_policy);       // create the Solver object first and then add ConstraintProjectors to it
    this->_createElasticConstraints();     // create constraints and add ConstraintProjectors to the solver object
}

template<typename SolverType, typename... ConstraintTypes>
std::string FirstOrderXPBDMeshObject<SolverType, TypeList<ConstraintTypes...>>::toString(const int indent) const
{
    // TODO: better toString
    return XPBDMeshObject<SolverType, TypeList<ConstraintTypes...>>::toString(indent);
}

template<typename SolverType, typename... ConstraintTypes>
void FirstOrderXPBDMeshObject<SolverType, TypeList<ConstraintTypes...>>::_createElasticConstraints()
{
    // TODO: think about this... we need to resize each vector initially so that pointers to constraints are still valid...
    // alternative: use vector unique_ptr<Constraint> 
    // another TODO: this code is duplicated from XPBDMeshObject - really probably shouldn't have it in two places
    this->_constraints.template reserve<Solver::HydrostaticConstraint>(this->tetMesh()->numElements());
    this->_constraints.template reserve<Solver::DeviatoricConstraint>(this->tetMesh()->numElements());
    this->_constraints.template reserve<Solver::StaticDeformableCollisionConstraint>(1000*this->_mesh->numFaces());
    this->_constraints.template reserve<Solver::RigidDeformableCollisionConstraint>(this->_mesh->numFaces());
    this->_constraints.template reserve<Solver::OffsetAttachmentConstraint>(this->_mesh->numVertices());

    // create constraint(s) for each element
    for (int i = 0; i < this->tetMesh()->numElements(); i++)
    {
        const Eigen::Vector4i element = this->tetMesh()->element(i);
        const int v0 = element[0];
        const int v1 = element[1];
        const int v2 = element[2];
        const int v3 = element[3];

        Real* v0_ptr = this->_mesh->vertexPointer(v0);
        Real* v1_ptr = this->_mesh->vertexPointer(v1);
        Real* v2_ptr = this->_mesh->vertexPointer(v2);
        Real* v3_ptr = this->_mesh->vertexPointer(v3);

        Real m0 = 1/_inv_B[v0];
        Real m1 = 1/_inv_B[v1];
        Real m2 = 1/_inv_B[v2];
        Real m3 = 1/_inv_B[v3];
        // if (std::holds_alternative<XPBDMeshObjectConstraintConfigurations::StableNeohookean>(_constraint_type))
        if constexpr (std::is_same_v<typename SolverType::projector_type_list, FirstOrderXPBDMeshObjectConstraintConfigurations::StableNeohookean::projector_type_list>)
        {
            Solver::HydrostaticConstraint& hyd_constraint = 
                this->_constraints.template emplace_back<Solver::HydrostaticConstraint>(v0, v0_ptr, m0, v1, v1_ptr, m1, v2, v2_ptr, m2, v3, v3_ptr, m3, this->_material);
            Solver::DeviatoricConstraint& dev_constraint = 
                this->_constraints.template emplace_back<Solver::DeviatoricConstraint>(v0, v0_ptr, m0, v1, v1_ptr, m1, v2, v2_ptr, m2, v3, v3_ptr, m3, this->_material);
            
            // TODO: support separate constraints - maybe though SeparateConstraintProjector class?.
            this->_solver.addConstraintProjector(this->_sim->dt(), &hyd_constraint);
            this->_solver.addConstraintProjector(this->_sim->dt(), &dev_constraint);
            // _solver.addConstraintProjector(_sim->dt(), projector_options, &dev_constraint, &hyd_constraint);
            
        }
        // else if (std::holds_alternative<XPBDMeshObjectConstraintConfigurations::StableNeohookeanCombined>(_constraint_type))
        if constexpr (std::is_same_v<typename SolverType::projector_type_list, FirstOrderXPBDMeshObjectConstraintConfigurations::StableNeohookeanCombined::projector_type_list>)
        {
            Solver::HydrostaticConstraint& hyd_constraint = 
                this->_constraints.template emplace_back<Solver::HydrostaticConstraint>(v0, v0_ptr, m0, v1, v1_ptr, m1, v2, v2_ptr, m2, v3, v3_ptr, m3, this->_material);
            Solver::DeviatoricConstraint& dev_constraint = 
                this->_constraints.template emplace_back<Solver::DeviatoricConstraint>(v0, v0_ptr, m0, v1, v1_ptr, m1, v2, v2_ptr, m2, v3, v3_ptr, m3, this->_material);

            this->_solver.addConstraintProjector(this->_sim->dt(), &dev_constraint, &hyd_constraint);
        }
    }
}

template<typename SolverType, typename... ConstraintTypes>
void FirstOrderXPBDMeshObject<SolverType, TypeList<ConstraintTypes...>>::addStaticCollisionConstraint(const Geometry::SDF* sdf, const Vec3r& p, const Vec3r& n,
                                    int face_ind, const Real u, const Real v, const Real w)
{
    const Eigen::Vector3i face = this->_mesh->face(face_ind);
    int v1 = face[0];
    int v2 = face[1];
    int v3 = face[2];

    _vertices_in_collision[v1] = true;
    _vertices_in_collision[v2] = true;
    _vertices_in_collision[v3] = true;

    Real* v1_ptr = this->_mesh->vertexPointer(v1);
    Real* v2_ptr = this->_mesh->vertexPointer(v2);
    Real* v3_ptr = this->_mesh->vertexPointer(v3);

    Real m1 = 1/_inv_B[v1];
    Real m2 = 1/_inv_B[v2];
    Real m3 = 1/_inv_B[v3];

    Solver::StaticDeformableCollisionConstraint& collision_constraint = 
        this->_constraints.template emplace_back<Solver::StaticDeformableCollisionConstraint>(sdf, p, n, v1, v1_ptr, m1, v2, v2_ptr, m2, v3, v3_ptr, m3, u, v, w);

    this->_solver.addConstraintProjector(this->_sim->dt(), &collision_constraint);
    // this->_solver.setConstraintProjector(face_ind, this->_sim->dt(), &collision_constraint); // TODO: accomodate for first-order method

    // XPBDCollisionConstraint xpbd_collision_constraint;
    // xpbd_collision_constraint.constraint = std::move(collision_constraint);
    // xpbd_collision_constraint.projector_index = index;
    // xpbd_collision_constraint.num_steps_unused = 0;

    // _collision_constraints.push_back(std::move(xpbd_collision_constraint));
}

template<typename SolverType, typename... ConstraintTypes>
void FirstOrderXPBDMeshObject<SolverType, TypeList<ConstraintTypes...>>::addVertexStaticCollisionConstraint(const Geometry::SDF* sdf, const Vec3r& p, const Vec3r& n, int vert_ind)
{
    
    Real* v_ptr = this->_mesh->vertexPointer(vert_ind);

    Real m = 1 / _inv_B[vert_ind];

    Solver::StaticDeformableCollisionConstraint& collision_constraint = 
        this->_constraints.template emplace_back<Solver::StaticDeformableCollisionConstraint>(sdf, p, n, vert_ind, v_ptr, m, vert_ind, v_ptr, m, vert_ind, v_ptr, m, 1, 0, 0);

    this->_solver.addConstraintProjector(this->_sim->dt(), &collision_constraint);
    // this->_solver.setConstraintProjector(this->_mesh->numFaces() + vert_ind, this->_sim->dt(), &collision_constraint);

    // std::cout << "addVertexStaticCollisionConstraint...vertex = " << vert_ind << std::endl;
}

template<typename SolverType, typename... ConstraintTypes>
void FirstOrderXPBDMeshObject<SolverType, TypeList<ConstraintTypes...>>::addRigidDeformableCollisionConstraint(const Geometry::SDF* sdf, Sim::RigidObject* rigid_obj, const Vec3r& rigid_body_point, const Vec3r& collision_normal,
                                       int face_ind, const Real u, const Real v, const Real w)
{
    // std::unique_ptr<Solver::RigidDeformableCollisionConstraint> collision_constraint = std::make_unique<Solver::RigidDeformableCollisionConstraint>(sdf, rigid_obj, rigid_body_point, collision_normal, deformable_obj, v1, v2, v3, u, v, w);
    // std::unique_ptr<Solver::ConstraintProjector> collision_projector = std::make_unique<Solver::RigidBodyConstraintProjector>(collision_constraint.get(), _sim->dt());
    const Eigen::Vector3i face = this->_mesh->face(face_ind);
    int v1 = face[0];
    int v2 = face[1];
    int v3 = face[2];
    
    Real* v1_ptr = this->_mesh->vertexPointer(v1);
    Real* v2_ptr = this->_mesh->vertexPointer(v2);
    Real* v3_ptr = this->_mesh->vertexPointer(v3);

    Real m1 = 1/_inv_B[v1];
    Real m2 = 1/_inv_B[v2];
    Real m3 = 1/_inv_B[v3];

    Solver::RigidDeformableCollisionConstraint& collision_constraint = 
        this->_constraints.template emplace_back<Solver::RigidDeformableCollisionConstraint>(sdf, rigid_obj, rigid_body_point, collision_normal, v1, v1_ptr, m1, v2, v2_ptr, m2, v3, v3_ptr, m3, u, v, w);

    this->_solver.setConstraintProjector(face_ind, this->_sim->dt(), &collision_constraint); // TODO: accomodate for first-order method

    // XPBDCollisionConstraint xpbd_collision_constraint;
    // xpbd_collision_constraint.constraint = std::move(collision_constraint);
    // xpbd_collision_constraint.projector_index = index;
    // xpbd_collision_constraint.num_steps_unused = 0;

    // _collision_constraints.push_back(std::move(xpbd_collision_constraint));
}

template<typename SolverType, typename... ConstraintTypes>
void FirstOrderXPBDMeshObject<SolverType, TypeList<ConstraintTypes...>>::clearCollisionConstraints()
{
    XPBDMeshObject<SolverType, TypeList<ConstraintTypes...>>::clearCollisionConstraints();

    for (unsigned i = 0; i < _vertices_in_collision.size(); i++)
    {
        _vertices_in_collision[i] = false;
    }
}

template<typename SolverType, typename... ConstraintTypes>
void FirstOrderXPBDMeshObject<SolverType, TypeList<ConstraintTypes...>>::_movePositionsInertially()
{
    for (int i = 0; i < this->_mesh->numVertices(); i++)
    {
        // if (_vertices_in_collision[i])
        // {
        //     std::cout << "Skipping vertex " << i << " in collision!" << std::endl;
        //     continue;
        // }

        this->_mesh->displaceVertex(i, Vec3r(0, 0, -this->_sim->gAccel() * this->_vertex_masses[i] * this->_sim->dt() * _inv_B[i]));
    }   
}


} // namespace Sim


/////////////////////////////////////////////////////////////////
// Explicit template instantiations
////////////////////////////////////////////////////////////////

#include "common/XPBDTypedefs.hpp"

namespace Sim 
{
using SolverTypesStableNeohookean = FirstOrderXPBDObjectSolverTypes<FirstOrderXPBDMeshObjectConstraintConfigurations::StableNeohookean::projector_type_list>;
using SolverTypesStableNeohookeanCombined = FirstOrderXPBDObjectSolverTypes<FirstOrderXPBDMeshObjectConstraintConfigurations::StableNeohookeanCombined::projector_type_list>;
using StableNeohookeanConstraints = FirstOrderXPBDMeshObjectConstraintConfigurations::StableNeohookean::constraint_type_list;
using StableNeohookeanCombinedConstraints = FirstOrderXPBDMeshObjectConstraintConfigurations::StableNeohookeanCombined::constraint_type_list;

// Stable Neohookean constraint config
template class FirstOrderXPBDMeshObject<SolverTypesStableNeohookean::GaussSeidel, StableNeohookeanConstraints>;
template class FirstOrderXPBDMeshObject<SolverTypesStableNeohookean::Jacobi, StableNeohookeanConstraints>;
template class FirstOrderXPBDMeshObject<SolverTypesStableNeohookean::ParallelJacobi, StableNeohookeanConstraints>;



// Stable Neohookean Combined constraint config
template class FirstOrderXPBDMeshObject<SolverTypesStableNeohookeanCombined::GaussSeidel, StableNeohookeanCombinedConstraints>;
template class FirstOrderXPBDMeshObject<SolverTypesStableNeohookeanCombined::Jacobi, StableNeohookeanCombinedConstraints>;
template class FirstOrderXPBDMeshObject<SolverTypesStableNeohookeanCombined::ParallelJacobi, StableNeohookeanCombinedConstraints>;



} //namespace Sim