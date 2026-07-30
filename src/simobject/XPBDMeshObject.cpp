#include "simobject/XPBDMeshObject.hpp"

#include "common/colors.hpp"

#include "config/simobject/XPBDMeshObjectConfig.hpp"
#include "config/simobject/FirstOrderXPBDMeshObjectConfig.hpp"

#include "simobject/RigidObject.hpp"
#include "simulation/Simulation.hpp"

#include "solver/xpbd_solver/XPBDGaussSeidelSolver.hpp"
#include "solver/xpbd_solver/XPBDJacobiSolver.hpp"
#include "solver/xpbd_solver/XPBDParallelJacobiSolver.hpp"
#include "solver/constraint/StaticDeformableCollisionConstraint.hpp"
#include "solver/constraint/RigidDeformableCollisionConstraint.hpp"
#include "solver/constraint/DeformableDeformableCollisionConstraint.hpp"
#include "solver/constraint/HydrostaticConstraint.hpp"
#include "solver/constraint/DeviatoricConstraint.hpp"
#include "solver/xpbd_projector/CombinedConstraintProjector.hpp"
#include "solver/xpbd_projector/ConstraintProjector.hpp"
#include "solver/xpbd_projector/RigidBodyConstraintProjector.hpp"
#include "utils/MeshUtils.hpp"
#include "utils/FileUtils.hpp"

#include "geometry/DeformableMeshSDF.hpp"

#include <deque>

#ifdef HAVE_CUDA
#include "gpu/resource/XPBDMeshObjectGPUResource.hpp"
#endif

namespace Sim
{

template<bool IsFirstOrder>
XPBDMeshObject_Base_<IsFirstOrder>::XPBDMeshObject_Base_(const Simulation* sim, const ConfigType* config)
    : Object(sim, config), RefinedTetMeshObject(config, config)
{
    // if multiple materials are specified, use those
    if (config->materialClasses().size() > 0)
    {
        for (const auto& mat_name : config->materialClasses())
        {
            _material_classes.push_back(sim->getMaterialClass(mat_name));
        }
    }

    // otherwise use the single material specified in the ObjectConfig
    else
    {
        _material_classes.push_back(sim->getMaterialClass(config->materialClass()));
    }

    if (_material_classes.size() == 0)
    {
        std::cerr << KRED << BOLD << "FATAL: " << RST << KRED << "No materials were specified!" << RST << std::endl;
        assert(0);
    }

    // set the material of the base object to the first material
    _material_class = _material_classes[0];
}

template<bool IsFirstOrder>
void XPBDMeshObject_Base_<IsFirstOrder>::serialize(std::vector<std::byte>& buf) const
{
    Object::serialize(buf);
    MeshObject::serialize(buf);
    pack(buf, _previous_vertices);
    pack(buf, _vertex_velocities);
    pack(buf, _initial_velocity);
    pack(buf, _material_classes);
    pack(buf, _vertex_masses);
    pack(buf, _is_fixed_vertex);
    pack(buf, _vertex_applied_force);
    pack(buf, _sdf);
    pack(buf, _heat_solver);
    pack(buf, _adaptive_mesh_refinement);
    pack(buf, _max_refinement_level);
    pack(buf, _refinement_distance_threshold);
    pack(buf, _damping_multiplier);
    pack(buf, _adjust_b_to_material);
    pack(buf, _vertex_B);
}

template<bool IsFirstOrder>
void XPBDMeshObject_Base_<IsFirstOrder>::deserialize(const std::byte*& buf)
{
    Object::deserialize(buf);
    MeshObject::deserialize(buf);
    unpack(buf, _previous_vertices);
    unpack(buf, _vertex_velocities);
    unpack(buf, _initial_velocity);
    unpack(buf, _material_classes);
    unpack(buf, _vertex_masses);
    unpack(buf, _is_fixed_vertex);
    unpack(buf, _vertex_applied_force);
    unpack(buf, _sdf);
    unpack(buf, _heat_solver);
    unpack(buf, _adaptive_mesh_refinement);
    unpack(buf, _max_refinement_level);
    unpack(buf, _refinement_distance_threshold);
    unpack(buf, _damping_multiplier);
    unpack(buf, _adjust_b_to_material);
    unpack(buf, _vertex_B);
}

template<bool IsFirstOrder>
void XPBDMeshObject_Base_<IsFirstOrder>::createSDF()
{
    if (!_sdf.has_value())
        _sdf.emplace(this, _sim->embreeScene());
}

////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////

template<bool IsFirstOrder, typename SolverType, typename... ConstraintTypes>
XPBDMeshObject_<IsFirstOrder, SolverType, TypeList<ConstraintTypes...>>::XPBDMeshObject_(const Simulation* sim, const ConfigType* config)
    : XPBDMeshObject_Base_<IsFirstOrder>(sim, config),
        _solver(this, config->numSolverIters(), config->residualPolicy())
{
    // make sure that if this object is using the 1st-Order formulation, that the XPBDSolver is too
    static_assert(SolverType::is_first_order == IsFirstOrder, "XPBD solver order much match object order!");

    /* extract values from the Config object */
    
    // set initial velocity if specified in config
    _initial_velocity = config->initialVelocity();
    
    // constraint specifications
    _constraint_type = config->constraintType();

    // local collision iterations
    _num_local_collision_iters = config->numLocalCollisionIters();

    // filename that has info on element classes (optional)
    _element_classes_filename = config->elementClassesFilename();

    _ground_faces_filename = config->groundFacesFilename();

    // filename that has info on fixed faces/vertices (optional)
    _fixed_faces_filename = config->fixedFacesFilename();

    // whether or not to compute heat conduction
    _compute_heat_conduction = config->computeHeatConduction();

    // whether or not to adaptively refine the mesh
    _adaptive_mesh_refinement = config->adaptiveMeshRefinement();

    // deepest level to refine to
    _max_refinement_level = config->maxRefinementLevel();

    // distance threshold for refinement
    _refinement_distance_threshold = config->refinementDistanceThreshold();

    // get the damping multiplier for 1st-order objects
    if constexpr (IsFirstOrder)
    {
        _damping_multiplier = config->dampingMultiplier();
        _adjust_b_to_material = config->adjustDampingToMaterial();
    }
}

template<bool IsFirstOrder, typename SolverType, typename... ConstraintTypes>
XPBDMeshObject_<IsFirstOrder, SolverType, TypeList<ConstraintTypes...>>::~XPBDMeshObject_()
{

}

template<bool IsFirstOrder, typename SolverType, typename... ConstraintTypes>
Geometry::AABB XPBDMeshObject_<IsFirstOrder, SolverType, TypeList<ConstraintTypes...>>::boundingBox() const
{
    return _mesh->boundingBox();
}

template<bool IsFirstOrder, typename SolverType, typename... ConstraintTypes>
void XPBDMeshObject_<IsFirstOrder, SolverType, TypeList<ConstraintTypes...>>::setup()
{
    loadAndConfigureMesh();

    // add the class property to the element mesh, with default value 0
    tetMesh()->template addElementProperty<int>("class", 0);
    tetMesh()->template addFaceProperty<int>("class", 0);
    tetMesh()->template addVertexProperty<int>("class", 0);

    // add the "cut_surface" face property to the mesh, with default value false
    // this will be turned to true for faces that are part of the cut surface
    tetMesh()->template addFaceProperty<bool>("on-cut-surface", false);

    if (_element_classes_filename.has_value())
    {
        Geometry::MeshProperty<int>& elem_class_prop = tetMesh()-> template getElementProperty<int>("class");
        Geometry::MeshProperty<int>& face_class_prop = tetMesh()-> template getFaceProperty<int>("class");
        Geometry::MeshProperty<int>& vert_class_prop = tetMesh()-> template getVertexProperty<int>("class");

        std::vector<int> elem_classes = FileUtils::readVectorFromFile<int>(_element_classes_filename.value());

        assert(elem_classes.size() == (unsigned)tetMesh()->numElements() && "Element classes file has a different number of elements than the mesh!");
        
        // set the class for each element in the mesh
        for (unsigned i = 0; i < elem_classes.size(); i++)
        {
            elem_class_prop.set(i, elem_classes[i]);
        }

        // set the class for each surface face in the mesh, from the element class for the element containing the surface face
        for (int i = 0; i < tetMesh()->numFaces(); i++)
        {
            int elem_index = tetMesh()->elementWithFace(i);
            face_class_prop.set(i, elem_classes[elem_index]);
        }

        // set the class for each vertex in the mesh, based on the element class for the element(s) containing the vertex
        // since multiple elements share the same vertices, the maximum of the element classes is used for each vertex
        for (int i = 0; i < tetMesh()->numVertices(); i++)
        {
            // get elements attached to the vertex
            std::vector<int> attached_elements = tetMesh()->vertexAttachedElements(i);
            // find the max element class of these attached elements
            int max_class = 0;
            for (const auto& elem_index : attached_elements)
            {
                max_class = std::max(max_class, elem_classes[elem_index]);
            }
            vert_class_prop.set(i, max_class);
        }
    }

    _solver.setup();

    // initialize the previous vertices matrix once we've loaded the mesh
    _previous_vertices.resize(_mesh->vertices().totalSize());
    
    for (const auto& vert_ind : _mesh->vertices().validIndices())
    {
        _previous_vertices[vert_ind] = _mesh->vertex(vert_ind);
    }

    // initialize vertex applied forces
    _vertex_applied_force.resize(_mesh->vertices().totalSize(), Vec3r::Zero());
    
    // initialize each vertex's velocity with the specified bulk initial velocity
    _vertex_velocities.resize(_mesh->vertices().totalSize());
    for (auto& vel : _vertex_velocities)
        vel = _initial_velocity;

    _calculatePerVertexQuantities();
    _createElasticConstraints();     // create constraints and add ConstraintProjectors to the solver object

    // if we are modeling heat conduction, set up the solver
    if (_compute_heat_conduction)
    {
            /** 
             * 
             * 
             * 
             * TODO: only using first material right now. Extend to handle multiple materials in the same mesh?
             * 
             * 
             * 
             * 
             * 
             */
            _heat_solver.emplace(refinedTetMesh(), _material_classes[0]->material(), 0, 23);

            if (_ground_faces_filename.has_value())
            {
                std::set<int> vertices;
                std::vector<int> faces;
                MeshUtils::verticesAndFacesFromFixedFacesFile(_ground_faces_filename.value(), vertices, faces);
                for (const auto& v : vertices)
                {
                    _heat_solver->setVoltageAtBoundary(v, 0, true);
                }
            }
            else
            {
                // throw an error if no grounded faces are specified
                std::cerr << KRED << BOLD << "FATAL:" << RST << KRED << " Thermal simulation enabled but grounded faces not specified! (did you forget to specify the ground-faces-filename parameter?)" << RST << std::endl;
                assert(0);
            }
    }
        
    // if the fixed-faces file is given, read the fixed vertices from it and then set those vertices to be fixed during the sim
    if (_fixed_faces_filename.has_value())
    {
        std::set<int> vertices;
        std::vector<int> faces;
        MeshUtils::verticesAndFacesFromFixedFacesFile(_fixed_faces_filename.value(), vertices, faces);
        for (const auto& v : vertices)
        {
            addAttachmentConstraint(v, v, &_mesh->initialVertices());
        }
    }

    // set the characteristic dimension as the smallest dim in the AABB
    Geometry::AABB bbox = _mesh->boundingBox();
    this->_char_dim = bbox.size().minCoeff();

    // test: refine all elements in the mesh uniformly
    // for (const auto& elem_index : refinedTetMesh()->elements().validIndices())
    // {
    //     refineElement(elem_index, 1, true);
    // }
}

// template<bool IsFirstOrder, typename SolverType, typename... ConstraintTypes>
// int XPBDMeshObject_<IsFirstOrder, SolverType, TypeList<ConstraintTypes...>>::numConstraintsForPosition(const int index) const
// {
//     if constexpr (std::is_same_v<typename SolverType::projector_type_list, XPBDMeshObjectConstraintConfigurations::StableNeohookean::projector_type_list>)
//     {
//         return 2*_vertex_attached_elements[index];   // if sequential constraints are used, there are 2 constraints per element ==> # of constraint updates = 2 * # of elements attached to that vertex
//     }
//     else if constexpr (std::is_same_v<typename SolverType::projector_type_list, XPBDMeshObjectConstraintConfigurations::StableNeohookeanCombined::projector_type_list>)
//     {
//         return _vertex_attached_elements[index];     // if combined constraints are used, there are 2 constraints per element but they are solved together ==> # of constraint updates = # of elements attached to that vertex
//     }
//     else
//     {
//         assert(0); // something weird happened, shouldn't get to here
//         return 0;
//     }
// }

template<bool IsFirstOrder, typename SolverType, typename... ConstraintTypes>
Solver::ConstraintProjectorReference<Solver::ConstraintProjector<IsFirstOrder, Solver::StaticDeformableCollisionConstraint>>
XPBDMeshObject_<IsFirstOrder, SolverType, TypeList<ConstraintTypes...>>::addStaticCollisionConstraint(
    const Geometry::SDF* sdf, const Vec3r& p, const Vec3r& n,
    int v1, int v2, int v3, const Real u, const Real v, const Real w,
    int element_ind, int face_ind
)
{
    Geometry::Mesh::vertices_vec_type* vec_ptr = &_mesh->vertices();

    Real m1 = vertexConstraintInertia(v1);
    Real m2 = vertexConstraintInertia(v2);
    Real m3 = vertexConstraintInertia(v3);

    std::vector<Solver::StaticDeformableCollisionConstraint>& constraint_vec = _constraints.template get<Solver::StaticDeformableCollisionConstraint>();
    constraint_vec.emplace_back(sdf, p, n, v1, vec_ptr, m1, v2, vec_ptr, m2, v3, vec_ptr, m3, u, v, w, element_ind, face_ind);

    using ConstraintRefType = Solver::ConstraintReference<Solver::StaticDeformableCollisionConstraint>;
    auto proj_ref = _solver.addConstraintProjector(_sim->dt(), ConstraintRefType(constraint_vec, constraint_vec.size()-1));

    // add an entry in the element -> collision projector index map
    _element_to_collision_proj_index.insert({element_ind, proj_ref.index()});

    return proj_ref;
}

template<bool IsFirstOrder, typename SolverType, typename... ConstraintTypes>
Solver::ConstraintProjectorReference<Solver::RigidBodyConstraintProjector<IsFirstOrder, Solver::RigidDeformableCollisionConstraint>>
XPBDMeshObject_<IsFirstOrder, SolverType, TypeList<ConstraintTypes...>>::addRigidDeformableCollisionConstraint(const Geometry::SDF* sdf, Sim::RigidObject* rigid_obj, const Vec3r& rigid_body_point, const Vec3r& collision_normal,
                                       int face_ind, const Real u, const Real v, const Real w)
{
    const Eigen::Vector3i face = _mesh->face(face_ind);
    int v1 = face[0];
    int v2 = face[1];
    int v3 = face[2];
    
    Geometry::Mesh::vertices_vec_type* vec_ptr = &_mesh->vertices();

    Real m1 = vertexConstraintInertia(v1);
    Real m2 = vertexConstraintInertia(v2);
    Real m3 = vertexConstraintInertia(v3);

    std::vector<Solver::RigidDeformableCollisionConstraint>& constraint_vec = _constraints.template get<Solver::RigidDeformableCollisionConstraint>();
    constraint_vec.emplace_back(sdf, rigid_obj, rigid_body_point, collision_normal, v1, vec_ptr, m1, v2, vec_ptr, m2, v3, vec_ptr, m3, u, v, w);

    using ConstraintRefType = Solver::ConstraintReference<Solver::RigidDeformableCollisionConstraint>;
    return _solver.addConstraintProjector(_sim->dt(), ConstraintRefType(constraint_vec, constraint_vec.size()-1));
}

template<bool IsFirstOrder, typename SolverType, typename... ConstraintTypes>
void XPBDMeshObject_<IsFirstOrder, SolverType, TypeList<ConstraintTypes...>>::clearCollisionConstraints()
{
    // set any collision constraint projectors in the solver invalid
    // NOTE: because the collision constraint
    using StaticCollisionConstraintType = Solver::ConstraintProjector<IsFirstOrder, Solver::StaticDeformableCollisionConstraint>;
    using DeformableCollisionConstraintType = Solver::ConstraintProjector<IsFirstOrder, Solver::DeformableDeformableCollisionConstraint>;
    using RigidCollisionConstraintType = Solver::RigidBodyConstraintProjector<IsFirstOrder, Solver::RigidDeformableCollisionConstraint>;
    _solver.template clearProjectorsOfType<StaticCollisionConstraintType>();
    _solver.template clearProjectorsOfType<DeformableCollisionConstraintType>();
    _solver.template clearProjectorsOfType<RigidCollisionConstraintType>();

    // clear the collision constraints lists
    _constraints.template clear<Solver::StaticDeformableCollisionConstraint>();
    _constraints.template clear<Solver::DeformableDeformableCollisionConstraint>();
    _constraints.template clear<Solver::RigidDeformableCollisionConstraint>();

    // clear the element index -> collision projector index map
    _element_to_collision_proj_index.clear();

}

template<bool IsFirstOrder, typename SolverType, typename... ConstraintTypes>
Solver::ConstraintProjectorReference<Solver::ConstraintProjector<IsFirstOrder, Solver::OffsetAttachmentConstraint>> 
XPBDMeshObject_<IsFirstOrder, SolverType, TypeList<ConstraintTypes...>>::addOffsetAttachmentConstraint(int v_ind, const Vec3r* attach_pos_ptr, const Vec3r& attachment_offset)
{
    Real mass = vertexConstraintInertia(v_ind);

    Geometry::Mesh::vertices_vec_type* vec_ptr = &_mesh->vertices();

    std::vector<Solver::OffsetAttachmentConstraint>& constraint_vec = _constraints.template get<Solver::OffsetAttachmentConstraint>();
    constraint_vec.emplace_back(v_ind, vec_ptr, mass, attach_pos_ptr, attachment_offset);
    
    using ConstraintRefType = Solver::ConstraintReference<Solver::OffsetAttachmentConstraint>;
    auto proj = _solver.addConstraintProjector(_sim->dt(), ConstraintRefType(constraint_vec, constraint_vec.size()-1));
    _vertex_to_offset_attachment_proj_index.insert({v_ind, proj.index()});
    return proj;
}

template<bool IsFirstOrder, typename SolverType, typename... ConstraintTypes>
void XPBDMeshObject_<IsFirstOrder, SolverType, TypeList<ConstraintTypes...>>::clearOffsetAttachmentConstraints()
{
    using OffsetAttachmentConstraintProjType = Solver::ConstraintProjector<IsFirstOrder, Solver::OffsetAttachmentConstraint>;
    // clear projectors
    _solver.template clearProjectorsOfType<OffsetAttachmentConstraintProjType>();
    // clear constraints
    _constraints.template clear<Solver::OffsetAttachmentConstraint>();

    _vertex_to_offset_attachment_proj_index.clear();
}

template<bool IsFirstOrder, typename SolverType, typename... ConstraintTypes>
Solver::ConstraintProjectorReference<Solver::ConstraintProjector<IsFirstOrder, Solver::FaceOffsetAttachmentConstraint>> 
XPBDMeshObject_<IsFirstOrder, SolverType, TypeList<ConstraintTypes...>>::addFaceOffsetAttachmentConstraint(int face_ind, const Vec3r& bary_coords, const Vec3r* attach_pos_ptr, const Vec3r& attachment_offset)
{
    const Vec3i& face = _mesh->face(face_ind);
    int v1 = face[0];
    int v2 = face[1];
    int v3 = face[2];

    Real m1 = vertexConstraintInertia(v1);
    Real m2 = vertexConstraintInertia(v2);
    Real m3 = vertexConstraintInertia(v3);

    Geometry::Mesh::vertices_vec_type* vec_ptr = &_mesh->vertices();

    std::vector<Solver::FaceOffsetAttachmentConstraint>& constraint_vec = _constraints.template get<Solver::FaceOffsetAttachmentConstraint>();
    constraint_vec.emplace_back(v1, m1, v2, m2, v3, m3, vec_ptr, bary_coords, attach_pos_ptr, attachment_offset);
    
    using ConstraintRefType = Solver::ConstraintReference<Solver::FaceOffsetAttachmentConstraint>;
    return _solver.addConstraintProjector(_sim->dt(), ConstraintRefType(constraint_vec, constraint_vec.size()-1));
}

template<bool IsFirstOrder, typename SolverType, typename... ConstraintTypes>
void XPBDMeshObject_<IsFirstOrder, SolverType, TypeList<ConstraintTypes...>>::clearFaceOffsetAttachmentConstraints()
{
    using FaceOffsetAttachmentConstraintProjType = Solver::ConstraintProjector<IsFirstOrder, Solver::FaceOffsetAttachmentConstraint>;
    // clear projectors
    _solver.template clearProjectorsOfType<FaceOffsetAttachmentConstraintProjType>();
    // clear constraints
    _constraints.template clear<Solver::FaceOffsetAttachmentConstraint>();
}

template<bool IsFirstOrder, typename SolverType, typename... ConstraintTypes>
Solver::ConstraintProjectorReference<Solver::ConstraintProjector<IsFirstOrder, Solver::AttachmentConstraint>> 
XPBDMeshObject_<IsFirstOrder, SolverType, TypeList<ConstraintTypes...>>::addAttachmentConstraint(int v_ind, const Vec3r* attach_pos_ptr)
{
    Real mass = vertexConstraintInertia(v_ind);

    Geometry::Mesh::vertices_vec_type* vec_ptr = &_mesh->vertices();

    std::vector<Solver::AttachmentConstraint>& constraint_vec = _constraints.template get<Solver::AttachmentConstraint>();
    constraint_vec.emplace_back(v_ind, vec_ptr, mass, attach_pos_ptr);
    
    using ConstraintRefType = Solver::ConstraintReference<Solver::AttachmentConstraint>;
    auto proj = _solver.addConstraintProjector(_sim->dt(), ConstraintRefType(constraint_vec, constraint_vec.size()-1));
    _vertex_to_attachment_proj_index.insert({v_ind, proj.index()});
    return proj;
}

template<bool IsFirstOrder, typename SolverType, typename... ConstraintTypes>
Solver::ConstraintProjectorReference<Solver::ConstraintProjector<IsFirstOrder, Solver::AttachmentConstraint>> 
XPBDMeshObject_<IsFirstOrder, SolverType, TypeList<ConstraintTypes...>>::addAttachmentConstraint(int v_ind, int attach_ind, const std::vector<Vec3r>* attach_vec_ptr)
{
    Real mass = vertexConstraintInertia(v_ind);

    Geometry::Mesh::vertices_vec_type* vec_ptr = &_mesh->vertices();

    std::vector<Solver::AttachmentConstraint>& constraint_vec = _constraints.template get<Solver::AttachmentConstraint>();
    constraint_vec.emplace_back(v_ind, vec_ptr, mass, attach_ind, attach_vec_ptr);
    
    using ConstraintRefType = Solver::ConstraintReference<Solver::AttachmentConstraint>;
    auto proj = _solver.addConstraintProjector(_sim->dt(), ConstraintRefType(constraint_vec, constraint_vec.size()-1));
    _vertex_to_attachment_proj_index.insert({v_ind, proj.index()});
    return proj;
}

template<bool IsFirstOrder, typename SolverType, typename... ConstraintTypes>
void XPBDMeshObject_<IsFirstOrder, SolverType, TypeList<ConstraintTypes...>>::clearAttachmentConstraints()
{
    using AttachmentConstraintProjType = Solver::ConstraintProjector<IsFirstOrder, Solver::AttachmentConstraint>;
    // clear projectors
    _solver.template clearProjectorsOfType<AttachmentConstraintProjType>();
    // clear constraints
    _constraints.template clear<Solver::AttachmentConstraint>();

    _vertex_to_attachment_proj_index.clear();
}

template<bool IsFirstOrder, typename SolverType, typename... ConstraintTypes>
Vec3r XPBDMeshObject_<IsFirstOrder, SolverType, TypeList<ConstraintTypes...>>::attachmentConstraintTotalForce() const
{
    using AttachmentConstraintProjType = Solver::ConstraintProjector<IsFirstOrder, Solver::AttachmentConstraint>;

    Vec3r total_force = Vec3r::Zero();

    // iterate through constraint projectors and compute force associated with the last iteration
    const std::vector<AttachmentConstraintProjType>& projs = _solver.template getConstraintProjectorsOfType<AttachmentConstraintProjType>();
    for (const auto& proj : projs)
    {
        std::vector<Vec3r> forces = proj.constraintForces();
        total_force += forces[0];   // attachment constraint only affects one position
    }

    return total_force;
}

template<bool IsFirstOrder, typename SolverType, typename... ConstraintTypes>
int XPBDMeshObject_<IsFirstOrder, SolverType, TypeList<ConstraintTypes...>>::numActiveAttachmentConstraints() const
{
    using ProjectorType = Solver::ConstraintProjector<IsFirstOrder, Solver::AttachmentConstraint>;
    const auto& projs = _solver.template getConstraintProjectorsOfType<ProjectorType>();

    int cnt = 0;
    for (const auto& proj : projs)
    {
        if (proj.isValid())
            cnt++;
    }
    return cnt;
}

template<bool IsFirstOrder, typename SolverType, typename... ConstraintTypes>
void XPBDMeshObject_<IsFirstOrder, SolverType, TypeList<ConstraintTypes...>>::_calculatePerVertexQuantities()
{
    // calculate masses for each vertex
    _vertex_masses.resize(_mesh->numVertices());
    _is_fixed_vertex.resize(_mesh->numVertices(), false);

    std::vector<Real> vertex_E(_mesh->numVertices());
    std::vector<Real> vertex_nu(_mesh->numVertices());
    const Geometry::MeshProperty<int>& class_prop = tetMesh()->template getElementProperty<int>("class"); 
    for (int i = 0; i < tetMesh()->numElements(); i++)
    {
        // get the material for this element
        int material_ind = class_prop.get(i);
        if (static_cast<unsigned>(material_ind) >= _material_classes.size())
        {
            std::cout << KYEL << BOLD << "WARNING: " << RST << KYEL << "Only " << _material_classes.size() << " materials were specified, but element " <<
                i << " has class " << material_ind << ". (Specify more materials in the config file)" << RST << std::endl;
            
            // set the material index to the largest valid index
            material_ind = _material_classes.size() - 1;
        }
        const ElasticMaterial& material = _material_classes[material_ind]->material();

        const Eigen::Vector4i& element = tetMesh()->element(i);
        // compute volume from X
        const Real volume = tetMesh()->elementVolume(i);
        // _vols(i) = vol;

        // compute mass of element
        const Real element_mass = volume * material.density();
        // add mass contribution of element to each of its vertices
        _vertex_masses[element[0]] += element_mass/4.0;
        _vertex_masses[element[1]] += element_mass/4.0;
        _vertex_masses[element[2]] += element_mass/4.0;
        _vertex_masses[element[3]] += element_mass/4.0;

        // add material properties to each of its vertices
        vertex_E[element[0]] += material.E() / tetMesh()->vertexAttachedElements(element[0]).size();
        vertex_E[element[1]] += material.E() / tetMesh()->vertexAttachedElements(element[1]).size();
        vertex_E[element[2]] += material.E() / tetMesh()->vertexAttachedElements(element[2]).size();
        vertex_E[element[3]] += material.E() / tetMesh()->vertexAttachedElements(element[3]).size();

        vertex_nu[element[0]] += material.nu() / tetMesh()->vertexAttachedElements(element[0]).size();
        vertex_nu[element[1]] += material.nu() / tetMesh()->vertexAttachedElements(element[1]).size();
        vertex_nu[element[2]] += material.nu() / tetMesh()->vertexAttachedElements(element[2]).size();
        vertex_nu[element[3]] += material.nu() / tetMesh()->vertexAttachedElements(element[3]).size();
    }

    // for 1st-order objects, calculate per-vertex damping
    if constexpr (IsFirstOrder)
    {
        _vertex_B.resize(_mesh->numVertices());
        for (int i = 0; i < _mesh->numVertices(); i++)
        {
            if (_adjust_b_to_material)
            {
                _vertex_B[i] = tetMesh()->vertexRestVolume(i) * _damping_multiplier * vertex_E[i] / (1+vertex_nu[i]);
            }
            else
            {
                _vertex_B[i] = tetMesh()->vertexRestVolume(i) * _damping_multiplier;
            }
        }
    }
    
}

template<bool IsFirstOrder, typename SolverType, typename... ConstraintTypes>
void XPBDMeshObject_<IsFirstOrder, SolverType, TypeList<ConstraintTypes...>>::_createElasticConstraints()
{
    // reserve space for the elastic constraints we're creating
    _constraints.template reserve<Solver::HydrostaticConstraint>(tetMesh()->numElements());
    _constraints.template reserve<Solver::DeviatoricConstraint>(tetMesh()->numElements());

    // create constraint(s) for each element
    const Geometry::MeshProperty<int>& class_prop = tetMesh()->template getElementProperty<int>("class"); 
    for (int i = 0; i < tetMesh()->numElements(); i++)
    {
        // get the material for this element
        int material_ind = class_prop.get(i);
        if ((unsigned)material_ind >= _material_classes.size())  material_ind = _material_classes.size()-1;
        const ElasticMaterial& material = _material_classes[material_ind]->material();

        // get the vertices for the element
        const Eigen::Vector4i element = tetMesh()->element(i);
        const int v0 = element[0];
        const int v1 = element[1];
        const int v2 = element[2];
        const int v3 = element[3];

        // Real* v0_ptr = _mesh->vertexPointer(v0);
        // Real* v1_ptr = _mesh->vertexPointer(v1);
        // Real* v2_ptr = _mesh->vertexPointer(v2);
        // Real* v3_ptr = _mesh->vertexPointer(v3);

        Geometry::Mesh::vertices_vec_type* vec_ptr = &_mesh->vertices();

        Real m0 = vertexConstraintInertia(v0);
        Real m1 = vertexConstraintInertia(v1);
        Real m2 = vertexConstraintInertia(v2);
        Real m3 = vertexConstraintInertia(v3);

        // if the constraint configuration is StableNeohookean, add separate constraint projectors for the hydrostatic and deviatoric constraints
        if constexpr (std::is_same_v<typename SolverType::projector_type_list, typename XPBDMeshObjectConstraintConfigurations<IsFirstOrder>::StableNeohookean::projector_type_list>)
        {
            std::vector<Solver::HydrostaticConstraint>& hyd_constraint_vec = _constraints.template get<Solver::HydrostaticConstraint>();
            std::vector<Solver::DeviatoricConstraint>& dev_constraint_vec = _constraints.template get<Solver::DeviatoricConstraint>();
            hyd_constraint_vec.emplace_back(v0, vec_ptr, m0, v1, vec_ptr, m1, v2, vec_ptr, m2, v3, vec_ptr, m3, material);
            dev_constraint_vec.emplace_back(v0, vec_ptr, m0, v1, vec_ptr, m1, v2, vec_ptr, m2, v3, vec_ptr, m3, material);
            
            using HydConstraintRefType = Solver::ConstraintReference<Solver::HydrostaticConstraint>;
            using DevConstraintRefType = Solver::ConstraintReference<Solver::DeviatoricConstraint>;
            // TODO: support separate constraints - maybe though SeparateConstraintProjector class?.
            _solver.addConstraintProjector(_sim->dt(), HydConstraintRefType(hyd_constraint_vec, hyd_constraint_vec.size()-1));
            _solver.addConstraintProjector(_sim->dt(), DevConstraintRefType(dev_constraint_vec, dev_constraint_vec.size()-1));
            
        }
        // if the constraint configuration is StableNeohookeanCombined, add a combined constraint projector for the hydrostatic and deviatoric constraints
        if constexpr (std::is_same_v<typename SolverType::projector_type_list, typename XPBDMeshObjectConstraintConfigurations<IsFirstOrder>::StableNeohookeanCombined::projector_type_list>)
        {
            std::vector<Solver::HydrostaticConstraint>& hyd_constraint_vec = _constraints.template get<Solver::HydrostaticConstraint>();
            std::vector<Solver::DeviatoricConstraint>& dev_constraint_vec = _constraints.template get<Solver::DeviatoricConstraint>();
            hyd_constraint_vec.emplace_back(v0, vec_ptr, m0, v1, vec_ptr, m1, v2, vec_ptr, m2, v3, vec_ptr, m3, material);
            dev_constraint_vec.emplace_back(v0, vec_ptr, m0, v1, vec_ptr, m1, v2, vec_ptr, m2, v3, vec_ptr, m3, material);

            using HydConstraintRefType = Solver::ConstraintReference<Solver::HydrostaticConstraint>;
            using DevConstraintRefType = Solver::ConstraintReference<Solver::DeviatoricConstraint>;
            
            _solver.addConstraintProjector(_sim->dt(),
                DevConstraintRefType(dev_constraint_vec, dev_constraint_vec.size()-1), 
                HydConstraintRefType(hyd_constraint_vec, hyd_constraint_vec.size()-1)
            );
        }
    }
}

template<bool IsFirstOrder, typename SolverType, typename... ConstraintTypes>
std::string XPBDMeshObject_<IsFirstOrder, SolverType, TypeList<ConstraintTypes...>>::toString(const int indent) const
{
    // TODO: complete toString
    return Object::toString(indent+1);
}

template<bool IsFirstOrder, typename SolverType, typename... ConstraintTypes>
void XPBDMeshObject_<IsFirstOrder, SolverType, TypeList<ConstraintTypes...>>::update()
{
    // set _x_prev to be ready for the next substep
    for (const auto& vert_ind : _mesh->vertices().validIndices())
    {
        _previous_vertices[vert_ind] = _mesh->vertex(vert_ind);
    }

    _movePositionsInertially();
    _projectConstraints();

    // for (int i = 0; i < tetMesh()->numElements(); i++)
    // {
    //     const Mat3r F = tetMesh()->elementDeformationGradient(i);
    //     if (F.determinant() <= 0)
    //     {
    //         std::cout << "element " << i << " det(F) <= 0!" << std::endl;
    //     }
        
    // }

    if (_compute_heat_conduction)
        _heat_solver->step(_sim->dt());
}

template<bool IsFirstOrder, typename SolverType, typename... ConstraintTypes>
void XPBDMeshObject_<IsFirstOrder, SolverType, TypeList<ConstraintTypes...>>::_movePositionsInertially()
{
    const Real dt = _sim->dt();
    if constexpr (IsFirstOrder)
    {
        for (const auto& index : _mesh->vertices().validIndices())
        {
            Vec3r g_force = Vec3r(0, 0, -_sim->gAccel() * _vertex_masses[index]);
            Vec3r total_force = g_force + _vertex_applied_force[index];
            Vec3r dx = total_force * dt / _vertex_B[index];
            _mesh->displaceVertex(index, dx);
        }
        
    }
    else
    {
        // move vertices according to their velocity
        for (const auto& index : _mesh->vertices().validIndices())
        {
            _mesh->displaceVertex(index, dt*_vertex_velocities[index]);

            // external forces (right now just gravity, which acts in -z direction)
            Vec3r g_force = Vec3r(0, 0, -_sim->gAccel() * _vertex_masses[index]);
            Vec3r total_force = g_force + _vertex_applied_force[index];
            Vec3r dx = total_force * dt * dt / _vertex_masses[index];
            _mesh->displaceVertex(index, dx);
        }
    }
}

template<bool IsFirstOrder, typename SolverType, typename... ConstraintTypes>
void XPBDMeshObject_<IsFirstOrder, SolverType, TypeList<ConstraintTypes...>>::_projectConstraints()
{
    // global iteration - initial solve of all the constraints
    _solver.solve();


    // local iterations - helpful for better convergence of applied collision constraints

    typename SolverType::projector_reference_container_type proj_to_reproject = _gatherProjectorsForLocalCollisionIterations();
    // reproject the added constraints with solver iterations and no re-initialization
    _solver.solve(proj_to_reproject, _num_local_collision_iters, false);

    // TODO: remove
    // for (int i = 0; i < _mesh->numVertices(); i++)
    // {
    //     const Vec3r& v = _mesh->vertex(i);
    //     if (v[2] < 0)
    //     {
    //         _mesh->setVertex(i, Vec3r(v[0], v[1], 0));
    //     }
    // }

    // TODO: replace with constraints?
    // enforce fixed vertices (move them back to previous position)
    for (const auto& index : _mesh->vertices().validIndices())
    {
        if (vertexFixed(index))
        {
            _mesh->setVertex(index, vertexPreviousPosition(index));
        }
    }

}

template<bool IsFirstOrder, typename SolverType, typename... ConstraintTypes>
void XPBDMeshObject_<IsFirstOrder, SolverType, TypeList<ConstraintTypes...>>::velocityUpdate()
{
    // TODO: apply frictional forces
    // we do this in the velocity update (i.e. after update() is finished) to ensure that all objects have had their constraints projected already
    // for (const auto& c : _collision_constraints)
    // {
    //     const Real lam = _solver->constraintProjectors()[c.projector_index]->lambda()[0];
    //     // only apply friction forces for this constraint if it was active (i.e. lambda > 0)
    //     // if it was "inactive", there was no penetration and thus no contact and thus no friction
    //     if (lam > 0)
    //     {
    //         c.constraint->applyFriction(lam, _material.muS(), _material.muK());
    //     }
    // }

    // for (int i = 0; i < tetMesh()->numElements(); i++)
    // {
    //     Mat3r F = tetMesh()->elementDeformationGradient(i);
    //     if (F.determinant() < 0)
    //     {
    //         std::cout << "det(F) < 0 for element " << i << std::endl;
    //     }
    // }

    // velocities are simply (cur_pos - last_pos) / deltaT

    for (const auto& index : _mesh->vertices().validIndices())
    {
        _vertex_velocities[index] = (_mesh->vertex(index) - vertexPreviousPosition(index)) / _sim->dt();
    }
}

template<bool IsFirstOrder, typename SolverType, typename... ConstraintTypes>
void XPBDMeshObject_<IsFirstOrder, SolverType, TypeList<ConstraintTypes...>>::removeElement(int elem_index)
{
    if (!tetMesh()->elementValid(elem_index))
        return;

    Vec4i removed_element = refinedTetMesh()->element(elem_index);  // get a copy of the element vertices we are about to remove
    refinedTetMesh()->removeElement(elem_index);


    /** Get the newest added/removed vertices/hanging vertices/faces/elements */
    const std::vector<Geometry::RefinedTetMesh::NewVertex>& latest_added_vertices = refinedTetMesh()->latestAddedVertices();
    const std::vector<Geometry::RefinedTetMesh::RemovedVertex>& latest_removed_vertices = refinedTetMesh()->latestRemovedVertices();
    const std::vector<int>& latest_added_faces = refinedTetMesh()->latestAddedFaces();
    const std::vector<int>& latest_added_elements = refinedTetMesh()->latestAddedElements();
    const std::vector<Geometry::TetMesh::RemovedElement>& latest_removed_elements = refinedTetMesh()->latestRemovedElements();
    const std::vector<Geometry::RefinedTetMesh::NewVertex>& latest_added_hanging_vertices = refinedTetMesh()->latestAddedHangingVertices();
    const std::vector<int>& latest_removed_hanging_vertices = refinedTetMesh()->latestRemovedHangingVertices();

    // std::cout << "Added elements: ";
    // for (const auto& e : latest_added_elements)
    //     std::cout << e << ", ";
    // std::cout << std::endl;

    // std::cout << "Removed elements: ";
    // for (const auto& e : latest_removed_elements)
    //     std::cout << e.index << ", ";
    // std::cout << std::endl;

    // std::cout << "Added vertices: ";
    // for (const auto& v : latest_added_vertices)
    //     std::cout << v.index << ", ";
    // std::cout << std::endl;

    // std::cout << "Removed vertices: ";
    // for (const auto& v : latest_removed_vertices)
    //     std::cout << v.index << ", ";
    // std::cout << std::endl;


    // create the vector for new element classes
    Geometry::MeshProperty<int>& class_elem_prop = tetMesh()->template getElementProperty<int>("class");
    int refined_elem_class = class_elem_prop.get(elem_index);

    // create the vector for removed element classes
    std::vector<int> added_element_classes(latest_added_elements.size());
    for (unsigned i = 0; i < latest_added_elements.size(); i++)
    {
        added_element_classes[i] = refined_elem_class;
    }

    std::vector<int> removed_element_classes(latest_removed_elements.size());
    for (unsigned i = 0; i < latest_removed_elements.size(); i++)
    {
        removed_element_classes[i] = refined_elem_class;
    }

    // update everything based for the latest mesh topology change(s)
    _updateAfterMeshTopologyChange(
        latest_added_vertices, latest_removed_vertices, latest_added_hanging_vertices, latest_removed_hanging_vertices,
        latest_added_faces, latest_added_elements, latest_removed_elements, added_element_classes, removed_element_classes
    );

    /** Update the 'class' property for new vertices, elements, and faces */
    Geometry::MeshProperty<int>& class_vert_prop = _mesh->template getVertexProperty<int>("class");
    Geometry::MeshProperty<int>& class_face_prop = _mesh->template getFaceProperty<int>("class");
    
    for (const auto& new_vert : latest_added_vertices)
    {
        class_vert_prop.set(new_vert.index, refined_elem_class);
    }
    for (const auto& new_face : latest_added_faces)
    {
        class_face_prop.set(new_face, refined_elem_class);
    }
    for (const auto& new_elem : latest_added_elements)
    {
        class_elem_prop.set(new_elem, refined_elem_class);
    }

    /** Update the 'on_cut_surface' vertex property. */
    // vertices that were part of the removed element, and were not in the set of removed vertices are on the cut surface
    Geometry::MeshProperty<bool>& on_cut_surface_prop = _mesh->template getFaceProperty<bool>("on-cut-surface");
    // look for each of the removed element's vertices in the newly added faces
    for (const auto& new_face_index : latest_added_faces)
    {
        const Vec3i& face = refinedTetMesh()->face(new_face_index);
        bool v1_in_elem = (face[0] == removed_element[0] || face[0] == removed_element[1] || face[0] == removed_element[2] || face[0] == removed_element[3]);
        if (!v1_in_elem)
        {
            on_cut_surface_prop.set(new_face_index, false);
            continue;
        }

        bool v2_in_elem = (face[1] == removed_element[0] || face[1] == removed_element[1] || face[1] == removed_element[2] || face[1] == removed_element[3]);
        if (!v2_in_elem)
        {
            on_cut_surface_prop.set(new_face_index, false);
            continue;
        }
        
        bool v3_in_elem = (face[2] == removed_element[0] || face[2] == removed_element[1] || face[2] == removed_element[2] || face[2] == removed_element[3]);
        if (!v3_in_elem)
        {
            on_cut_surface_prop.set(new_face_index, false);
            continue;
        }
            
        // face was part of the element ==> it's on the cut surface
        on_cut_surface_prop.set(new_face_index, true);
    }

}

template<bool IsFirstOrder, typename SolverType, typename... ConstraintTypes>
void XPBDMeshObject_<IsFirstOrder, SolverType, TypeList<ConstraintTypes...>>::refineElement(int elem_index, int refinement_level, bool absolute)
{
    bool refined = refinedTetMesh()->refineElement(elem_index, refinement_level, absolute);
    if (!refined)
    {
        return;
    }

    // std::cout << "Element refined! New number of elements: " << tetMesh()->numElements() << std::endl;

    /** Get the newest added/removed vertices/hanging vertices/faces/elements */
    const std::vector<Geometry::RefinedTetMesh::NewVertex>& latest_added_vertices = refinedTetMesh()->latestAddedVertices();
    const std::vector<Geometry::RefinedTetMesh::RemovedVertex>& latest_removed_vertices = refinedTetMesh()->latestRemovedVertices();
    const std::vector<int>& latest_added_faces = refinedTetMesh()->latestAddedFaces();
    const std::vector<int>& latest_added_elements = refinedTetMesh()->latestAddedElements();
    const std::vector<Geometry::TetMesh::RemovedElement>& latest_removed_elements = refinedTetMesh()->latestRemovedElements();
    const std::vector<Geometry::RefinedTetMesh::NewVertex>& latest_added_hanging_vertices = refinedTetMesh()->latestAddedHangingVertices();
    const std::vector<int>& latest_removed_hanging_vertices = refinedTetMesh()->latestRemovedHangingVertices();

    // create the vector for new element classes
    Geometry::MeshProperty<int>& class_elem_prop = tetMesh()->template getElementProperty<int>("class");
    int refined_elem_class = class_elem_prop.get(elem_index);

    std::vector<int> added_element_classes(latest_added_elements.size());
    for (unsigned i = 0; i < latest_added_elements.size(); i++)
    {
        added_element_classes[i] = refined_elem_class;
    }
    
    // create the vector for removed element classes
    std::vector<int> removed_element_classes(latest_removed_elements.size());
    for (unsigned i = 0; i < latest_removed_elements.size(); i++)
    {
        removed_element_classes[i] = refined_elem_class;
    }

    // update everything based for the latest mesh topology change(s)
    _updateAfterMeshTopologyChange(
        latest_added_vertices, latest_removed_vertices, latest_added_hanging_vertices, latest_removed_hanging_vertices,
        latest_added_faces, latest_added_elements, latest_removed_elements, added_element_classes, removed_element_classes
    );
        

    /** Update the 'class' property for new vertices, elements, and faces */
    Geometry::MeshProperty<int>& class_vert_prop = _mesh->template getVertexProperty<int>("class");
    Geometry::MeshProperty<int>& class_face_prop = _mesh->template getFaceProperty<int>("class");
    
    for (const auto& new_vert : latest_added_vertices)
    {
        class_vert_prop.set(new_vert.index, refined_elem_class);
    }
    for (const auto& new_face : latest_added_faces)
    {
        class_face_prop.set(new_face, refined_elem_class);
    }
    for (const auto& new_elem : latest_added_elements)
    {
        class_elem_prop.set(new_elem, refined_elem_class);
    }

    /** Update the 'on_cut_surface' property for new faces (should always be false when refining).
     * TODO: UNLESS WE'RE REFINING A FACE ALREADY ON THE CUT SURFACE
     */
    Geometry::MeshProperty<bool>& on_cut_surface_prop = _mesh->template getFaceProperty<bool>("on-cut-surface");
    for (const auto& new_face_index : latest_added_faces)
    {
        on_cut_surface_prop.set(new_face_index, false);
    }
    
}

template<bool IsFirstOrder, typename SolverType, typename... ConstraintTypes>
void XPBDMeshObject_<IsFirstOrder, SolverType, TypeList<ConstraintTypes...>>::coarsenElement(int elem_index, int coarsening_level, bool absolute)
{
    if (!tetMesh()->elements().indexValid(elem_index))
    {
        return;
    }

    bool coarsened = refinedTetMesh()->coarsenElement(elem_index, coarsening_level, absolute);
    if (!coarsened)
    {
        return;
    }

    /** Get the newest added/removed vertices/hanging vertices/faces/elements */
    const std::vector<Geometry::RefinedTetMesh::NewVertex>& latest_added_vertices = refinedTetMesh()->latestAddedVertices();
    const std::vector<Geometry::RefinedTetMesh::RemovedVertex>& latest_removed_vertices = refinedTetMesh()->latestRemovedVertices();
    const std::vector<int>& latest_added_faces = refinedTetMesh()->latestAddedFaces();
    const std::vector<int>& latest_added_elements = refinedTetMesh()->latestAddedElements();
    const std::vector<Geometry::TetMesh::RemovedElement>& latest_removed_elements = refinedTetMesh()->latestRemovedElements();
    const std::vector<Geometry::RefinedTetMesh::NewVertex>& latest_added_hanging_vertices = refinedTetMesh()->latestAddedHangingVertices();
    const std::vector<int>& latest_removed_hanging_vertices = refinedTetMesh()->latestRemovedHangingVertices();

    // create the vector for new element classes
    Geometry::MeshProperty<int>& class_elem_prop = tetMesh()->template getElementProperty<int>("class");
    int coarsened_elem_class = class_elem_prop.get(elem_index);

    std::vector<int> added_element_classes(latest_added_elements.size());
    for (unsigned i = 0; i < latest_added_elements.size(); i++)
    {
        added_element_classes[i] = coarsened_elem_class;
    }
    
    // create the vector for removed element classes
    std::vector<int> removed_element_classes(latest_removed_elements.size());
    for (unsigned i = 0; i < latest_removed_elements.size(); i++)
    {
        removed_element_classes[i] = coarsened_elem_class;
    }

    // update everything based for the latest mesh topology change(s)
    _updateAfterMeshTopologyChange(
        latest_added_vertices, latest_removed_vertices, latest_added_hanging_vertices, latest_removed_hanging_vertices,
        latest_added_faces, latest_added_elements, latest_removed_elements, added_element_classes, removed_element_classes
    );

    /** Update the 'class' property for new vertices, elements, and faces */
    Geometry::MeshProperty<int>& class_vert_prop = _mesh->template getVertexProperty<int>("class");
    Geometry::MeshProperty<int>& class_face_prop = _mesh->template getFaceProperty<int>("class");
    
    for (const auto& new_vert : latest_added_vertices)
    {
        class_vert_prop.set(new_vert.index, coarsened_elem_class);
    }
    for (const auto& new_face : latest_added_faces)
    {
        class_face_prop.set(new_face, coarsened_elem_class);
    }
    for (const auto& new_elem : latest_added_elements)
    {
        class_elem_prop.set(new_elem, coarsened_elem_class);
    }

    /** Update the 'on_cut_surface' property for new faces (should always be false when refining).
     * TODO: UNLESS WE'RE REFINING A FACE ALREADY ON THE CUT SURFACE
     */
    Geometry::MeshProperty<bool>& on_cut_surface_prop = _mesh->template getFaceProperty<bool>("on-cut-surface");
    for (const auto& new_face_index : latest_added_faces)
    {
        on_cut_surface_prop.set(new_face_index, false);
    }

}

template<bool IsFirstOrder, typename SolverType, typename... ConstraintTypes>
void XPBDMeshObject_<IsFirstOrder, SolverType, TypeList<ConstraintTypes...>>::_updateAfterMeshTopologyChange(    
    const std::vector<Geometry::RefinedTetMesh::NewVertex>& added_vertices, const std::vector<Geometry::RefinedTetMesh::RemovedVertex>& removed_vertices,
    const std::vector<Geometry::RefinedTetMesh::NewVertex>& added_hanging_vertices, const std::vector<int>& removed_hanging_vertices,
    const std::vector<int>& added_faces,
    const std::vector<int>& added_elements, const std::vector<Geometry::TetMesh::RemovedElement>& removed_elements,
    const std::vector<int>& added_element_classes, const std::vector<int>& removed_element_classes)
{
    /** Resize per-vertex vectors */
    size_t new_size = _mesh->vertices().totalSize();    // use total size since there may be gaps in the TombstoneVector
    _vertex_masses.resize(new_size);
    _vertex_velocities.resize(new_size);
    _vertex_applied_force.resize(new_size, Vec3r::Zero());
    _previous_vertices.resize(new_size);
    _is_fixed_vertex.resize(new_size);

    if constexpr(IsFirstOrder)
    {
        _vertex_B.resize(new_size);
    }

    /** Update initial values for new vertices */
    // first, iterate through newly added vertices and set their masses and volumes to be 0
    for (const auto& new_vert : added_vertices)
    {
        // initial new vertex mass, volume, and damping (if first order) to 0
        // this will be updated later
        _vertex_masses[new_vert.index] = 0;

        if constexpr (IsFirstOrder)
        {
            _vertex_B[new_vert.index] = 0;
        }

        // interpolate velocity, previous position, and initial position - new vertex is halfway between parents
        _vertex_velocities[new_vert.index] = 0.5*(_vertex_velocities[new_vert.parent1] + _vertex_velocities[new_vert.parent2]);
        _previous_vertices[new_vert.index] = 0.5*(_previous_vertices[new_vert.parent1] + _previous_vertices[new_vert.parent2]);

        _is_fixed_vertex[new_vert.index] = (_is_fixed_vertex[new_vert.parent1] && _is_fixed_vertex[new_vert.parent2]);
    }


    /** Update mass and volumes */
    // first, for the element that was refined, subtract 1/4 the nominal element volume from the vertices it touches
    for (unsigned i = 0; i < removed_elements.size(); i++)
    {
        const Geometry::TetMesh::RemovedElement& removed_elem = removed_elements[i];
        int removed_elem_class = removed_element_classes[i];

        // get the removed element mass from the volume * density
        const ElasticMaterial& removed_elem_material = _material_classes[removed_elem_class]->material();
        Real removed_elem_mass = removed_elem.rest_volume * removed_elem_material.density();

        for (const auto& elem_vert : removed_elem.vertices)
        {   
            // update mass for each vertex
            _vertex_masses[elem_vert] -= 0.25*removed_elem_mass;
        }
    }

    // then, iterate through new elements, and add 1/4 the nominal element volume/mass to the vertices it touches
    for (unsigned i = 0; i < added_elements.size(); i++)
    {
        int new_elem = added_elements[i];
        int new_elem_class = added_element_classes[i];

        const Vec4i& elem_vertices = tetMesh()->element(new_elem);
        // calculate rest volume for the new tet
        const Vec3r& iv1 = refinedTetMesh()->initialVertex(elem_vertices[0]);
        const Vec3r& iv2 = refinedTetMesh()->initialVertex(elem_vertices[1]);
        const Vec3r& iv3 = refinedTetMesh()->initialVertex(elem_vertices[2]);
        const Vec3r& iv4 = refinedTetMesh()->initialVertex(elem_vertices[3]);
        Real rest_volume = tetMesh()->elementVolume(iv1, iv2, iv3, iv4);

        // calculate rest mass for the new tet
        const ElasticMaterial& material = _material_classes[new_elem_class]->material();
        Real element_mass = rest_volume * material.density();

        for (const auto& elem_vert : elem_vertices)
        {
            // update mass for each vertex
            _vertex_masses[elem_vert] += 0.25*element_mass;
        }
    }


    // for 1st-order objects, calculate per-vertex damping
    /** TODO: Think about this a bit more...
     * 
     * For now, just assign per-vertex damping to the new vertices
     */
    if constexpr (IsFirstOrder)
    {
        // // if adjust_b_to_material is used, we must calculate the appropriate E and nu at each vertex in the old element
        // // the new vertices will all have E and nu corresponding to the element that was refined
        // for (const auto& removed_elem : latest_removed_elements)
        // {
        //     Vec4r vertex_E = Vec4r::Zero();
        //     Vec4r vertex_nu = Vec4r::Zero();

        //     for (int i = 0; i < 4; i++)
        //     {
        //         vertex_E[i] 
        //     }
        // }
        for (const auto& new_vert : added_vertices)
        {
            if (_adjust_b_to_material)
            {                
                /** TODO: this is a temporary, lazy solution to use just the first new element class as the class for all new vertices.
                 * Works for mesh refinement, but may not work for other topology changes...
                 * 
                 * 
                 */
                const ElasticMaterial& material = _material_classes[added_element_classes[0]]->material();
                _vertex_B[new_vert.index] = tetMesh()->vertexRestVolume(new_vert.index) * _damping_multiplier * material.E() / material.nu();
            }
            else
            {
                _vertex_B[new_vert.index] = tetMesh()->vertexRestVolume(new_vert.index) * _damping_multiplier;
            }
        }
    }

    /** Remove collision constraints associated with removed elements */
    for (const auto& elem_index : removed_elements)
    {
        auto cc_proj_range = _element_to_collision_proj_index.equal_range(elem_index.index);
        for (auto it = cc_proj_range.first; it != cc_proj_range.second; it++)
        {
            using CollisionProjectorType = Solver::ConstraintProjector<IsFirstOrder, Solver::StaticDeformableCollisionConstraint>;
            _solver.template setProjectorValidity<CollisionProjectorType>(it->second, false);
        }

        // remove map entries
        _element_to_collision_proj_index.erase(elem_index.index);
    }

    /** Remove offset attachment constraints associated with removed vertices */
    for (const auto& vert_index : removed_vertices)
    {
        auto oa_proj_range = _vertex_to_offset_attachment_proj_index.equal_range(vert_index.index);
        for (auto it = oa_proj_range.first; it != oa_proj_range.second; it++)
        {
            using CollisionProjectorType = Solver::ConstraintProjector<IsFirstOrder, Solver::OffsetAttachmentConstraint>;
            _solver.template setProjectorValidity<CollisionProjectorType>(it->second, false);
        }
    }

    // remove attachment constraints associated with removed vertices
    for (const auto& removed_vert : removed_vertices)
    {
        auto a_proj_range = _vertex_to_attachment_proj_index.equal_range(removed_vert.index);
        for (auto it = a_proj_range.first; it != a_proj_range.second; it++)
        {
            using ProjectorType = Solver::ConstraintProjector<IsFirstOrder, Solver::AttachmentConstraint>;
            _solver.template setProjectorValidity<ProjectorType>(it->second, false);
        }

    }

    /** Run collision detection on newly added faces */
    _sim->collisionScene()->collideObjectsWithFacesOfXPBDMeshObj(this, added_faces);
    // convert the ConstraintProjectorReferenceWrapper to ConstraintProjectorReferences
    // TOOD: this kinda sucks. code smell. Needs rewrite.
    // typename SolverType::projector_reference_container_type new_proj_refs;
    // for (const auto& wrapper : new_proj_ref_wrappers)
    // {
    //     auto* new_proj_ref = wrapper.template getAs<IsFirstOrder>();
    //     using CollisionProjectorType = Solver::ConstraintProjector<IsFirstOrder, Solver::StaticDeformableCollisionConstraint>;
    //     using CollisionProjRefType = Solver::ConstraintProjectorReference<CollisionProjectorType>;
    //     new_proj_refs.template push_back<CollisionProjRefType>(*new_proj_ref);
    // }
    // _solver.solve(new_proj_refs, 1, true);

    // std::cout << "# added faces: " << added_faces.size() << std::endl;
    // std::cout << "# new collision constraints: " << new_proj_ref_wrappers.size() << std::endl;

    /** Update constraints and constraint projectors. */
    // if the constraint configuration is StableNeohookean, add separate constraint projectors for the hydrostatic and deviatoric constraints
    if constexpr (std::is_same_v<typename SolverType::projector_type_list, typename XPBDMeshObjectConstraintConfigurations<IsFirstOrder>::StableNeohookean::projector_type_list>)
    {
        // resize the constraint projectors in the Solver
        size_t total_num_elements = tetMesh()->elements().totalSize();
        using DevProjectorType = Solver::ConstraintProjector<IsFirstOrder, Solver::DeviatoricConstraint>;
        using HydProjectorType = Solver::ConstraintProjector<IsFirstOrder, Solver::HydrostaticConstraint>;
        _solver.template resizeProjectorsOfType<DevProjectorType>(total_num_elements);
        _solver.template resizeProjectorsOfType<HydProjectorType>(total_num_elements);

        // invalidate constraint projectors associated with the elements that were removed
        for (unsigned i = 0; i < removed_elements.size(); i++)
        {
            _solver.template setProjectorValidity<DevProjectorType>(removed_elements[i].index, false);
            _solver.template setProjectorValidity<HydProjectorType>(removed_elements[i].index, false);
        }
        

        // add constraints and constraint projectors for new elements
        std::vector<Solver::HydrostaticConstraint>& hyd_constraint_vec = _constraints.template get<Solver::HydrostaticConstraint>();
        std::vector<Solver::DeviatoricConstraint>& dev_constraint_vec = _constraints.template get<Solver::DeviatoricConstraint>();
        hyd_constraint_vec.resize(total_num_elements);
        dev_constraint_vec.resize(total_num_elements);
        for (unsigned i = 0; i < added_elements.size(); i++)
        {
            int new_elem_index = added_elements[i];
            int new_elem_class = added_element_classes[i];
            const ElasticMaterial& new_elem_material = _material_classes[new_elem_class]->material();

            // get the vertices for the element
            const Vec4i& element = tetMesh()->element(new_elem_index);
            const int v0 = element[0];
            const int v1 = element[1];
            const int v2 = element[2];
            const int v3 = element[3];

            Geometry::Mesh::vertices_vec_type* vec_ptr = &_mesh->vertices();

            Real m0 = vertexConstraintInertia(v0);
            Real m1 = vertexConstraintInertia(v1);
            Real m2 = vertexConstraintInertia(v2);
            Real m3 = vertexConstraintInertia(v3);
            
            const Mat3r& Q = tetMesh()->elementInvUndeformedBasis(new_elem_index);
            Real rest_volume = tetMesh()->elementRestVolume(new_elem_index);
            hyd_constraint_vec[new_elem_index] = Solver::HydrostaticConstraint(v0, vec_ptr, m0, v1, vec_ptr, m1, v2, vec_ptr, m2, v3, vec_ptr, m3, new_elem_material, Q, rest_volume);
            dev_constraint_vec[new_elem_index] = Solver::DeviatoricConstraint(v0, vec_ptr, m0, v1, vec_ptr, m1, v2, vec_ptr, m2, v3, vec_ptr, m3, new_elem_material, Q, rest_volume);
        
            using HydConstraintRefType = Solver::ConstraintReference<Solver::HydrostaticConstraint>;
            using DevConstraintRefType = Solver::ConstraintReference<Solver::DeviatoricConstraint>;
            _solver.setConstraintProjector(new_elem_index, _sim->dt(), HydConstraintRefType(hyd_constraint_vec, new_elem_index));
            _solver.setConstraintProjector(new_elem_index, _sim->dt(), DevConstraintRefType(dev_constraint_vec, new_elem_index));
        }
        
    }
    // if the constraint configuration is StableNeohookeanCombined, add a combined constraint projector for the hydrostatic and deviatoric constraints
    if constexpr (std::is_same_v<typename SolverType::projector_type_list, typename XPBDMeshObjectConstraintConfigurations<IsFirstOrder>::StableNeohookeanCombined::projector_type_list>)
    {
        // resize the constraint projectors in the Solver
        size_t total_num_elements = tetMesh()->elements().totalSize();
        using ProjectorType = Solver::CombinedConstraintProjector<IsFirstOrder, Solver::DeviatoricConstraint, Solver::HydrostaticConstraint>;
        _solver.template resizeProjectorsOfType<ProjectorType>(total_num_elements);

        // invalidate constraint projectors associated with the element that was refined (removed)
        for (unsigned i = 0; i < removed_elements.size(); i++)
        {
            _solver.template setProjectorValidity<ProjectorType>(removed_elements[i].index, false);
        }
        

        // add constraints and constraint projectors for new elements
        std::vector<Solver::HydrostaticConstraint>& hyd_constraint_vec = _constraints.template get<Solver::HydrostaticConstraint>();
        std::vector<Solver::DeviatoricConstraint>& dev_constraint_vec = _constraints.template get<Solver::DeviatoricConstraint>();
        hyd_constraint_vec.resize(total_num_elements);
        dev_constraint_vec.resize(total_num_elements);
        for (unsigned i = 0; i < added_elements.size(); i++)
        {
            int new_elem_index = added_elements[i];
            int new_elem_class = added_element_classes[i];
            const ElasticMaterial& new_elem_material = _material_classes[new_elem_class]->material();

            // get the vertices for the element
            const Vec4i& element = tetMesh()->element(new_elem_index);
            const int v0 = element[0];
            const int v1 = element[1];
            const int v2 = element[2];
            const int v3 = element[3];

            Geometry::Mesh::vertices_vec_type* vec_ptr = &_mesh->vertices();

            Real m0 = vertexConstraintInertia(v0);
            Real m1 = vertexConstraintInertia(v1);
            Real m2 = vertexConstraintInertia(v2);
            Real m3 = vertexConstraintInertia(v3);
            
            const Mat3r& Q = tetMesh()->elementInvUndeformedBasis(new_elem_index);
            Real rest_volume = tetMesh()->elementRestVolume(new_elem_index);
            hyd_constraint_vec[new_elem_index] = Solver::HydrostaticConstraint(v0, vec_ptr, m0, v1, vec_ptr, m1, v2, vec_ptr, m2, v3, vec_ptr, m3, new_elem_material, Q, rest_volume);
            dev_constraint_vec[new_elem_index] = Solver::DeviatoricConstraint(v0, vec_ptr, m0, v1, vec_ptr, m1, v2, vec_ptr, m2, v3, vec_ptr, m3, new_elem_material, Q, rest_volume);
        
            using HydConstraintRefType = Solver::ConstraintReference<Solver::HydrostaticConstraint>;
            using DevConstraintRefType = Solver::ConstraintReference<Solver::DeviatoricConstraint>;
            _solver.setConstraintProjector(new_elem_index, _sim->dt(),  
                DevConstraintRefType(dev_constraint_vec, new_elem_index),
                HydConstraintRefType(hyd_constraint_vec, new_elem_index)
            );
        }
    }


    /** Update the midpoint constraints for hanging vertices */

    // std::cout << "   Added hanging verts: (";
    // for (const auto& v : added_hanging_vertices)
    // {
    //     std::cout << v.index << ", ";
    // }
    // std::cout << ")" << std::endl;

    // std::cout << "   Removed hanging verts: (";
    // for (const auto& v : removed_hanging_vertices)
    // {
    //     std::cout << v << ", ";
    // }
    // std::cout << ")" << std::endl;

    // add new hanging vertices
    // IMPORTANT: do this before removing the latest removed hanging vertices
    //  Sometimes when coarsening, the same hanging vertex is added and removed within the same coarsening operation
    //  So if we remove the latest removed hanging vertices first, the hanging vertex may not exist causing an error
    //  It does create a little bit of redundant work, but as of right now that's just the nature of the algorithm :)
    std::vector<Solver::MidpointConstraint>& midpoint_constraint_vec = _constraints.template get<Solver::MidpointConstraint>();
    for (const auto& new_hanging_vert : added_hanging_vertices)
    {
        int new_vector_index = _hanging_vertices_vec.push_back(new_hanging_vert.index);

        // std::cout << "   Added hanging vert " << new_hanging_vert.index << " at index " << new_vector_index << std::endl;
        // add entry in vertex index -> hanging vertex index map
        _vertex_to_hanging_index.insert({new_hanging_vert.index, new_vector_index});

        // resize the constraint vector (do we really have to do this every loop iteration?)
        midpoint_constraint_vec.resize(_hanging_vertices_vec.totalSize());
        using MidProjector = Solver::ConstraintProjector<IsFirstOrder, Solver::MidpointConstraint>;
        _solver.template resizeProjectorsOfType<MidProjector>(_hanging_vertices_vec.totalSize());

        // create constraint and constraint projector
        const int v1 = new_hanging_vert.index;
        const int v2 = new_hanging_vert.parent1;
        const int v3 = new_hanging_vert.parent2;

        Geometry::Mesh::vertices_vec_type* vec_ptr = &_mesh->vertices();

        Real m1 = vertexConstraintInertia(v1);
        Real m2 = vertexConstraintInertia(v2);
        Real m3 = vertexConstraintInertia(v3);

        midpoint_constraint_vec[new_vector_index] = Solver::MidpointConstraint(v1, vec_ptr, m1, v2, vec_ptr, m2, v3, vec_ptr, m3);

        using MidConstraintRefType = Solver::ConstraintReference<Solver::MidpointConstraint>;
        _solver.setConstraintProjector(new_vector_index, _sim->dt(), MidConstraintRefType(midpoint_constraint_vec, new_vector_index));
    }

    // remove hanging vertices from its vector and remove the associated MidpointConstraint
    for (const auto& removed_hanging_vert : removed_hanging_vertices)
    {
        // std::cout << "   Removed hanging vert " << removed_hanging_vert << std::endl;
        int vector_index = _vertex_to_hanging_index.at(removed_hanging_vert);

        // remove from the hanging vertices vector
        _hanging_vertices_vec.erase(vector_index);

        // remove from the vertex index -> hanging vertex index map
        _vertex_to_hanging_index.erase(removed_hanging_vert);

        // set the projector as invalid
        using MidProjector = Solver::ConstraintProjector<IsFirstOrder, Solver::MidpointConstraint>;
        _solver.template setProjectorValidity<MidProjector>(vector_index, false);

        // don't have to explicitly remove the constraint from the constraint vector - we will just overwrite later
    }
}


template<bool IsFirstOrder, typename SolverType, typename... ConstraintTypes>
Real XPBDMeshObject_<IsFirstOrder, SolverType, TypeList<ConstraintTypes...>>::totalStrainEnergy() const
{
    // iterate over all hydrostatic and deviatoric constraints
    Real total_energy = 0;
    _constraints.template for_each_element<Solver::DeviatoricConstraint, Solver::HydrostaticConstraint>([&total_energy](const auto& constraint){
        Real eval;
        constraint.evaluate(&eval);
        total_energy += eval * eval / constraint.alpha();
    });

    return total_energy;
}

template<bool IsFirstOrder, typename SolverType, typename... ConstraintTypes>
Vec3r XPBDMeshObject_<IsFirstOrder, SolverType, TypeList<ConstraintTypes...>>::elasticForceAtVertex(int index) const
{
    // get elements attached to the vertex in the mesh
    const std::vector<int>& _attached_elements = tetMesh()->vertexAttachedElements(index);

    /** TODO: figure out which approach is correct. */
    Vec3r total_force = Vec3r::Zero();
    Vec3r total_force_proj = Vec3r::Zero();
    for (const auto& elem_index : _attached_elements)
    {
        // std::cout << "Elastic force for element " << elem_index << std::endl;
        const Vec3r& dev_force = _constraints.template get<Solver::DeviatoricConstraint>()[elem_index].elasticForce(index);
        const Vec3r& hyd_force = _constraints.template get<Solver::HydrostaticConstraint>()[elem_index].elasticForce(index);

        Vec3r proj_force = Vec3r::Zero();

        if constexpr (std::is_same_v<typename SolverType::projector_type_list, typename XPBDMeshObjectConstraintConfigurations<IsFirstOrder>::StableNeohookeanCombined::projector_type_list>)
        {
            const auto& proj = 
                _solver.template getConstraintProjector<Solver::CombinedConstraintProjector<IsFirstOrder, Solver::DeviatoricConstraint, Solver::HydrostaticConstraint>>(elem_index);
            std::vector<Vec3r> proj_forces = proj.constraintForces();
            const std::vector<Solver::PositionReference>& positions = proj.positions();
            
            for (unsigned i = 0; i < positions.size(); i++)
            {
                if (positions[i].index == index)
                {
                    proj_force = proj_forces[i];
                    break;
                }
            }
        }

        // TODO: THIS IS A HACK THAT WILL PROBABLY BITE ME IN THE ASS LATER
        // for some reason, very small elements produce incorrect forces (they are very large, probably due to machine precision limits) - which messes up force feedback in the Haptic demos
        // need to find a better fix than this
        if (_constraints.template get<Solver::DeviatoricConstraint>()[elem_index].restVolume() > 1e-10)
        {
            total_force += dev_force + hyd_force;
            total_force_proj += proj_force;
        } 
            
        // else
        //     std::cout << "LARGE FORCE ELEMENT VOLUME: " << _constraints.template get<Solver::DeviatoricConstraint>()[elem_index].restVolume() << std::endl;
        // std::cout << "Forces at element " << elem_index << ": (" << dev_force[0] << ", " << dev_force[1] << ", " << dev_force[2] << ") Hyd: ("<< hyd_force[0] << ", " << hyd_force[1] << ", " << hyd_force[2] << ")" << std::endl;
        
    }

    // std::cout << ""

    std::cout << "\nTotal elastic force at vertex (from constraints): " << total_force[0] << ", " << total_force[1] << ", " << total_force[2] << std::endl;
    std::cout << "Total elastic force at vertex (from projectors): " << total_force_proj[0] << ", " << total_force_proj[1] << ", " << total_force_proj[2] << std::endl;

    return total_force;
}

template<bool IsFirstOrder, typename SolverType, typename... ConstraintTypes>
Eigen::SparseMatrix<Real> XPBDMeshObject_<IsFirstOrder, SolverType, TypeList<ConstraintTypes...>>::stiffnessMatrix() const
{
    // auto t_start = std::chrono::high_resolution_clock::now();

    // the stiffness matrix only concerns the elastic constraints - i.e. the hydrostatic and deviatoric constraints
    // With adaptive mesh refinement, there may be some "inactive" constraints in the constraint vector, since
    //    the index of each hydrostatic and deviatoric constraint in the respective constraint arrays is the index of the element
    //    with those constraints in the element array.
    //
    // Thus, we iterate through the elements instead of the constraints themselves, and only compute contributions for constraints that correspond to
    //    valid elements.

    /** Step 1: Assemble global delC matrix */

    // number of elastic constraints = number of active elements * 2
    size_t num_elastic_constraints = 2*tetMesh()->numElements();

    // allocate space for the constraint vector and constraint compliances vector
    _stiffness_matrix_C_vec = VecXr::Zero(num_elastic_constraints);
    _stiffness_matrix_alpha_inv_vec = VecXr::Zero(num_elastic_constraints);

    // auto t_alloc = std::chrono::high_resolution_clock::now();
    // double alloc_ms = std::chrono::duration_cast<std::chrono::nanoseconds>(t_alloc - t_start).count() / 1.0e6;
    // std::cout << "Zeroing C vec and alpha inv vec took: " << alloc_ms << " ms" << std::endl;

    // allocate space for the triplets used to build the sparse delC matrix
    std::vector<Eigen::Triplet<Real>> delC_triplets;
    delC_triplets.reserve(12*num_elastic_constraints);   // each constraint gradient has 12 nonzeros

    // auto t_alloc_delC = std::chrono::high_resolution_clock::now();
    // double alloc_delC_ms = std::chrono::duration_cast<std::chrono::nanoseconds>(t_alloc_delC - t_alloc).count() / 1.0e6;
    // std::cout << "Zeroing C vec and alpha inv vec took: " << alloc_delC_ms << " ms" << std::endl;

    // helper function to put constraint graidnet into global delC matrix
    auto process_constraint = [&](const auto& constraint, int constraint_index)
    {
        // get the gradient from the constraint
        using ConstraintType = std::remove_cv_t<std::remove_reference_t<decltype(constraint)>>;
        Real grad[ConstraintType::NUM_COORDINATES];
        Real C;
        constraint.evaluateWithGradient(&C, grad);

        // get the positions that the constraint affects
        const std::vector<Solver::PositionReference>& constraint_positions = constraint.positions();

        for (unsigned i = 0; i < ConstraintType::NUM_POSITIONS; i++)
        {
            int dof = 3*constraint_positions[i].index;
            const Vec3r grad_i = Eigen::Map<Vec3r>(grad + 3*i);

            // _stiffness_matrix_orig_delC.block<1,3>(constraint_index, 3*position_index) = grad_i;
            delC_triplets.emplace_back(constraint_index, dof,   grad_i[0]);
            delC_triplets.emplace_back(constraint_index, dof+1, grad_i[1]);
            delC_triplets.emplace_back(constraint_index, dof+2, grad_i[2]);
        }

        // add constraint stiffness to alpha
        _stiffness_matrix_alpha_inv_vec[constraint_index] = 1.0/constraint.alpha();

        _stiffness_matrix_C_vec[constraint_index] = C;
    };

    // iterate through each constraint and put its gradient into the global delC matrix
    const Geometry::TetMesh::elements_vec_type& elements = tetMesh()->elements();
    const std::vector<Solver::HydrostaticConstraint>& hyd_constraints = _constraints.template get<Solver::HydrostaticConstraint>();
    const std::vector<Solver::DeviatoricConstraint>& dev_constraints = _constraints.template get<Solver::DeviatoricConstraint>();

    // keep track of the constraint index that we are on
    int constraint_index = 0;
    for (const auto& elem_index : elements.validIndices())
    {
        process_constraint(hyd_constraints[elem_index], constraint_index);
        constraint_index++;
        process_constraint(dev_constraints[elem_index], constraint_index);
        constraint_index++;
    }

    // auto t_delC = std::chrono::high_resolution_clock::now();
    // double delC_ms = std::chrono::duration_cast<std::chrono::nanoseconds>(t_delC - t_alloc_delC).count() / 1.0e6;
    // std::cout << "Computing delC triplets took: " << delC_ms << " ms" << std::endl;

    // build delC matrix from triplets
    Eigen::SparseMatrix<Real> delC(num_elastic_constraints, 3*_mesh->vertices().totalSize());
    delC.setFromTriplets(delC_triplets.begin(), delC_triplets.end());

    // auto t_delC_ass = std::chrono::high_resolution_clock::now();
    // double delC_ass_ms = std::chrono::duration_cast<std::chrono::nanoseconds>(t_delC_ass - t_delC).count() / 1.0e6;
    // std::cout << "Assembling delC triplets took: " << delC_ass_ms << " ms" << std::endl;

    /** Step 2: compute the Hessian term */

    // compute the Hessian term
    std::vector<Eigen::Triplet<Real>> hessian_triplets;
    hessian_triplets.reserve(144*num_elastic_constraints + 3*_mesh->vertices().totalSize());

    // "fix" all the fixed vertices in the mesh by adding a large amount to the diagonal corresponding to their 3 DOF
    for (const auto& vertex_index : _mesh->vertices().validIndices())
    {
        if (vertexFixed(vertex_index))
        {
            for (int j = 0; j < 3; j++)
                hessian_triplets.emplace_back(3*vertex_index+j,3*vertex_index+j, 1e9);
        }
    }

    // auto t_hess = std::chrono::high_resolution_clock::now();
    // double hess_ms = std::chrono::duration_cast<std::chrono::nanoseconds>(t_hess - t_delC_ass).count() / 1.0e6;
    // std::cout << "Reserving Hessian triplets took: " << hess_ms << " ms" << std::endl;

    // helper lambda for computing Hessian term of constraint
    auto compute_hessian = [&](const auto& constraint, int constraint_index)
    {
        // get the 12x12 Hessian matrix from the constraint
        using ConstraintType = std::remove_cv_t<std::remove_reference_t<decltype(constraint)>>;

        // Real alpha_inv = _stiffness_matrix_alpha_inv_vec[constraint_index];
        Real C = _stiffness_matrix_C_vec[constraint_index];
        typename ConstraintType::HessianMatType hessian_mat = constraint.hessian() * C / constraint.alpha();

        // scatter the constraint-specific Hessian mat to the appropriate DOF in the global Hessian matrix
        for (int i = 0; i < ConstraintType::NUM_POSITIONS; i++)
        {
            int dof_i = constraint.positions()[i].index;
            for (int j = 0; j < ConstraintType::NUM_POSITIONS; j++)
            {
                int dof_j = constraint.positions()[j].index;
                
                // _stiffness_matrix_hessian_term.block<3,3>(3*dof_i, 3*dof_j) += hessian_mat.template block<3,3>(3*i, 3*j);
                for (int a = 0; a < 3; a++)
                {
                    for (int b = 0; b < 3; b++)
                    {
                        hessian_triplets.emplace_back(3*dof_i + a, 3*dof_j + b, hessian_mat(3*i + a, 3*j + b));
                    }
                }
            }
        }
    };

    constraint_index = 0;
    for (const auto& elem_index : elements.validIndices())
    {
        compute_hessian(hyd_constraints[elem_index], constraint_index);
        constraint_index++;
        compute_hessian(dev_constraints[elem_index], constraint_index);
        constraint_index++;
    }

    // auto t_hess_triplet = std::chrono::high_resolution_clock::now();
    // double hess_triplet_ms = std::chrono::duration_cast<std::chrono::nanoseconds>(t_hess_triplet - t_hess).count() / 1.0e6;
    // std::cout << "Computing Hessian triplets took: " << hess_triplet_ms << " ms" << std::endl;

    // std::cout << "144*num_elastic_constraints: " << 144*num_elastic_constraints << ", hessian_triplets size: " << hessian_triplets.size() << std::endl;

    /** Step 3: Assemble stiffness matrix */
    // start with Hessian terms
    Eigen::SparseMatrix<Real> stiffness_matrix(3*_mesh->vertices().totalSize(), 3*_mesh->vertices().totalSize());
    stiffness_matrix.setFromTriplets(hessian_triplets.begin(), hessian_triplets.end());

    // auto t_hess_ass = std::chrono::high_resolution_clock::now();
    // double hess_ass_ms = std::chrono::duration_cast<std::chrono::nanoseconds>(t_hess_ass - t_hess_triplet).count() / 1.0e6;
    // std::cout << "Assembling Hessian triplets took: " << hess_ass_ms << " ms" << std::endl;

    // std::cout << "NEW stiffness matrix hessian term:\n" << _stiffness_matrix_hessian_term << std::endl;

    // then add the delC^T * alpha^{-1} * delC term
    stiffness_matrix += (delC.transpose() * _stiffness_matrix_alpha_inv_vec.asDiagonal() * delC);

    // auto t_add = std::chrono::high_resolution_clock::now();
    // double add_ms = std::chrono::duration_cast<std::chrono::nanoseconds>(t_add - t_hess_ass).count() / 1.0e6;
    // std::cout << "Adding stiffness matrix terms took: " << add_ms << " ms" << std::endl;

    return stiffness_matrix;
}

template<bool IsFirstOrder, typename SolverType, typename... ConstraintTypes>
void XPBDMeshObject_<IsFirstOrder, SolverType, TypeList<ConstraintTypes...>>::selfCollisionCheck()
{
    const Geometry::EmbreeScene* embree_scene = _sim->embreeScene();
    for (int i = 0; i < _mesh->numVertices(); i++)
    {
        if (!_mesh->vertexOnSurface(i))
            continue;

        std::set<Geometry::EmbreePQHit> hits = embree_scene->tetMeshSelfCollisionQuery(i, this);
        if (hits.size() > 0)
        {
            int face_index = _sdf->closestSurfaceFaceToPointInTet(_mesh->vertex(i), hits.begin()->prim_index);

            if (face_index < 0)
                continue;

            const Eigen::Vector3i& face = _mesh->face(face_index);

            // Real* q_ptr = _mesh->vertexPointer(i);
            // Real* p1_ptr = _mesh->vertexPointer(face[0]);
            // Real* p2_ptr = _mesh->vertexPointer(face[1]);
            // Real* p3_ptr = _mesh->vertexPointer(face[2]);

            Geometry::Mesh::vertices_vec_type* vec_ptr = &_mesh->vertices();

            Real qm = vertexConstraintInertia(i);
            Real p1m = vertexConstraintInertia(face[0]);
            Real p2m = vertexConstraintInertia(face[1]);
            Real p3m = vertexConstraintInertia(face[2]);

            
            // std::cout << "  SELF COLLISION WITH VERTEX " << i << " WITH FACE " << face_index << "!" << std::endl;
            // std::cout << "  Tet indices: " << tetMesh()->element(hits.begin()->prim_index).transpose() << std::endl;
            // std::cout << "  Face indices: " << face.transpose() << std::endl;
            // std::cout << "  Vertex: " << _mesh->vertex(i).transpose() << 
            //     "  Face:\n\t" << _mesh->vertex(face[0]).transpose() << ",\n\t" << _mesh->vertex(face[1]).transpose()  << ",\n\t" << _mesh->vertex(face[2]).transpose() << std::endl;
            std::vector<Solver::DeformableDeformableCollisionConstraint>& constraint_vec = _constraints.template get<Solver::DeformableDeformableCollisionConstraint>();
            constraint_vec.emplace_back(i, vec_ptr, qm, face[0], vec_ptr, p1m, face[1], vec_ptr, p2m, face[2], vec_ptr, p3m);

            using ConstraintRefType = Solver::ConstraintReference<Solver::DeformableDeformableCollisionConstraint>;
            _solver.addConstraintProjector(_sim->dt(), ConstraintRefType(constraint_vec, constraint_vec.size()-1));
        }
    }
    
}

template<bool IsFirstOrder, typename SolverType, typename... ConstraintTypes>
typename SolverType::projector_reference_container_type
XPBDMeshObject_<IsFirstOrder, SolverType, TypeList<ConstraintTypes...>>::_gatherProjectorsForLocalCollisionIterations()
{
    // create a container to store all the constraint projectors that we should re-project
    typename SolverType::projector_reference_container_type proj_to_reproject;

    // add all midpoint constraints to be re-projected - midpoint constraints (i.e. hanging nodes) are really only generated in collision areas
    /** TODO: be more selective about which midpoint constraints get reprojected
     * 
     * 
     * 
     */
    using MidProjectorType = Solver::ConstraintProjector<IsFirstOrder, Solver::MidpointConstraint>;
    using MidProjectorTypeRef = Solver::ConstraintProjectorReference<MidProjectorType>;
    std::vector<MidProjectorType>& midpoint_projectors = _solver.template getConstraintProjectorsOfType<MidProjectorType>();
    for (unsigned i = 0; i < midpoint_projectors.size(); i++)
    {
        proj_to_reproject.template emplace_back<MidProjectorTypeRef>(midpoint_projectors, i);
    }

    // go through each collision constraint and find the ones that were actually projected (lambda != 0)
    using StaticCollisionProjectorType = Solver::ConstraintProjector<IsFirstOrder, Solver::StaticDeformableCollisionConstraint>;
    using StaticCollisionProjectorTypeRef = Solver::ConstraintProjectorReference<StaticCollisionProjectorType>;
    std::vector<StaticCollisionProjectorType>& collision_projectors = _solver.template getConstraintProjectorsOfType<StaticCollisionProjectorType>();

    // int ring_size = 1;

    // the list of elements to whose elastic constraints we need to reproject
    // std::unordered_set<int> elements_to_reproject_old;
    // std::unordered_set<int> elements_to_reproject;
    // keep track of vertices we have already visited
    // std::unordered_set<int> visited_vertices;
    // keep track of vertices we need to visit
    // std::deque<std::pair<int, int>> vertices_to_visit;

    for (unsigned i = 0; i < collision_projectors.size(); i++)
    {
        // add all collision constraints to be re-projected - this is necessary to maintain a consistent contact set
        proj_to_reproject.template emplace_back<StaticCollisionProjectorTypeRef>(collision_projectors, i);

        // if the collision constraint was violated last frame and projected, then we want to perform local iterations in its local area
        if (collision_projectors[i].lambda() != 0)
        {
            // get the vertices affected by this collision constraint
            const std::vector<Solver::PositionReference>& positions = collision_projectors[i].positions();

            // for each of the vertices in the collision face, add them to the stack to be processed
            // for (const auto& position : positions)
            // {
            //     vertices_to_visit.push_back({position.index, 0});
            // }

            if constexpr (std::is_same_v<typename SolverType::projector_type_list, typename XPBDMeshObjectConstraintConfigurations<IsFirstOrder>::StableNeohookeanCombined::projector_type_list>)
            {
                using DevHydProjectorType = Solver::CombinedConstraintProjector<IsFirstOrder, Solver::DeviatoricConstraint, Solver::HydrostaticConstraint>;
                using DevHydProjectorTypeRef = Solver::ConstraintProjectorReference<DevHydProjectorType>;
                std::vector<DevHydProjectorType>& elastic_projectors = _solver.template getConstraintProjectorsOfType<DevHydProjectorType>();

                // for each of the positions in the collision face, get the elements that they are attached to and add the elastic per-element constraints
                // to be reprojected
                for (const auto& position : positions)
                {
                    for (const auto& element_index : tetMesh()->vertexAttachedElements(position.index))
                    {
                        // elements_to_reproject_old.insert(element_index);
                        proj_to_reproject.template emplace_back<DevHydProjectorTypeRef>(elastic_projectors, element_index);
                    }
                }
            }
            else if (std::is_same_v<typename SolverType::projector_type_list, typename XPBDMeshObjectConstraintConfigurations<IsFirstOrder>::StableNeohookean::projector_type_list>)
            {
                using DevProjectorType = Solver::ConstraintProjector<IsFirstOrder, Solver::DeviatoricConstraint>;
                using DevProjectorTypeRef = Solver::ConstraintProjectorReference<DevProjectorType>;
                using HydProjectorType = Solver::ConstraintProjector<IsFirstOrder, Solver::HydrostaticConstraint>;
                using HydProjectorTypeRef = Solver::ConstraintProjectorReference<HydProjectorType>;

                std::vector<DevProjectorType>& dev_projectors = _solver.template getConstraintProjectorsOfType<DevProjectorType>();
                std::vector<HydProjectorType>& hyd_projectors = _solver.template getConstraintProjectorsOfType<HydProjectorType>();
                for (const auto& position : positions)
                {
                    for (const auto& element_index : tetMesh()->vertexAttachedElements(position.index))
                    {
                        proj_to_reproject.template emplace_back<DevProjectorTypeRef>(dev_projectors, element_index);
                        proj_to_reproject.template emplace_back<HydProjectorTypeRef>(hyd_projectors, element_index);
                    }
                }
            }
        }
    }

    // iterate through all the vertices to visit
    // while (!vertices_to_visit.empty())
    // {
    //     auto [vertex_index, ring] = vertices_to_visit.front();
    //     vertices_to_visit.pop_front();

    //     // try insertion into the vertices we've visited
    //     auto [it, success] = visited_vertices.insert(vertex_index);

    //     // if vertex already was visited, move on
    //     if (!success)
    //         continue;

    //     // get attached elements
    //     for (const auto& element_index : tetMesh()->vertexAttachedElements(vertex_index))
    //     {
    //         elements_to_reproject.insert(element_index);
    //     }

    //     // get all adjacent vertices
    //     if (ring < ring_size)
    //     {
    //         for (const auto& adj_vert_index : tetMesh()->vertexAdjacentVertices(vertex_index))
    //         {
    //             vertices_to_visit.push_back({adj_vert_index, ring+1});
    //         }
    //     }
    // }

    // std::cout << "Old elements: ";
    // for (const auto& element_index : elements_to_reproject_old)
    // {
    //     std::cout << element_index << ", ";
    // }
    // std::cout << "\nNew elements: ";
    // for (const auto& element_index : elements_to_reproject)
    // {
    //     std::cout << element_index << ", ";
    // }
    // std::cout << std::endl;

    // now we have a list of elements to reproject
    // for (const auto& element_index : elements_to_reproject)
    // {
    //     if constexpr (std::is_same_v<typename SolverType::projector_type_list, typename XPBDMeshObjectConstraintConfigurations<IsFirstOrder>::StableNeohookeanCombined::projector_type_list>)
    //     {
    //         using DevHydProjectorType = Solver::CombinedConstraintProjector<IsFirstOrder, Solver::DeviatoricConstraint, Solver::HydrostaticConstraint>;
    //         using DevHydProjectorTypeRef = Solver::ConstraintProjectorReference<DevHydProjectorType>;
    //         std::vector<DevHydProjectorType>& elastic_projectors = _solver.template getConstraintProjectorsOfType<DevHydProjectorType>();

    //         proj_to_reproject.template emplace_back<DevHydProjectorTypeRef>(elastic_projectors, element_index);
    //     }
    //     else if (std::is_same_v<typename SolverType::projector_type_list, typename XPBDMeshObjectConstraintConfigurations<IsFirstOrder>::StableNeohookean::projector_type_list>)
    //     {
    //         using DevProjectorType = Solver::ConstraintProjector<IsFirstOrder, Solver::DeviatoricConstraint>;
    //         using DevProjectorTypeRef = Solver::ConstraintProjectorReference<DevProjectorType>;
    //         using HydProjectorType = Solver::ConstraintProjector<IsFirstOrder, Solver::HydrostaticConstraint>;
    //         using HydProjectorTypeRef = Solver::ConstraintProjectorReference<HydProjectorType>;

    //         std::vector<DevProjectorType>& dev_projectors = _solver.template getConstraintProjectorsOfType<DevProjectorType>();
    //         std::vector<HydProjectorType>& hyd_projectors = _solver.template getConstraintProjectorsOfType<HydProjectorType>();
    //         proj_to_reproject.template emplace_back<DevProjectorTypeRef>(dev_projectors, element_index);
    //         proj_to_reproject.template emplace_back<HydProjectorTypeRef>(hyd_projectors, element_index);
    //     }
    // }

    return proj_to_reproject;
}

#ifdef HAVE_CUDA

template<bool IsFirstOrder, typename SolverType, typename... ConstraintTypes>
void XPBDMeshObject_<IsFirstOrder, SolverType, TypeList<ConstraintTypes...>>::createGPUResource()
{
    if (!_gpu_resource)
    {
        _gpu_resource = std::make_unique<Sim::XPBDMeshObjectGPUResource>(this);
        _gpu_resource->allocate();
    }
}

template<bool IsFirstOrder, typename SolverType, typename... ConstraintTypes>
XPBDMeshObjectGPUResource* XPBDMeshObject_<IsFirstOrder, SolverType, TypeList<ConstraintTypes...>>::gpuResource()
{
    assert(_gpu_resource);
    // TODO: see if we can remove this dynamic_cast somehow
    return dynamic_cast<XPBDMeshObjectGPUResource*>(_gpu_resource.get());
}

template<bool IsFirstOrder, typename SolverType, typename... ConstraintTypes>
const XPBDMeshObjectGPUResource* XPBDMeshObject_<IsFirstOrder, SolverType, TypeList<ConstraintTypes...>>::gpuResource() const
{
    assert(_gpu_resource);
    // TODO: see if we can remove this dynamic_cast somehow
    return dynamic_cast<const XPBDMeshObjectGPUResource*>(_gpu_resource.get());
}
#endif

template<bool IsFirstOrder, typename SolverType, typename... ConstraintTypes>
void XPBDMeshObject_<IsFirstOrder, SolverType, TypeList<ConstraintTypes...>>::serialize(std::vector<std::byte>& buf) const
{
    XPBDMeshObject_Base_<IsFirstOrder>::serialize(buf);
    pack(buf, _constraint_type);
    pack(buf, _solver);
    pack(buf, _constraints);
    pack(buf, _hanging_vertices_vec);
    pack(buf, _vertex_to_hanging_index);
    pack(buf, _element_to_collision_proj_index);
    pack(buf, _num_local_collision_iters);
    pack(buf, _element_classes_filename);
    pack(buf, _ground_faces_filename);
    pack(buf, _compute_heat_conduction);
    pack(buf, _fixed_faces_filename);
}

template<bool IsFirstOrder, typename SolverType, typename... ConstraintTypes>
void XPBDMeshObject_<IsFirstOrder, SolverType, TypeList<ConstraintTypes...>>::deserialize(const std::byte*& buf)
{
    XPBDMeshObject_Base_<IsFirstOrder>::deserialize(buf);
    unpack(buf, _constraint_type);
    unpack(buf, _solver);
    unpack(buf, _constraints);
    unpack(buf, _hanging_vertices_vec);
    unpack(buf, _vertex_to_hanging_index);
    unpack(buf, _element_to_collision_proj_index);
    unpack(buf, _num_local_collision_iters);
    unpack(buf, _element_classes_filename);
    unpack(buf, _ground_faces_filename);
    unpack(buf, _compute_heat_conduction);
    unpack(buf, _fixed_faces_filename);
}


} // namespace Sim




/////////////////////////////////////////////////////////////////
// Explicit template instantiations
////////////////////////////////////////////////////////////////

#include "common/XPBDTypedefs.hpp"
// instantiate templates

// First attempt at automating template instantation - didn't work, bunch of linker errors.

// // Helper to instantiate XPBDMeshObject
// template<typename SolverType, typename ConstraintsTypeList>
// struct XPBDMeshObjectInstantiator
// {
//     // static constexpr void instantiate()
//     // {
//     //     // INSTANTIATE_XPBDMESHOBJECT(SolverType, ConstraintsTypeList);
//     //     return (void)sizeof
//     // }
//     // static constexpr int dummy = sizeof(XPBDMeshObject<SolverType, ConstraintsTypeList>);
//     inline static constexpr int __attribute__((used)) dummy = sizeof(XPBDMeshObject<SolverType, ConstraintsTypeList>);
// };

// template<typename SolverTypeList>
// struct InstantiateXPBDMeshObjectsFromSolverType;

// template<typename ...SolverTypes>
// struct InstantiateXPBDMeshObjectsFromSolverType<TypeList<SolverTypes...>>
// {
//     // static constexpr void instantiate()
//     // {
//     //     (XPBDMeshObjectInstantiator<SolverTypes, typename SolverTypes::constraint_type_list>::instantiate(), ...);
//     // }
//     // using expand = int[];
//     // inline static constexpr expand __attribute__((used)) dummy = {
//     //     (XPBDMeshObjectInstantiator<SolverTypes, typename SolverTypes::constraint_type_list>::dummy, 0)...
//     // };
//     std::tuple<XPBDMeshObject<SolverTypes, typename SolverTypes::constraint_type_list>...> unused;
// };

// template<typename ConstraintConfigTypeList>
// struct InstantiateAllXPBDMeshObjects;

// template<typename ...ConstraintConfigs>
// struct InstantiateAllXPBDMeshObjects<TypeList<ConstraintConfigs...>>
// {
//     // static constexpr void instantiate()
//     // {
//     //     (InstantiateXPBDMeshObjectsFromSolverType<typename XPBDMeshObjectSolverTypes<typename ConstraintConfigs::projector_type_list>::type_list>::instantiate(), ...);
//     // }
//     // using expand = int[];
//     // inline static constexpr expand __attribute__((used)) dummy = {
//     //     (InstantiateXPBDMeshObjectsFromSolverType<typename XPBDMeshObjectSolverTypes<typename ConstraintConfigs::projector_type_list>::type_list>::dummy[0], 0)...
//     // };
//     std::tuple<InstantiateXPBDMeshObjectsFromSolverType<typename XPBDMeshObjectSolverTypes<typename ConstraintConfigs::projector_type_list>::type_list>...> unused;
// };

// // inline constexpr int __attribute__((used)) enusre_instantiation = InstantiateAllXPBDMeshObjects<XPBDMeshObjectConstraintConfigurations::type_list>::dummy[0];
// // InstantiateAllXPBDMeshObjects<typename XPBDMeshObjectConstraintConfigurations::type_list>::instantiate();
// template struct InstantiateAllXPBDMeshObjects<typename XPBDMeshObjectConstraintConfigurations::type_list>;

namespace Sim {

// TODO: find a way to automate this!
using SolverTypesStableNeohookean = XPBDObjectSolverTypes<false, typename XPBDMeshObjectConstraintConfigurations<false>::StableNeohookean::projector_type_list>;
using SolverTypesStableNeohookeanCombined = XPBDObjectSolverTypes<false, typename XPBDMeshObjectConstraintConfigurations<false>::StableNeohookeanCombined::projector_type_list>;
using StableNeohookeanConstraints = typename XPBDMeshObjectConstraintConfigurations<false>::StableNeohookean::constraint_type_list;
using StableNeohookeanCombinedConstraints = typename XPBDMeshObjectConstraintConfigurations<false>::StableNeohookeanCombined::constraint_type_list;

// Stable Neohookean constraint config
template class XPBDMeshObject_<false, SolverTypesStableNeohookean::GaussSeidel, StableNeohookeanConstraints>;
template class XPBDMeshObject_<false, SolverTypesStableNeohookean::Jacobi, StableNeohookeanConstraints>;
template class XPBDMeshObject_<false, SolverTypesStableNeohookean::ParallelJacobi, StableNeohookeanConstraints>;

// Stable Neohookean Combined constraint config
template class XPBDMeshObject_<false, SolverTypesStableNeohookeanCombined::GaussSeidel, StableNeohookeanCombinedConstraints>;
template class XPBDMeshObject_<false, SolverTypesStableNeohookeanCombined::Jacobi, StableNeohookeanCombinedConstraints>;
template class XPBDMeshObject_<false, SolverTypesStableNeohookeanCombined::ParallelJacobi, StableNeohookeanCombinedConstraints>;

using FirstOrderSolverTypesStableNeohookean = XPBDObjectSolverTypes<true, typename XPBDMeshObjectConstraintConfigurations<true>::StableNeohookean::projector_type_list>;
using FirstOrderSolverTypesStableNeohookeanCombined = XPBDObjectSolverTypes<true, typename XPBDMeshObjectConstraintConfigurations<true>::StableNeohookeanCombined::projector_type_list>;
using FirstOrderStableNeohookeanConstraints = typename XPBDMeshObjectConstraintConfigurations<true>::StableNeohookean::constraint_type_list;
using FirstOrderStableNeohookeanCombinedConstraints = typename XPBDMeshObjectConstraintConfigurations<true>::StableNeohookeanCombined::constraint_type_list;
template class XPBDMeshObject_<true, FirstOrderSolverTypesStableNeohookean::GaussSeidel, FirstOrderStableNeohookeanConstraints>;
template class XPBDMeshObject_<true, FirstOrderSolverTypesStableNeohookean::Jacobi, FirstOrderStableNeohookeanConstraints>;
template class XPBDMeshObject_<true, FirstOrderSolverTypesStableNeohookean::ParallelJacobi, FirstOrderStableNeohookeanConstraints>;

template class XPBDMeshObject_<true, FirstOrderSolverTypesStableNeohookeanCombined::GaussSeidel, FirstOrderStableNeohookeanCombinedConstraints>;
template class XPBDMeshObject_<true, FirstOrderSolverTypesStableNeohookeanCombined::Jacobi, FirstOrderStableNeohookeanCombinedConstraints>;
template class XPBDMeshObject_<true, FirstOrderSolverTypesStableNeohookeanCombined::ParallelJacobi, FirstOrderStableNeohookeanCombinedConstraints>;
// CTAD
// template<typename SolverType, typename ...ConstraintTypes> XPBDMeshObject(TypeList<ConstraintTypes...>, const Simulation*, const XPBDMeshObjectConfig* config)
//     -> XPBDMeshObject<IsFirstOrder, SolverType, TypeList<ConstraintTypes...>>;

} // namespace Sim