#include "simulation/StiffnessMapSimulation.hpp"

namespace Sim
{

StiffnessMapSimulation::StiffnessMapSimulation(const Config::StiffnessMapSimulationConfig* config)
    : Simulation(config),
    _displacement_magnitude(config->displacementMagnitude()),
    _time_to_steady_state(config->timeToSteadyState()),
    _applying_force(false)
    
{
    _query_points.emplace(0, Vec3r(0.3, 0.5, 0.2));
}

void StiffnessMapSimulation::setup()
{
    Simulation::setup();

    // find the XPBDMeshObject (which we're assuming to be the tissue)
    // first check 1st-order XPBDMeshObjects
    auto& fo_xpbd_objs = _objects.template get<std::unique_ptr<FirstOrderXPBDMeshObject_Base>>();
    auto& xpbd_objs = _objects.template get<std::unique_ptr<XPBDMeshObject_Base>>();
    if (fo_xpbd_objs.size() > 0)
        _tissue_obj = fo_xpbd_objs.front().get();
    else if (xpbd_objs.size() > 0)
        _tissue_obj = xpbd_objs.front().get();
    else
        assert((xpbd_objs.size() + fo_xpbd_objs.size() > 0) && "There must be at least 1 XPBDMeshObject or FirstOrderXPBDMeshObject in the simulation (there is no tissue object!).");
}

void StiffnessMapSimulation::_timeStep()
{
    Simulation::_timeStep();

    // if we are not already applying a displacement and there is a query point in the queue, start applying a displacement
    if (!_applying_force && !_query_points.empty())
    {
        _applying_force = true;
        _dir_index = 0;

        // record the time that we started applying this force
        _displacement_application_time = _time;

        // get the query point at the front of the queue
        _cur_query_point = _query_points.front();
        _query_points.pop();

        std::cout << "\n=== Processing new query point ===" << std::endl;
        std::cout << "  Face index: " << _cur_query_point.face_ind << std::endl;
        std::cout << "  Barycentric coords: " << _cur_query_point.face_barys.transpose() << std::endl;

        

        // initialize the attach position to the initial position of the query point
        const Vec3i& face = _tissue_obj.mesh()->face(_cur_query_point.face_ind);
        const Vec3r& v1 = _tissue_obj.mesh()->vertex(face[0]);
        const Vec3r& v2 = _tissue_obj.mesh()->vertex(face[1]);
        const Vec3r& v3 = _tissue_obj.mesh()->vertex(face[2]);
        _initial_attach_position = _cur_query_point.face_barys[0]*v1 + _cur_query_point.face_barys[1]*v2 + _cur_query_point.face_barys[2]*v3;

        std::cout << "  Initial position: " << _initial_attach_position.transpose() << std::endl;

        if (_tissue_obj.vertexFixed(face[0]) || _tissue_obj.vertexFixed(face[1]) || _tissue_obj.vertexFixed(face[2]))
        {
            std::cout << "  Error: at least one of the vertices in the face is fixed! Skipping this query point..." << std::endl;
            _applying_force = false;
            return;
        }
        

        // the prescribed displacement is initially in the x direction
        _prescribed_displacement = Vec3r(_displacement_magnitude, 0, 0);
        _attach_position = _initial_attach_position + _prescribed_displacement;

        std::cout << "  Attach position: " << _attach_position.transpose() << std::endl;

        std::cout << " Creating attachment constraint..." << std::endl;
        // create the face attachment constraint
        _attachment_constraint_proj = _tissue_obj.addFaceOffsetAttachmentConstraint(
            _cur_query_point.face_ind,
            _cur_query_point.face_barys,
            &_attach_position,
            Vec3r::Zero()
        );

        std::cout << " Displacement application time: " << _displacement_application_time << std::endl;
        std::cout << " Waiting until: " << _displacement_application_time + _time_to_steady_state << std::endl;
    }

    // assume that after a certain amount of time we have reached steady state
    // calculate the force and assemble the stiffness matrix
    if (_applying_force && _time > _displacement_application_time + _time_to_steady_state)
    {
        std::cout << " Reached steady-state for direction " << _dir_index << "! Reading force..." << std::endl;
        std::vector<Vec3r> constraint_forces = _attachment_constraint_proj.constraintForces();
        Vec3r net_force = -std::reduce(constraint_forces.cbegin(), constraint_forces.cend());

        _cur_stiffness_matrix.col(_dir_index) = net_force / _displacement_magnitude;

        // reset the displacement application time
        _displacement_application_time = _time;

        // switch the prescribed displacement
        _dir_index++;
        if (_dir_index == 1)
            _prescribed_displacement = Vec3r(0, _displacement_magnitude, 0);
        else if (_dir_index == 2)
            _prescribed_displacement = Vec3r(0, 0, _displacement_magnitude);
        else
        {
            // dir_index > 2 ==> we are done
            _applying_force = false;
            _tissue_obj.clearFaceOffsetAttachmentConstraints();
            std::cout << "Stiffness matrix:\n" << _cur_stiffness_matrix << std::endl;
            return;
        }

        // update the attach position
        _attach_position = _initial_attach_position + _prescribed_displacement;
    }
    else if (_applying_force)
    {
        std::cout << " Waiting..." << std::endl;
    }
}

} // namespace Sim