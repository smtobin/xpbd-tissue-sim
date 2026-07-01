#include "simulation/StiffnessMapSimulation.hpp"

namespace Sim
{

StiffnessMapSimulation::StiffnessMapSimulation(const Config::StiffnessMapSimulationConfig* config)
    : Simulation(config),
    _force_magnitude(config->displacementMagnitude()),
    _time_to_steady_state(config->timeToSteadyState()),
    _force_application_time(-100000),
    _applying_force(false),
    _releasing_force(false)
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
    if (!_applying_force && !_releasing_force && !_query_points.empty() && _time > _force_application_time + _time_to_steady_state*2)
    {
        _applying_force = true;
        _dir_index = 0;

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
        _initial_position = _cur_query_point.face_barys[0]*v1 + _cur_query_point.face_barys[1]*v2 + _cur_query_point.face_barys[2]*v3;

        std::cout << "  Initial position: " << _initial_position.transpose() << std::endl;

        if (_tissue_obj.vertexFixed(face[0]) || _tissue_obj.vertexFixed(face[1]) || _tissue_obj.vertexFixed(face[2]))
        {
            std::cout << "  Error: at least one of the vertices in the face is fixed! Skipping this query point..." << std::endl;
            _applying_force = false;
            return;
        }

        // record the time that we started applying this force
        _force_application_time = _time;

        // apply forces proportional to barycentric coordinates
        Vec3r total_force = _force_magnitude*Vec3r(1,0,0);
        for (int i = 0; i < 3; i++)
        {
            Vec3r force_i = _cur_query_point.face_barys[i]*total_force;
            _tissue_obj.setVertexAppliedForce(face[i], force_i);
        }
    }

    // assume that after a certain amount of time we have reached steady state
    // calculate the net displacement and assemble the compliance matrix
    // then, release the applied force
    if (_applying_force && _time > _force_application_time + _time_to_steady_state)
    {
        std::cout << " Reached steady-state for direction " << _dir_index << "! Reading force..." << std::endl;
        // std::vector<Vec3r> constraint_forces = _attachment_constraint_proj.constraintForces();
        // Vec3r net_force = std::reduce(constraint_forces.cbegin(), constraint_forces.cend());

        const Vec3i& face = _tissue_obj.mesh()->face(_cur_query_point.face_ind);
        const Vec3r& v1 = _tissue_obj.mesh()->vertex(face[0]);
        const Vec3r& v2 = _tissue_obj.mesh()->vertex(face[1]);
        const Vec3r& v3 = _tissue_obj.mesh()->vertex(face[2]);
        Vec3r cur_attach_position = _cur_query_point.face_barys[0]*v1 + _cur_query_point.face_barys[1]*v2 + _cur_query_point.face_barys[2]*v3;
        Vec3r displacement = cur_attach_position - _initial_position;

        _cur_compliance_matrix.col(_dir_index) = displacement / _force_magnitude;

        for (int i = 0; i < 3; i++)
        {
            _tissue_obj.setVertexAppliedForce(face[i], Vec3r::Zero());
        }

        // release the force
        _releasing_force = true;
        _applying_force = false;
    }

    // assume that after a certain amount of time releasing the force we have reached steady state (the initial state) again
    // apply the next force
    if (_releasing_force && _time > _force_application_time + 2*_time_to_steady_state)
    {
        std::cout << " Reached steady-state after releasing force for direction " << _dir_index << "! Moving to next direction..." << std::endl;

        const Vec3i& face = _tissue_obj.mesh()->face(_cur_query_point.face_ind);
        const Vec3r& v1 = _tissue_obj.mesh()->vertex(face[0]);
        const Vec3r& v2 = _tissue_obj.mesh()->vertex(face[1]);
        const Vec3r& v3 = _tissue_obj.mesh()->vertex(face[2]);
        _initial_position = _cur_query_point.face_barys[0]*v1 + _cur_query_point.face_barys[1]*v2 + _cur_query_point.face_barys[2]*v3;

        // switch the prescribed force
        _dir_index++;
        Vec3r total_force = Vec3r::Zero();
        if (_dir_index == 1)
            total_force = _force_magnitude*Vec3r(0,1,0);
        else if (_dir_index == 2)
            total_force = _force_magnitude*Vec3r(0,0,1);
        else
        {
            // dir_index > 2 ==> we are done
            _applying_force = false;
            _releasing_force = false;
            std::cout << "Stiffness matrix:\n" << _cur_compliance_matrix << std::endl;
            _results.emplace_back(_cur_query_point, _cur_compliance_matrix);

            return;
        }

        for (int i = 0; i < 3; i++)
        {
            Vec3r force_i = _cur_query_point.face_barys[i]*total_force;
            _tissue_obj.setVertexAppliedForce(face[i], force_i);
        }

        _applying_force = true;

        // reset the displacement application time
        _force_application_time = _time;
    }
}

} // namespace Sim