#include "simulation/GraspingSimulation.hpp"

namespace Sim
{

GraspingSimulation::GraspingSimulation(const Config::GraspingSimulationConfig* config)
    : Simulation(config), _grasping(false), _cursor(nullptr)
{
    _grasp_radius = config->graspRadius();
    _fix_min_z = config->fixMinZ();

    // initialize the keys map with relevant keycodes for controlling the Virtuoso robot with the keyboard
    SimulationInput::Key keys[] = {
        SimulationInput::Key::SPACE, // space bar
        SimulationInput::Key::W, // W
        SimulationInput::Key::S, // S
    };

    size_t num_keys = sizeof(keys) / sizeof(keys[0]);
    for (unsigned i = 0; i < num_keys; i++)
        _keys_held[keys[i]] = 0;
}

void GraspingSimulation::setup()
{
    Simulation::setup();

    if (_fix_min_z)
    {
        std::vector<std::unique_ptr<Sim::XPBDMeshObject_Base>>& xpbd_mesh_objs = _objects.template get<std::unique_ptr<Sim::XPBDMeshObject_Base>>();
        for (auto& obj : xpbd_mesh_objs)
        {
            // get min z coordinate of the object's mesh
            Vec3r min_bbox_point = obj->mesh()->boundingBox().min;
            std::vector<int> vertices_to_fix = obj->mesh()->getVerticesWithZ(min_bbox_point[2]);
            for (const auto& v : vertices_to_fix)
            {
                obj->fixVertex(v);
            }
        }
    }
    

    // create an object to show where grasping is
    Config::RigidSphereConfig cursor_config("cursor", Vec3r(0,0,0), Vec3r(0,0,0), Vec3r(0,0,0), Vec3r(0,0,0),
        1.0, _grasp_radius, false, true, false, Config::ObjectRenderConfig());
    _cursor = _addObjectFromConfig(&cursor_config);
    assert(_cursor);
}

void GraspingSimulation::notifyMouseButtonPressed(SimulationInput::MouseButton button, SimulationInput::MouseAction action, int modifiers)
{

    // button = 0 ==> left mouse button
    // button = 1 ==> right mouse button
    // action = 0 ==> mouse up
    // action = 1 ==> mouse down

    if (button == SimulationInput::MouseButton::LEFT && action == SimulationInput::MouseAction::PRESS)
    {
        _toggleGrasping();
    }

    Simulation::notifyMouseButtonPressed(button, action, modifiers);

}

void GraspingSimulation::notifyMouseMoved(double x, double y)
{
    if (_keys_held[SimulationInput::Key::SPACE] > 0) // space bar = clutch
    {
        const Real scaling = _grasp_radius/50.0;
        Real dx = x - _last_mouse_pos[0];
        Real dy = y - _last_mouse_pos[1];

        // camera plane defined by camera up direction and camera right direction
        // changes in mouse y position = changes along camera up direction
        // changes in mouse x position = changes along camera right direction
        const Vec3r up_vec = _graphics_scene->cameraUpDirection();
        const Vec3r right_vec = _graphics_scene->cameraRightDirection();
        
        const Vec3r offset = right_vec*dx + up_vec*-dy; // negate dy since increasing dy is actually opposite of camera frame up vec
        _moveCursor(offset*scaling);
    }

    _last_mouse_pos[0] = x;
    _last_mouse_pos[1] = y;
}

void GraspingSimulation::notifyKeyPressed(SimulationInput::Key key, SimulationInput::KeyAction action, int modifiers)
{

    // find key in map
    auto it = _keys_held.find(key);
    if (it != _keys_held.end())
    {
        it->second = (action == SimulationInput::KeyAction::PRESS); // if action > 0, key is pressed or held
    }

    Simulation::notifyKeyPressed(key, action, modifiers);

}

void GraspingSimulation::notifyMouseScrolled(double dx, double dy)
{
    // when using mouse input, mouse scrolling moves the robot tip in and out of the page
    if (_keys_held[SimulationInput::Key::SPACE] > 0) // space bar = clutch
    {
        const Real scaling = _grasp_radius/2.0;
        const Vec3r view_dir = _graphics_scene->cameraViewDirection();

        const Vec3r offset = view_dir*dy;
        _moveCursor(offset*scaling);
    }
    

    Simulation::notifyMouseScrolled(dx, dy);
    
}

void GraspingSimulation::_moveCursor(const Vec3r& dp)
{
    // move the tip cursor and the active arm tip position
    const Vec3r current_position = _cursor->position();
    _cursor->setPosition(current_position + dp);
}

void GraspingSimulation::_timeStep()
{
    Real grasp_radius_change = 0;
    if (_keys_held[SimulationInput::Key::W] > 0) // W = increase grasping radius
    {
        grasp_radius_change += _grasp_radius/300.0;

    }
    if (_keys_held[SimulationInput::Key::S] > 0) // S = decrease grasping radius
    {
        grasp_radius_change += -_grasp_radius/300.0;
    }
    _grasp_radius += grasp_radius_change;
    _cursor->setRadius(_grasp_radius);
    

    Simulation::_timeStep();
}

void GraspingSimulation::_toggleGrasping()
{
    std::vector<std::unique_ptr<Sim::XPBDMeshObject_Base>>& xpbd_mesh_objs = _objects.template get<std::unique_ptr<Sim::XPBDMeshObject_Base>>();
    // if not currently grasping, start grasping vertices inside grasping radius
    if (!_grasping)
    {
        std::map<std::pair<Sim::XPBDMeshObject_Base*, int>, Vec3r> vertices_to_grasp;

        const Vec3r grasp_center = _cursor->position();
        // quick and dirty way to find all vertices in a sphere
        for (int theta = 0; theta < 360; theta+=30)
        {
            for (int phi = 0; phi < 360; phi+=30)
            {
                for (double p = 0; p < _grasp_radius; p+=_grasp_radius/5.0)
                {
                    const double x = grasp_center[0] + p*std::sin(phi*M_PI/180)*std::cos(theta*M_PI/180);
                    const double y = grasp_center[1] + p*std::sin(phi*M_PI/180)*std::sin(theta*M_PI/180);
                    const double z = grasp_center[2] + p*std::cos(phi*M_PI/180);

                    for (auto& xpbd_mesh_obj : xpbd_mesh_objs)
                    {
                        int v = xpbd_mesh_obj->mesh()->getClosestVertex(Vec3r(x, y, z));

                        // make sure v is inside grasping sphere
                        if ((grasp_center - xpbd_mesh_obj->mesh()->vertex(v)).norm() <= _grasp_radius)
                            if (!xpbd_mesh_obj->vertexFixed(v))
                            {
                                std::pair<Sim::XPBDMeshObject_Base*, int> obj_vert_pair = std::make_pair(xpbd_mesh_obj.get(), v);
                                const Vec3r attachment_offset = xpbd_mesh_obj->mesh()->vertex(v) - grasp_center;
                                vertices_to_grasp[obj_vert_pair] = attachment_offset;
                            }
                    }
                    
                }
            }
        }

        for (const auto& [obj_vert_pair, offset] : vertices_to_grasp)
        {
            obj_vert_pair.first->addAttachmentConstraint(obj_vert_pair.second, &_cursor->position(), offset);
            _grasped_vertices.push_back(obj_vert_pair);
        }
    }

    // if tool state has changed from 1 to 0, stop grasping
    else if (_grasping)
    {
        for (auto& xpbd_mesh_obj : xpbd_mesh_objs)
        {
            xpbd_mesh_obj->clearAttachmentConstraints();
        }
        _grasped_vertices.clear();
    }

    _grasping = !_grasping;
}

} // namespace Sim