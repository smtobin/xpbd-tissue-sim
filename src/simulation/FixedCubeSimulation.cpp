#include "simulation/FixedCubeSimulation.hpp"

#include "config/simulation/FixedCubeSimulationConfig.hpp"

namespace Sim
{

FixedCubeSimulation::FixedCubeSimulation(const Config::FixedCubeSimulationConfig* config)
    : Simulation(config)
{
    _text_file_save_interval = config->textFileSaveInterval();

    _fixed_face = config->cubeFixedFace();
    
    Vec3r ori = config->pointCloudSampleOrientation();
    Mat3r rot_mat = GeometryUtils::quatToMat(GeometryUtils::eulXYZ2Quat(ori[0], ori[1], ori[2]));
    _point_cloud_sample_frame = Geometry::CoordinateFrame(Geometry::TransformationMatrix(rot_mat, config->pointCloudSamplePosition()));
}

void FixedCubeSimulation::setup()
{
    Simulation::setup();

    
    if (_text_file_save_interval >= 0)
    {
        /** SAVE INITIAL VERTICES/ELEMENTS/FACES .txt file*/
    }

    // find the XPBDMeshObject (which we're assuming to be the cube)
    // first check 1st-order XPBDMeshObjects
    auto& fo_xpbd_objs = _objects.template get<std::unique_ptr<FirstOrderXPBDMeshObject_Base>>();
    auto& xpbd_objs = _objects.template get<std::unique_ptr<XPBDMeshObject_Base>>();
    if (fo_xpbd_objs.size() > 0)
        _cube_obj = fo_xpbd_objs.front().get();
    else if (xpbd_objs.size() > 0)
        _cube_obj = xpbd_objs.front().get();
    else
        assert((xpbd_objs.size() + fo_xpbd_objs.size() > 0) && "There must be at least 1 XPBDMeshObject or FirstOrderXPBDMeshObject in the simulation (there is no tissue object!).");

    // fix whatever face is prescribed by the config file
    Geometry::AABB bbox = _cube_obj.mesh()->boundingBox();
    std::vector<int> vertices_to_fix;
    if (_fixed_face == CubeFace::LEFT)
    {
        vertices_to_fix = _cube_obj.mesh()->getVerticesWithY(bbox.min[1]);
    }
    else if (_fixed_face == CubeFace::RIGHT)
    {
        vertices_to_fix = _cube_obj.mesh()->getVerticesWithY(bbox.max[1]);
    }
    else if (_fixed_face == CubeFace::TOP)
    {
        vertices_to_fix = _cube_obj.mesh()->getVerticesWithZ(bbox.max[2]);
    }
    else if (_fixed_face == CubeFace::BOTTOM)
    {
        vertices_to_fix = _cube_obj.mesh()->getVerticesWithZ(bbox.min[2]);
    }

    for (const auto& vert_index : vertices_to_fix)
    {
        _cube_obj.fixVertex(vert_index);
    }
}

void FixedCubeSimulation::_timeStep()
{
    Simulation::_timeStep();

    if (_text_file_save_interval >= 0 && _num_dt_since_last_save >= _text_file_save_interval)
    {
        /** SAVE TEXT FILES */

        _num_dt_since_last_save = 0;
    }

    _num_dt_since_last_save++;
}

} // namespace Sim