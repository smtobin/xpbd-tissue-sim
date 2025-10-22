#include "simulation/FixedObjectSimulation.hpp"

#include "config/simulation/FixedObjectSimulationConfig.hpp"

namespace Sim
{

FixedObjectSimulation::FixedObjectSimulation(const Config::FixedObjectSimulationConfig* config)
    : Simulation(config)
{
    _text_file_save_interval = config->textFileSaveInterval();
    _text_file_save_folder = config->textFileSaveFolder();
    if (_text_file_save_folder.back() != '/')
    {
        _text_file_save_folder += "/";
    }

    _fixed_face = config->cubeFixedFace();
    
    Vec3r ori = config->pointCloudSampleOrientation();
    Mat3r rot_mat = GeometryUtils::quatToMat(GeometryUtils::eulXYZ2Quat(ori[0]*M_PI/180.0, ori[1]*M_PI/180.0, ori[2]*M_PI/180.0));
    _point_cloud_sample_frame = Geometry::CoordinateFrame(Geometry::TransformationMatrix(rot_mat, config->pointCloudSamplePosition()));
}

void FixedObjectSimulation::setup()
{
    Simulation::setup();

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

    // save initial vertices/elements/faces .txt file
    if (_text_file_save_interval >= 0)
    {
        // create the folder to save vertices and stiffness matrices
        std::filesystem::create_directory(_text_file_save_folder);

        /** SAVE INITIAL VERTICES/ELEMENTS/FACES .txt file*/
        std::ofstream vertices_ss(_text_file_save_folder + "initial_vertices.txt");
        vertices_ss << _cube_obj.mesh()->vertices().transpose();
        vertices_ss.close();

        std::ofstream elements_ss(_text_file_save_folder + "initial_elements.txt");
        elements_ss << _cube_obj.tetMesh()->elements().transpose();
        elements_ss.close();
        
        std::ofstream surface_faces_ss(_text_file_save_folder + "initial_surface_faces.txt");
        surface_faces_ss << _cube_obj.mesh()->faces().transpose();
        surface_faces_ss.close();
    }
}

void FixedObjectSimulation::_timeStep()
{
    Simulation::_timeStep();

    if (_text_file_save_interval >= 0 && _num_dt_since_last_save >= _text_file_save_interval)
    {
        MatXr stiffness_mat = _cube_obj.stiffnessMatrix();

        // save stiffness matrix
        std::stringstream sfilename_ss;
        sfilename_ss << _text_file_save_folder << std::setw(6) << std::setfill('0') << "stiffness" << _num_saved_text_files << ".txt";
        std::ofstream stiffness_ss(sfilename_ss.str());
        stiffness_ss << stiffness_mat;
        stiffness_ss.close();

        // save vertices file
        std::stringstream vfilename_ss;
        vfilename_ss << _text_file_save_folder << std::setw(6) << std::setfill('0') << "vertices" << _num_saved_text_files << ".txt";
        std::ofstream vertices_ss(vfilename_ss.str());
        vertices_ss << _cube_obj.mesh()->vertices().transpose();
        vertices_ss.close();

        _num_saved_text_files++;
        _num_dt_since_last_save = 0;
    }

    _num_dt_since_last_save++;
}

} // namespace Sim