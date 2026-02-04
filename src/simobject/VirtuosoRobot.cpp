#include "simobject/VirtuosoRobot.hpp"

namespace Sim
{

VirtuosoRobot::VirtuosoRobot(const Simulation* sim, const ConfigType* config)
    : Object(sim, config), _arm1(), _arm2()
{
    for (const auto& arm_config : config->armConfigs())
    {
        if (!_arm1)
        {
            _arm1 = VirtuosoArm(sim, &arm_config);
        }
        else if (!_arm2)
        {
            _arm2 = VirtuosoArm(sim, &arm_config);
        }
        else
        {
            std::cout << "More than two arms specified! Using only the first two..." << std::endl;
            break;
        }
    }

    const Vec3r& position = config->initialPosition();

    const Vec3r& rot_eul_xyz = config->initialRotation() * M_PI / 180.0;
    const Mat3r& rot_mat = GeometryUtils::quatToMat(GeometryUtils::eulXYZ2Quat(rot_eul_xyz[0], rot_eul_xyz[1], rot_eul_xyz[2]));
    _endoscope_frame = Geometry::CoordinateFrame(Geometry::TransformationMatrix(rot_mat, position));

    _endoscope_dia = config->endoscopeDiameter();
    _endoscope_length = config->endoscopeLength();
    _arm_separation_dist = config->armSeparationDistance();
    _optic_vertical_dist = config->opticVerticalDistance();
    _optic_forward_dist = config->opticForwardDistance();
    _optic_tilt = config->opticTilt() * M_PI / 180.0;

    // origin of VirtuosoRobot frame is at the center of the end of the endoscope
    // Z points forward away from the endoscope (aligned with initial arm Z axes)
    // Y up, X left
    // arm 1 is the left arm, arm 2 is the right arm
    Geometry::TransformationMatrix arm1_rel_transform(Mat3r::Identity(), Vec3r(_arm_separation_dist/2.0, 0, 0));
    Geometry::TransformationMatrix arm2_rel_transform(Mat3r::Identity(), Vec3r(-_arm_separation_dist/2.0, 0, 0));
    _VB_frame = _endoscope_frame * arm1_rel_transform;
    _VR_frame = _endoscope_frame * arm2_rel_transform;

    if (_arm1)
    {
        _arm1->setBasePosition(_VB_frame.transform().translation());
        _arm1->setBaseRotation(_VB_frame.transform().rotMat());
    }
    if (_arm2)
    {
        _arm2->setBasePosition(_VR_frame.transform().translation());
        _arm2->setBaseRotation(_VR_frame.transform().rotMat());
    }

    // compute cam frame
    Geometry::TransformationMatrix cam_rel_transform(GeometryUtils::Rx(_optic_tilt), Vec3r(0, _optic_vertical_dist, _optic_forward_dist));
    _cam_frame = _endoscope_frame * cam_rel_transform;

    // set the characteristic dimension to the minimum between the endoscope diameter and length
    _char_dim = std::min(_endoscope_dia, _endoscope_length);
}

std::string VirtuosoRobot::toString(int indent) const
{
    // TODO: better toString
    return Object::toString(indent);
}

void VirtuosoRobot::setVBtoCamTransform(const Geometry::TransformationMatrix& new_transform)
{
    _cam_frame = _VB_frame * new_transform.inverse();
}

void VirtuosoRobot::setup()
{
    if (_arm1)
        _arm1->setup();
    if (_arm2)
        _arm2->setup();
}

void VirtuosoRobot::update()
{
    if (_arm1)
        _arm1->update();
    if (_arm2)
        _arm2->update();
}

void VirtuosoRobot::velocityUpdate()
{
    // nothing for now
    if (_arm1)
        _arm1->velocityUpdate();
    if (_arm2)
        _arm2->velocityUpdate();
}

} // namespace Sim