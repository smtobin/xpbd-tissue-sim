#include "simobject/VirtuosoArm.hpp"
#include "simobject/XPBDMeshObject.hpp"
#include "simulation/Simulation.hpp"

#include "config/simobject/VirtuosoArmConfig.hpp"

#include "utils/GeometryUtils.hpp"
#include "utils/MathUtils.hpp"

#include <math.h>
#include <algorithm>
#include <numeric>
#include <unordered_set>

namespace Sim 
{

VirtuosoArm::VirtuosoArm(const Simulation* sim, const ConfigType* config)
    : Object(sim, config), _ot_frames(), _it_frames()
{
    _it_outer_dia = config->innerTubeOuterDiameter();
    _it_inner_dia = config->innerTubeInnerDiameter();
    _it_translation = config->innerTubeInitialTranslation();
    _it_rotation = config->innerTubeInitialRotation() * M_PI/180.0;   // convert to radians

    _ot_outer_dia = config->outerTubeOuterDiameter();
    _ot_inner_dia = config->outerTubeInnerDiameter();
    _ot_r_curvature = config->outerTubeRadiusOfCurvature();
    _ot_translation = config->outerTubeInitialTranslation();
    _ot_rotation = config->outerTubeInitialRotation() * M_PI/180.0;   // convert to radians
    _ot_distal_straight_length = config->outerTubeDistalStraightLength();

    _max_ot_translation_speed = config->maxOTTranslationSpeed();
    _max_it_translation_speed = config->maxITTranslationSpeed();
    _max_ot_rotation_speed = config->maxOTRotationSpeed();
    _max_it_rotation_speed = config->maxITRotationSpeed();

    _tool_state = 0;  // default tool state is off
    _tool_type = config->toolType();
    _cutting_model = config->cuttingModel();
    _cutting_model_time_threshold = config->cuttingModelTimeThreshold();
    _tool_manipulated_object = (XPBDMeshObject_Base_<false>*)nullptr;

    if (config->toolType() == ToolType::SPATULA)
        _tool = std::make_unique<Sim::VirtuosoArmSpatulaTool>(_sim, config, nullptr);
    else if (config->toolType() == ToolType::CAUTERY)
        _tool = std::make_unique<Sim::VirtuosoArmCauteryTool>(_sim, config, nullptr);

    _arm_base_position = config->baseInitialPosition();
    Vec3r initial_rot_xyz = config->baseInitialRotation() * M_PI / 180.0;
    _arm_base_rotation = GeometryUtils::quatToMat(GeometryUtils::eulXYZ2Quat(initial_rot_xyz[0], initial_rot_xyz[1], initial_rot_xyz[2]));

    _tip_force = Vec3r::Zero();
    _tip_moment = Vec3r::Zero();
    _xpbd_collision_forces.resize(NUM_OT_FRAMES + NUM_IT_FRAMES, Vec3r::Zero());

    for (int i = 0; i < NUM_OT_FRAMES; i++)
        _ot_nodal_forces[i] = Vec3r::Zero();
    for (int i = 0; i < NUM_IT_FRAMES; i++)
        _it_nodal_forces[i] = Vec3r::Zero();

    _stale_frames = true;

    // set the characteristic dimension to the minimum diameter being used
    _char_dim = _it_outer_dia;

}

std::string VirtuosoArm::toString(const int indent) const
{
    // TODO: better toString
    return Object::toString(indent);
}

int VirtuosoArm::numSegments() const
{
    return (NUM_OT_FRAMES-1) + (NUM_IT_FRAMES-1);
}

Geometry::Capsule VirtuosoArm::segment(int index) const
{
    assert(index < numSegments());

    if (index < NUM_OT_FRAMES-1)
    {
        return Geometry::Capsule(_ot_frames[index].origin(), _ot_frames[index+1].origin(), _ot_outer_dia/2.0);
    }
    
    int it_index = index - (NUM_OT_FRAMES);
    if (it_index < NUM_IT_FRAMES-1)
    {
        return Geometry::Capsule(_it_frames[it_index].origin(), _it_frames[it_index+1].origin(), _it_outer_dia/2.0);
    }
}

Mat3r VirtuosoArm::complianceMatrixAtIntegrationPoint(int node_index)
{
    if (_stale_frames)
    {
        _recomputeCoordinateFramesStaticsModelWithNodalForces();
    }

    // store the current frames at integration points
    // (recomputeCoordinateFramesStaticModelWithNodalForces() will overwrite them)
    OuterTubeFramesArray orig_ot_frames = _ot_frames;
    InnerTubeFramesArray orig_it_frames = _it_frames;

    // get the array index of the node int he appropriate frames array
    int arr_index = -1;
    // figure out which segment of the arm the node index belongs to
    bool ot_frame = false;
    if (node_index < NUM_OT_FRAMES)
    {
        arr_index = node_index;
        ot_frame = true;
    }
    else if (node_index < NUM_OT_FRAMES + NUM_IT_FRAMES)
    {
        arr_index = node_index - NUM_OT_FRAMES;
    }
    else
    {
        std::cerr << "Error: VirtuosoArm::stiffnessMatrixAtIntegrationPoint(): " << node_index << " >= number of integration points." << std::endl;
        assert(0);
    }

    // get current position of this integration point
    Vec3r orig_pos;
    if (ot_frame)
        orig_pos = _ot_frames[arr_index].origin();
    else
        orig_pos = _it_frames[arr_index].origin();

    // std::cout << "Original position: " << orig_pos.transpose() << std::endl;
    // iterate through 3 coordinate directions
    Mat3r C;
    Real deltaF = 1e-3;
    for (int i = 0; i < 3; i++)
    {
        Vec3r dF = Vec3r::Zero();
        dF[i] = deltaF;

        Vec3r new_pos;
        if (ot_frame)
        {
            Vec3r cur_force = outerTubeNodalForce(arr_index);
            setOuterTubeNodalForce(arr_index, cur_force + dF);
            _recomputeCoordinateFramesStaticsModelWithNodalForces();
            setOuterTubeNodalForce(arr_index, cur_force);
            new_pos = _ot_frames[arr_index].origin();
            _ot_frames = orig_ot_frames;
            _it_frames = orig_it_frames;
        }
        else
        {
            Vec3r cur_force = innerTubeNodalForce(arr_index);
            setInnerTubeNodalForce(arr_index, cur_force + dF);
            _recomputeCoordinateFramesStaticsModelWithNodalForces();
            setInnerTubeNodalForce(arr_index, cur_force);
            new_pos = _it_frames[arr_index].origin();
            _ot_frames = orig_ot_frames;
            _it_frames = orig_it_frames;
        }

        C.col(i) = (new_pos - orig_pos)/deltaF;
    }

    _stale_frames = false;  // we've reset everything, so things shouldn't be stale
    return 0.5*(C + C.transpose()); // ensure that it is symmetric
}

Mat3r VirtuosoArm::interpolatedComplianceMatrix(int node_index, Real interp)
{
    if (node_index < NUM_OT_FRAMES)
    {
        if (_stale_ot_compliance_interp)
            _computeComplianceMatrixInterpolation(true, false, false);
        
        Real s = (node_index + interp) / (NUM_OT_FRAMES-1);
        Mat3r C_interp = _ot_compliance_coeff[0] + _ot_compliance_coeff[1] * s + _ot_compliance_coeff[2] * s*s + _ot_compliance_coeff[3] * s*s*s;
        return C_interp;
    }
    else if (node_index < NUM_OT_FRAMES + NUM_IT_FRAMES)
    {
        if (_stale_it_compliance_interp)
            _computeComplianceMatrixInterpolation(false, true, false);

        Real s = (node_index + interp - NUM_OT_FRAMES) / (NUM_IT_FRAMES-1);
        Mat3r C_interp = _it_compliance_coeff[0] + _it_compliance_coeff[1] * s + _it_compliance_coeff[2] * s*s + _it_compliance_coeff[3] * s*s*s;
        return C_interp;
    }
    else
    {
        std::cerr << "VirtuosoArm::interpolatedComplianceMatrix - Node index out of bounds!" << std::endl;
        assert(0);
        return Mat3r::Zero();
    }
}

void VirtuosoArm::_computeComplianceMatrixInterpolation(bool ot, bool it, bool tt)
{
    // compute interpolation for outer tube
    if (ot && _stale_ot_compliance_interp)
    {
        int ip0 = NUM_OT_FRAMES/3;
        int ip1 = 2*NUM_OT_FRAMES/3;
        int ip2 = NUM_OT_FRAMES-1;
        Real s0 = Real(ip0) / (NUM_OT_FRAMES-1);
        Real s1 = Real(ip1) / (NUM_OT_FRAMES-1);
        Real s2 = Real(ip2) / (NUM_OT_FRAMES-1);

        Mat3r vdm;
        vdm <<  s0, s0*s0, s0*s0*s0,
                s1, s1*s1, s1*s1*s1,
                s2, s2*s2, s2*s2*s2;
        Mat3r C0 = complianceMatrixAtIntegrationPoint(ip0);
        Mat3r C1 = complianceMatrixAtIntegrationPoint(ip1);
        Mat3r C2 = complianceMatrixAtIntegrationPoint(ip2);

        for (int i = 0; i < 3; i++)
        {
            for (int j = i; j < 3; j++)
            {
                Vec3r rhs( C0(i,j), C1(i,j), C2(i,j) );
                Vec3r coeff_ij = vdm.colPivHouseholderQr().solve(rhs);

                // special case: we know that the compliance at the base is 0, so a0=0
                _ot_compliance_coeff[0](i,j) = 0;
                _ot_compliance_coeff[0](j,i) = 0;
                for (int k = 1; k < 4; k++)
                {
                    _ot_compliance_coeff[k](i,j) = coeff_ij[k-1];
                    _ot_compliance_coeff[k](j,i) = coeff_ij[k-1];
                }
            }
        }

        _stale_ot_compliance_interp = false;
    }
    
    if (it && _stale_it_compliance_interp)
    {
        int ip0 = NUM_OT_FRAMES;
        int ip1 = NUM_OT_FRAMES + NUM_IT_FRAMES/3;
        int ip2 = NUM_OT_FRAMES + 2*NUM_IT_FRAMES/3;
        int ip3 = NUM_OT_FRAMES + NUM_IT_FRAMES-1;
        Real s0 = Real(ip0 - NUM_OT_FRAMES) / (NUM_IT_FRAMES-1);
        Real s1 = Real(ip1 - NUM_OT_FRAMES) / (NUM_IT_FRAMES-1);
        Real s2 = Real(ip2 - NUM_OT_FRAMES) / (NUM_IT_FRAMES-1);
        Real s3 = Real(ip3 - NUM_OT_FRAMES) / (NUM_IT_FRAMES-1);

        Mat4r vdm;
        vdm <<  1, s0, s0*s0, s0*s0*s0,
                1, s1, s1*s1, s1*s1*s1,
                1, s2, s2*s2, s2*s2*s2,
                1, s3, s3*s3, s3*s3*s3;
        Mat3r C0 = complianceMatrixAtIntegrationPoint(ip0);
        Mat3r C1 = complianceMatrixAtIntegrationPoint(ip1);
        Mat3r C2 = complianceMatrixAtIntegrationPoint(ip2);
        Mat3r C3 = complianceMatrixAtIntegrationPoint(ip3);

        for (int i = 0; i < 3; i++)
        {
            for (int j = i; j < 3; j++)
            {
                Vec4r rhs( C0(i,j), C1(i,j), C2(i,j), C3(i,j) );
                Vec4r coeff_ij = vdm.colPivHouseholderQr().solve(rhs);

                for (int k = 0; k < 4; k++)
                {
                    _it_compliance_coeff[k](i,j) = coeff_ij[k];
                    _it_compliance_coeff[k](j,i) = coeff_ij[k];
                }
            }
        }

        _stale_it_compliance_interp = false;
    }
}

Vec3r VirtuosoArm::actualTipPosition() const
{
    if (hasTool())
        return _tool->tipFrame().origin();
    else
        return _it_frames.back().origin();
}

void VirtuosoArm::setCommandedTipPosition(const Vec3r& new_position)
{
    // _jacobianDifferentialInverseKinematics(new_position - tipPosition());
    const Geometry::TransformationMatrix T_tip = _computeTipTransform(_ot_rotation, _ot_translation, _it_rotation, _it_translation);
    _hybridDifferentialInverseKinematics(new_position - T_tip.translation());
    _commanded_tip_position = new_position;
    _stale_frames = true;
}

void VirtuosoArm::setTipForce(const Vec3r& new_tip_force)
{
    _tip_force = new_tip_force;
    // _recomputeCoordinateFramesStaticsModelWithNodalForces();
    _stale_frames = true;
}

void VirtuosoArm::setTipMoment(const Vec3r& new_tip_moment)
{
    _tip_moment = new_tip_moment;
    // _recomputeCoordinateFramesStaticsModelWithNodalForces();
    _stale_frames = true;
}

void VirtuosoArm::setTipForceAndMoment(const Vec3r& new_tip_force, const Vec3r& new_tip_moment)
{
    _tip_force = new_tip_force;
    _tip_moment = new_tip_moment;
    // _recomputeCoordinateFramesStaticsModelWithNodalForces();
    _stale_frames = true;
}

void VirtuosoArm::setOuterTubeNodalForce(int node_index, const Vec3r& force)
{
    assert(node_index < NUM_OT_FRAMES);
    _ot_nodal_forces[node_index] = force;
    _stale_frames = true;
}

void VirtuosoArm::setInnerTubeNodalForce(int node_index, const Vec3r& force)
{
    assert(node_index < NUM_IT_FRAMES);
    _it_nodal_forces[node_index] = force;
    _stale_frames = true;
}

void VirtuosoArm::applyNodalForce(int node_index, Real interp, const Vec3r& force)
{
    if (node_index < NUM_OT_FRAMES-1)
    {
        _ot_nodal_forces[node_index] += (1-interp)*force;
        _ot_nodal_forces[node_index+1] += interp*force;
    }
    else if (node_index >= NUM_OT_FRAMES && node_index < NUM_OT_FRAMES + NUM_IT_FRAMES-1)
    {
        _it_nodal_forces[node_index - NUM_OT_FRAMES] += (1-interp)*force;
        _it_nodal_forces[node_index - NUM_OT_FRAMES+1] += interp*force;
    }
    _stale_frames = true;
}

void VirtuosoArm::setJointState(double ot_rotation, double ot_translation, double it_rotation, double it_translation, int tool)
{
    _ot_rotation = ot_rotation;
    _ot_translation = ot_translation;
    _it_rotation = it_rotation;
    _it_translation = it_translation;
    _tool_state = tool;

    // _recomputeCoordinateFrames();
    _stale_frames = true;
}

void VirtuosoArm::addRigidCollision(int node_index, Real interp, const Geometry::SDF* rigid_sdf)
{
    auto [it, success] = _rigid_collisions.emplace(node_index, interp, rigid_sdf);

    // if there already exists a rigid collision registered to this segment and this rigid body, then update the interpolation param
    // the interp member is mutable because it does not affect the hash or equality
    if (!success)
    {
        // good idea in theory, but can cause flickering oscillations
        // not really needed, since we just need a coarse collision response
        // TODO: revisit this at some point - maybe smoothly interpolate to new interp values based on the force (probably overkill)
        // it->interp = interp;
    }
}

void VirtuosoArm::addRigidCollision(const Vec3r& contact_point, const Geometry::SDF* rigid_sdf)
{
    auto [it, success] = _rigid_collisions.emplace(contact_point, rigid_sdf);
    // if there already exists a rigid collision registered to this segment and this rigid body, then update the interpolation param
    // the interp member is mutable because it does not affect the hash or equality
    // if (!success)
    // {
    //     // good idea in theory, but can cause flickering oscillations
    //     // not really needed, since we just need a coarse collision response
    //     // TODO: revisit this at some point - maybe smoothly interpolate to new interp values based on the force (probably overkill)
    //     // it->interp = interp;
    //     it->contact_point = contact_point;
    // }
}

void VirtuosoArm::addVirtuosoArmCollision(int node_index, Real interp, VirtuosoArm* other, int other_index, Real other_interp)
{
    bool found = false;
    for (auto& arm_collision : _virtuoso_arm_collisions)
    {
        if (node_index == arm_collision.node_index1 && arm_collision.arm2 == other)
        {
            arm_collision.node_index1 = node_index;
            arm_collision.interp1 = interp;
            arm_collision.node_index2 = other_index;
            arm_collision.interp2 = other_interp;
            found = true;
            break;
        }
    }

    if (!found)
        _virtuoso_arm_collisions.emplace_back(this, node_index, interp, other, other_index, other_interp);
}

void VirtuosoArm::addCollisionConstraint(const VirtuosoArm::CollisionConstraintInfo::ProjectorRefType& proj_ref, int node_index, Real interp)
{
    _collision_constraints.emplace_back(proj_ref, node_index, interp);
}

void VirtuosoArm::clearCollisionConstraints()
{
    _collision_constraints.clear();
    if (hasTool())
        _tool->clearCollisionConstraints();
}

void VirtuosoArm::setup()
{
    if (_tool)
        _tool->setArm(this);

    _recomputeCoordinateFrames();

    _commanded_tip_position = actualTipPosition();
}

void VirtuosoArm::update()
{
    const Geometry::TransformationMatrix T_tip = _computeTipTransform(_ot_rotation, _ot_translation, _it_rotation, _it_translation);
    _hybridDifferentialInverseKinematics(_commanded_tip_position - T_tip.translation());
    if (_stale_frames)
    {
        // _recomputeCoordinateFrames();
        _recomputeCoordinateFramesStaticsModelWithNodalForces();
    }

    // _toolAction();
}

void VirtuosoArm::velocityUpdate()
{
    // perform the tool action AFTER the update step so that the XPBD update for deformable objects has already occurred so that
    // we can compute the constraint forces associated with projections of various constraints
    _toolAction();

    const Real load_frac = 0.005;
    const Real unload_frac = 0.005;//0.1;
    _unfiltered_collision_force = Vec3r::Zero();
    _filtered_collision_force = Vec3r::Zero();

    // apply force/moment that is on the tool
    // if (hasTool())
    // {
    //     auto [tool_tip_force, tool_tip_moment] = _tool->collisionTipForceAndMoment();
    //     Vec3r new_tip_force = Vec3r::Zero();
    //     Vec3r new_tip_moment = Vec3r::Zero();
    //     if (tool_tip_force.squaredNorm() >= _tip_force.squaredNorm())
    //         new_tip_force = (1-load_frac)*_tip_force + load_frac*tool_tip_force;
    //     else
    //         new_tip_force = (1-unload_frac)*_tip_force + unload_frac*tool_tip_force;

    //     if (tool_tip_moment.squaredNorm() >= _tip_moment.squaredNorm())
    //         new_tip_moment = (1-load_frac)*_tip_moment + load_frac*tool_tip_moment;
    //     else
    //         new_tip_moment = (1-unload_frac)*_tip_moment + unload_frac*tool_tip_moment;

    //     _filtered_collision_force += new_tip_force;
    //     _unfiltered_collision_force += tool_tip_force;

    //     _tip_force = new_tip_force;
    //     _tip_moment = new_tip_moment;
    // }
    

    // zero out forces
    // for (auto& f : _ot_nodal_forces)
    //     f = Vec3r::Zero();
    // for (auto& f : _it_nodal_forces)
    //     f = Vec3r::Zero();
    // for (auto& f : _tt_nodal_forces)
    //     f = Vec3r::Zero();

    // apply forces from collision constraints
    std::vector<Vec3r> new_xpbd_collision_forces(NUM_OT_FRAMES + NUM_IT_FRAMES, Vec3r::Zero());
    for (const auto& collision : _collision_constraints)
    {
        
        if (collision.proj_ref.exists() && collision.proj_ref.isValid())
        {
            // the collision constraints give forces on the tissue, so we must negate them to get the forces on the Virtuoso
            std::vector<Vec3r> forces = collision.proj_ref.constraintForces();
            Vec3r net_force = -std::reduce(forces.cbegin(), forces.cend());
            new_xpbd_collision_forces[collision.node_index] += net_force*(1-collision.interp);
            new_xpbd_collision_forces[collision.node_index+1] += net_force*collision.interp;

            _unfiltered_collision_force += net_force;
        }
    }

    
    
    for (unsigned i = 0; i < new_xpbd_collision_forces.size(); i++)
    {
        const Vec3r& cur_xpbd_force = _xpbd_collision_forces[i];
        Vec3r new_force = Vec3r::Zero();
        if (new_xpbd_collision_forces[i].squaredNorm() >= cur_xpbd_force.squaredNorm())
        {
            new_force = (1-load_frac)*cur_xpbd_force + load_frac*new_xpbd_collision_forces[i];
        }
        else
        {
            new_force = (1-unload_frac)*cur_xpbd_force + unload_frac*new_xpbd_collision_forces[i];
        }

        Vec3r force_diff = new_force - _xpbd_collision_forces[i];

        _xpbd_collision_forces[i] = new_force;
        _filtered_collision_force += new_force;

        applyNodalForce(i, 0, force_diff);
    }

    std::cout << "Applying forces for " << _rigid_collisions.size() << " rigid collisions..." << std::endl;


    /** Apply forces for rigid collisions */

    for (auto it = _rigid_collisions.begin(); it != _rigid_collisions.end();)
    {
        auto& rigid_collision = *it;
        Vec3r last_force =  rigid_collision.force;

        // std::cout << "Applying force for rigid collision for node index " << rigid_collision.node_index << std::endl; 
        // get capsule representing segment
        Geometry::Capsule seg = segment(rigid_collision.node_index);
        Vec3r cp, normal;
        Real penetration_dist;
        Mat3r C;
        if (rigid_collision.is_tool)
        {
            cp = innerTubeEndFrame().transform().rotMat() * rigid_collision.contact_point + innerTubeEndFrame().origin();
            penetration_dist = rigid_collision.rigid_sdf->evaluate(cp);
            normal = rigid_collision.rigid_sdf->gradient(cp);

            std::cout << "Tool penetration dist: " << penetration_dist << std::endl;

            C = complianceMatrixAtIntegrationPoint(NUM_OT_FRAMES+NUM_IT_FRAMES-1);
        }
        else
        {
            cp = seg.p1() + rigid_collision.interp*(seg.p2() - seg.p1());
            penetration_dist = rigid_collision.rigid_sdf->evaluate(cp) - seg.radius();
            normal = rigid_collision.rigid_sdf->gradient(cp);

            C = interpolatedComplianceMatrix(rigid_collision.node_index, rigid_collision.interp);
        }
        Vec3r surface_point = cp - penetration_dist*normal;

        // std::cout << "  Penetration dist: " << penetration_dist << std::endl;
        // std::cout << "  Normal: " << normal.transpose() << std::endl;
        // std::cout << "  Sep dir: " << sep_dir.transpose() << std::endl;
        
        // the nominal change in position
        Vec3r dp = -penetration_dist*normal;

        // SVD
        Eigen::JacobiSVD<Mat3r> svd(C, Eigen::ComputeFullU | Eigen::ComputeFullV);

        // compute a change in position that doesn't require moving in the stiff direction
        // without this correction, the corrective forces are extremely large
        Vec3r dp_nonstiff = dp - dp.dot(svd.matrixU().col(2)) * svd.matrixU().col(2);

        // get the force required to (approximately) fix the penetration 
        Vec3r F_req = C.colPivHouseholderQr().solve(dp_nonstiff);
        Real F_req_mag = F_req.norm();

        // std::cout << "  Required force: " << F_req.transpose() << std::endl;

        // there is still penetration, we need more force
        if (penetration_dist < 0)
        {
            rigid_collision.force += 0.01*F_req;
        }
        // otherwise we need less force
        else
        {
            rigid_collision.force -= 0.01*rigid_collision.force;
        }

        // decay components of force not in the normal direction
        Vec3r force_not_normal = rigid_collision.force - rigid_collision.force.dot(normal) * normal;
        rigid_collision.force -= 0.001*force_not_normal;

        // std::cout << "  force not normal: " << 0.01*force_not_normal << std::endl;
        // std::cout << "  New collision force: " << rigid_collision.force.transpose() << std::endl;

        // std::cout << "  Force diff: " << (rigid_collision.force - last_force).transpose() << std::endl;

        // undo the previous force
        if (rigid_collision.is_tool)
        {
            _tip_force += -last_force;
            _tip_moment += -rigid_collision.prev_tip_moment;
        }
        else
        {
            applyNodalForce(rigid_collision.node_index, rigid_collision.prev_interp, -last_force);
        }

        // remove the collision if the force has gone down to near 0
        if (rigid_collision.force.norm() < 1e-5)
        {
            // applyNodalForce(rigid_collision.node_index, rigid_collision.interp, -rigid_collision.force);
            it = _rigid_collisions.erase(it);
            continue;
        }


        // apply the force
        if (rigid_collision.is_tool)
        {
            _tip_force += rigid_collision.force;

            Vec3r cur_tip_moment = (cp - innerTubeEndFrame().origin()).cross(rigid_collision.force);
            _tip_moment += cur_tip_moment;
            rigid_collision.prev_tip_moment = cur_tip_moment;
        }
        else
        {
            applyNodalForce(rigid_collision.node_index, rigid_collision.interp, rigid_collision.force);
            rigid_collision.prev_interp = rigid_collision.interp;
        }
        

        _filtered_collision_force += rigid_collision.force;
        _unfiltered_collision_force += rigid_collision.force;

        
        // move on to the next collision
        ++it;
    }

    /** Apply forces for virtuoso-virtuoso collisions */
    for (auto it = _virtuoso_arm_collisions.begin(); it != _virtuoso_arm_collisions.end();)
    {
        auto& arm_collision = *it;
        Vec3r last_force = arm_collision.force;

        Geometry::Capsule seg1 = segment(arm_collision.node_index1);
        Vec3r cp1 = seg1.p1() + arm_collision.interp1*(seg1.p2() - seg1.p1());

        Geometry::Capsule seg2 = arm_collision.arm2->segment(arm_collision.node_index2);
        Vec3r cp2 = seg2.p1() + arm_collision.interp2*(seg2.p2() - seg2.p1());

        // get penetration distance (negative if penetration, positive if no penetration)
        Real penetration_dist = (cp1 - cp2).norm() - seg1.radius() - seg2.radius();
        Vec3r normal = (cp1 - cp2).normalized();

        Geometry::VirtuosoArmSDF::DistanceAndGradientWithNodeInfo result = arm_collision.arm2->SDF()->evaluateWithGradientAndNodeInfo(cp1);
        penetration_dist = result.distance - seg1.radius();
        normal = result.gradient;
        arm_collision.node_index2 = result.node_index;
        arm_collision.interp2 = result.interp_factor;

        // the nominal change in position
        Vec3r dp1 = -penetration_dist*normal;
        Vec3r dp2 = penetration_dist*normal;

        // get compliance matrix at this point
        Mat3r C1 = interpolatedComplianceMatrix(arm_collision.node_index1, arm_collision.interp1);
        Mat3r C2 = arm_collision.arm2->interpolatedComplianceMatrix(arm_collision.node_index2, arm_collision.interp2);

        // SVD
        Eigen::JacobiSVD<Mat3r> svd1(C1, Eigen::ComputeFullU | Eigen::ComputeFullV);
        Eigen::JacobiSVD<Mat3r> svd2(C2, Eigen::ComputeFullU | Eigen::ComputeFullV);

        // compute a change in position that doesn't require moving in the stiff direction
        // without this correction, the corrective forces are extremely large
        Vec3r dp1_nonstiff = dp1 - dp1.dot(svd1.matrixU().col(2)) * svd1.matrixU().col(2);

        // get the force required to (approximately) fix the penetration 
        Vec3r F_req1 = C1.colPivHouseholderQr().solve(dp1_nonstiff);
        Real F_req_mag1 = F_req1.norm();

        // there is still penetration, we need more force
        if (penetration_dist < 0)
        {
            arm_collision.force += 0.05*F_req1;
        }
        // otherwise we need less force
        else
        {
            arm_collision.force -= 0.05*arm_collision.force;
        }

        // apply the force
        Vec3r force_diff = arm_collision.force - last_force;
        applyNodalForce(arm_collision.node_index1, arm_collision.interp1, arm_collision.force);
        applyNodalForce(arm_collision.last_node_index1, arm_collision.last_interp1, -last_force);
        arm_collision.arm2->applyNodalForce(arm_collision.node_index2, arm_collision.interp2, -arm_collision.force);
        arm_collision.arm2->applyNodalForce(arm_collision.last_node_index2, arm_collision.last_interp2, last_force);

        _filtered_collision_force += arm_collision.force;
        _unfiltered_collision_force += arm_collision.force;

        arm_collision.arm2->_filtered_collision_force += -arm_collision.force;
        arm_collision.arm2->_unfiltered_collision_force += -arm_collision.force;

        arm_collision.last_node_index1 = arm_collision.node_index1;
        arm_collision.last_interp1 = arm_collision.interp1;
        arm_collision.last_node_index2 = arm_collision.node_index2;
        arm_collision.last_interp2 = arm_collision.interp2;

        // remove the collision if the force has gone down to near 0
        if (arm_collision.force.norm() < 1e-5)
        {
            applyNodalForce(arm_collision.node_index1, arm_collision.interp1, -arm_collision.force);
            it = _virtuoso_arm_collisions.erase(it);
        }
        // otherwise, move on to the next collision
        else
        {
            ++it;
        }
    }

    _stale_frames = true;
}

/** Returns the axis-aligned bounding-box (AABB) for this Object in global simulation coordinates. */
Geometry::AABB VirtuosoArm::boundingBox() const
{
    return Geometry::AABB(Vec3r::Zero(), Vec3r::Zero());
}

Real VirtuosoArm::_outerTubeClearanceAngle(Real ot_trans)
{
    Real tubed_length = std::max(_ot_distal_straight_length - ot_trans, Real(0.0));

    Real sqrt_arg = tubed_length*tubed_length + 2*OT_ENDOSCOPE_CLEARANCE*OT_RADIUS_OF_CURVATURE - OT_ENDOSCOPE_CLEARANCE*OT_ENDOSCOPE_CLEARANCE;
    Real atan_arg = (tubed_length - std::sqrt(sqrt_arg)) / (OT_ENDOSCOPE_CLEARANCE - 2*OT_RADIUS_OF_CURVATURE);
    return 2*std::atan(atan_arg);
}

void VirtuosoArm::_toolAction()
{
    // if the manipulated object has not been given, do nothing
    if (!_tool_manipulated_object)
        return;

    if (_tool_type == ToolType::SPATULA)
        _spatulaToolAction();
    else if (_tool_type == ToolType::GRASPER)
        _grasperToolAction();
    else if (_tool_type == ToolType::CAUTERY)
        _cauteryToolAction();

    _last_tool_state = _tool_state;
}

void VirtuosoArm::_spatulaToolAction()
{
    // don't need to do anything for the spatula
}

void VirtuosoArm::_grasperToolAction()
{
    // if tool state has changed from 0 to 1, start grasping vertices inside grasping radius
    if (_tool_state == 1 && _last_tool_state == 0)
    {
        std::map<int, Vec3r> vertices_to_grasp;

        // quick and dirty way to find all vertices in a sphere
        Vec3r tool_position = _tool->tipFrame().origin();
        for (int theta = 0; theta < 360; theta+=30)
        {
            for (int phi = 0; phi < 360; phi+=30)
            {
                for (double p = 0; p < GRASPING_RADIUS; p+=GRASPING_RADIUS/5.0)
                {
                    const double x = tool_position[0] + p*std::sin(phi*M_PI/180)*std::cos(theta*M_PI/180);
                    const double y = tool_position[1] + p*std::sin(phi*M_PI/180)*std::sin(theta*M_PI/180);
                    const double z = tool_position[2] + p*std::cos(phi*M_PI/180);
                    int v = _tool_manipulated_object.mesh()->getClosestVertex(Vec3r(x, y, z));

                    // make sure v is inside grasping sphere
                    if ((tool_position - _tool_manipulated_object.mesh()->vertex(v)).norm() <= GRASPING_RADIUS)
                        if (!_tool_manipulated_object.vertexFixed(v))
                        {
                            const Vec3r attachment_offset = (_tool_manipulated_object.mesh()->vertex(v) - tool_position) * 0.9;
                            vertices_to_grasp[v] = attachment_offset;
                        }
                }
            }
        }

        // for (const auto& [v, offset] : vertices_to_grasp)
        // {
        //     Solver::ConstraintProjectorReferenceWrapper<Solver::AttachmentConstraint> proj_ref =
        //         _tool_manipulated_object.addAttachmentConstraint(v, &tool_position, offset);
        //     _grasping_constraints.push_back(std::move(proj_ref));
        // }
    }

    // if tool state has changed from 1 to 0, stop grasping
    else if (_tool_state == 0 && _last_tool_state == 1)
    {
        /** TODO: remove just the attachment constraints associated with grasping with this object */
        _tool_manipulated_object.clearAttachmentConstraints();
        _grasping_constraints.clear();
    }

    // apply tip forces
    Vec3r total_force = Vec3r::Zero();
    for (const auto& proj : _grasping_constraints)
    {
        std::vector<Vec3r> forces = proj.constraintForces();
        total_force += forces[0]; // attachment constraint only affects one vertex, so the vector only has 1 element
    }

    // smooth forces
    Vec3r new_tip_force = 0.99*tipForce() + -0.01* -total_force/1;
    setTipForce(new_tip_force);
    
}

void VirtuosoArm::_cauteryToolAction()
{
    if (_tool_manipulated_object.hasHeatSolver())
    {
        _tool_manipulated_object.heatSolver().clearVoltageBoundary();
    }

    if (_tool_state == 1)
    {
        /** Simple element removal on contact */
        if (_cutting_model == CuttingModel::INSTANT)
        {
            // stores (element index, element refinement level) pairs
            std::unordered_set<int> elements_to_remove;
            for (const auto& collision : _collision_constraints)
            {
                if (!collision.proj_ref.exists() || !collision.proj_ref.isValid())
                    continue;

                if (collision.node_index >= NUM_OT_FRAMES + NUM_IT_FRAMES && collision.proj_ref.lambda() > 0)
                {
                    // get element 
                    int element_index = collision.proj_ref.constraint()->elementIndex();
                    if (!_tool_manipulated_object.tetMesh()->elementValid(element_index))
                        continue;
                    
                    elements_to_remove.insert( element_index );                
                }
            }

            for (const auto& elem_index : elements_to_remove)
            {
                _tool_manipulated_object.removeElement(elem_index);
            }
        }

        /** Timer-based element removal. Each element in contact has their time-in-contact field incremented by dt. */
        if (_cutting_model == CuttingModel::TIMER)
        {
            Geometry::TetMesh* obj_mesh = _tool_manipulated_object.tetMesh();
            if (!obj_mesh->template hasElementProperty<Real>("time-in-contact"))
            {
                obj_mesh->template addElementProperty<Real>("time-in-contact", 0, true);
            }

            Geometry::MeshProperty<Real>& time_prop = obj_mesh->template getElementProperty<Real>("time-in-contact");
            std::unordered_set<int> elements_in_contact;
            for (const auto& collision : _collision_constraints)
            {
                if (!collision.proj_ref.exists() || !collision.proj_ref.isValid())
                    continue;

                if (collision.node_index >= NUM_OT_FRAMES + NUM_IT_FRAMES && collision.proj_ref.lambda() > 0)
                {
                    // get element 
                    int element_index = collision.proj_ref.constraint()->elementIndex();
                    if (!_tool_manipulated_object.tetMesh()->elementValid(element_index))
                        continue;
                    
                    elements_in_contact.insert(element_index);                
                }
            }

            for (const auto& elem_index : elements_in_contact)
            {
                Real old_time = time_prop.get(elem_index);
                time_prop.set(elem_index, old_time + _sim->wallClockdt());

                // if we've exceeded the threshold, remove the element
                if (old_time + _sim->wallClockdt() > _cutting_model_time_threshold)
                {
                    _tool_manipulated_object.removeElement(elem_index);
                    return;
                }
                
            }
        }
        

        if (_cutting_model == CuttingModel::THERMAL)
        {
            /** Code for applying voltage BC at tip */
            std::unordered_set<int> high_voltage_verts;
            for (const auto& collision : _collision_constraints)
            {
                if (!collision.proj_ref.exists() || !collision.proj_ref.isValid())
                    continue;

                if (collision.node_index >= NUM_OT_FRAMES + NUM_IT_FRAMES && collision.proj_ref.lambda() > 0)
                {
                    // get element 
                    int element_index = collision.proj_ref.constraint()->elementIndex();
                    if (!_tool_manipulated_object.tetMesh()->elementValid(element_index))
                        continue;

                    // set voltage at the face in collision
                    /** TODO: get the face index */
                    int face_index = -1;
                    const Vec3i& face = _tool_manipulated_object.mesh()->face(face_index);
                    if (_tool_manipulated_object.hasHeatSolver())
                    {
                        // set the closest vertex on the face to the voltage of the tool
                        // Eigen doesn't have argmax...
                        /** TODO: replace with the actual voltage of the cautery tool  */
                        Vec3r barys = collision.proj_ref.constraint()->barycentricCoordinates();
                        Real max_bary = barys.maxCoeff();
                        if (max_bary == barys[0])
                            // _tool_manipulated_object.heatSolver().setVoltageAtBoundary(face[0], 134);
                            high_voltage_verts.insert(face[0]);
                        else if (max_bary == barys[1])
                            // _tool_manipulated_object.heatSolver().setVoltageAtBoundary(face[1], 134);
                            high_voltage_verts.insert(face[1]);
                        else if (max_bary == barys[2])
                            // _tool_manipulated_object.heatSolver().setVoltageAtBoundary(face[2], 134);
                            high_voltage_verts.insert(face[2]);
                    }           
                }
            }

            for (const auto& vert : high_voltage_verts)
            {
                _tool_manipulated_object.heatSolver().setVoltageAtBoundary(vert, 134);
            }
            std::cout << "Number of high voltage verts: " << high_voltage_verts.size() << std::endl;
        }
    }
}

void VirtuosoArm::_recomputeCoordinateFrames()
{
    // compute arm (base) frame based on current position and orientation
    _arm_base_frame = Geometry::CoordinateFrame(Geometry::TransformationMatrix(_arm_base_rotation, _arm_base_position));

    // compute outer tube base frame
    // consists of rotating about the current z-axis according to outer tube rotation
    // and rotating about the new x-axis according to the clearance angle
    Geometry::TransformationMatrix T_rot_z(GeometryUtils::Rz(_ot_rotation), Vec3r::Zero());
    Geometry::TransformationMatrix T_rot_x(GeometryUtils::Rx(_outerTubeClearanceAngle(_ot_translation)), Vec3r::Zero());
    Geometry::CoordinateFrame ot_base_frame = _arm_base_frame * T_rot_z * T_rot_x;

    Real swept_angle = std::max(_ot_translation - _ot_distal_straight_length, Real(0.0)) / _ot_r_curvature; 
    // frames for the curved section of the outer tube
    for (int i = 0; i < NUM_OT_CURVE_FRAMES; i++)
    {
        Real cur_angle = i * swept_angle / (NUM_OT_CURVE_FRAMES - 1);
        const Vec3r curve_point(0.0, -_ot_r_curvature +_ot_r_curvature*std::cos(cur_angle), _ot_r_curvature*std::sin(cur_angle));
        Geometry::TransformationMatrix T(GeometryUtils::Rx(cur_angle), curve_point);
        _ot_frames[i] = ot_base_frame * T;
    }

    // frames for the straight section of the outer tube
    for (int i = 0; i < NUM_OT_STRAIGHT_FRAMES; i++)
    {
        // relative transformation along z-axis
        Geometry::TransformationMatrix T(Mat3r::Identity(), Vec3r(0,0, std::min(_ot_translation, _ot_distal_straight_length) / NUM_OT_STRAIGHT_FRAMES));
        _ot_frames[NUM_OT_CURVE_FRAMES + i] = _ot_frames[NUM_OT_CURVE_FRAMES + i -1] * T;
    }

    // frames for the inner tube
    // unrotate about z-axis for outer tube rotation, then rotate about z-axis with inner tube rotation to get to the beginning of the exposed part of the inner tube
    Geometry::TransformationMatrix T_rot_z_outer_to_inner(GeometryUtils::Rz(_it_rotation-_ot_rotation), Vec3r::Zero());
    _it_frames[0] = _ot_frames.back() * T_rot_z_outer_to_inner;
    const Real it_length = std::max(Real(0.0), _it_translation - _ot_translation);      // exposed length of the inner tube
    for (int i = 1; i < NUM_IT_FRAMES; i++)
    {
        // relative transformation matrix along z-axis
        Geometry::TransformationMatrix T(Mat3r::Identity(), Vec3r(0, 0, it_length / (NUM_IT_FRAMES - 1)));
        _it_frames[i] = _it_frames[i-1] * T;
    }

    _stale_frames = false;

}

void VirtuosoArm::_recomputeCoordinateFramesStaticsModelWithNodalForces()
{
    // compute arm base frame based on current position and orientation
    _arm_base_frame = Geometry::CoordinateFrame(Geometry::TransformationMatrix(_arm_base_rotation, _arm_base_position));

    // compute outer tube base frame
    // consists of rotating about the current z-axis according to outer tube rotation
    // and rotating about the new x-axis according to the clearance angle
    Geometry::TransformationMatrix T_rot_z(GeometryUtils::Rz(_ot_rotation), Vec3r::Zero());
    Geometry::TransformationMatrix T_rot_x(GeometryUtils::Rx(_outerTubeClearanceAngle(_ot_translation)), Vec3r::Zero());
    Geometry::CoordinateFrame ot_base_frame = _arm_base_frame * T_rot_z * T_rot_x;

    // calculate internal forces and moments carried at the base of the tube collection
    const Vec3r& tip_position = innerTubeEndFrame().origin();
    // add contributions from force and moment at tube tip
    Vec3r base_moment = _tip_moment + (tip_position - _arm_base_frame.origin()).cross(_tip_force);
    Vec3r base_force = _tip_force;
    // add contributions from forces applied along the tube
    for (int i = 0; i < NUM_OT_FRAMES; i++)
    {
        Vec3r moment_arm = _ot_frames[i].origin() - _arm_base_frame.origin();
        base_moment += moment_arm.cross(_ot_nodal_forces[i]);
        base_force += _ot_nodal_forces[i];
    }
    for (int i = 0; i < NUM_IT_FRAMES; i++)
    {
        Vec3r moment_arm = _it_frames[i].origin() - _arm_base_frame.origin();
        base_moment += moment_arm.cross(_it_nodal_forces[i]);
        base_force += _it_nodal_forces[i];
    }

    // EVERYTHING FROM NOW ON WILL USE MM
    // compute cross section properties
    Real E_mm = E/1000/1000;    // Young's Modulus (N/mm^2)
    Real G_mm = G/1000/1000;   // Shear modulus (N/mm^2)

    Real ot_outer_dia_mm = _ot_outer_dia * 1000;
    Real ot_inner_dia_mm = _ot_inner_dia * 1000;
    Real it_outer_dia_mm = _it_outer_dia * 1000;
    Real it_inner_dia_mm = _it_inner_dia * 1000;
    Real I_ot = M_PI/4 * (ot_outer_dia_mm*ot_outer_dia_mm*ot_outer_dia_mm*ot_outer_dia_mm/16 - ot_inner_dia_mm*ot_inner_dia_mm*ot_inner_dia_mm*ot_inner_dia_mm/16);
    Real I_it = M_PI/4 * (it_outer_dia_mm*it_outer_dia_mm*it_outer_dia_mm*it_outer_dia_mm/16 - it_inner_dia_mm*it_inner_dia_mm*it_inner_dia_mm*it_inner_dia_mm/16);
    Real J_ot = 2*I_ot;
    Real J_it = 2*I_it;

    Real EI_tool_tube = 0;
    Real GJ_tool_tube = 0;
    if (hasTool())
    {
        // these will only be nonzero if the tool threaded through the lumen of the inner tube has a non-negligible bending stiffness
        TubeProperties tool_tube_properties = _tool->tubeProperties();
        EI_tool_tube = tool_tube_properties.E * tool_tube_properties.I * 1000 * 1000;
        GJ_tool_tube = tool_tube_properties.G * tool_tube_properties.J * 1000 * 1000;
    }
    

    // the inverse stiffness matrix for the outer tube
    const Vec3r ot_K_inv(1/(E_mm*I_ot + E_mm*I_it + EI_tool_tube), 1/(E_mm*I_ot + E_mm*I_it + EI_tool_tube), 1/(G_mm*J_ot));
    // the inverse stiffness matrix for the inner tube
    const Vec3r it_K_inv(1/(E_mm*I_it + EI_tool_tube), 1/(E_mm*I_it + EI_tool_tube), 1/(G_mm*J_it + GJ_tool_tube));

    /** Starting state for the forward integration along the robot arm, at the outer tube base. */
    TubeIntegrationState ot_base_state;
    ot_base_state.position = ot_base_frame.origin()*1000;   // put position in mm
    ot_base_state.orientation = ot_base_frame.transform().rotMat();
    ot_base_state.internal_force = base_force;
    ot_base_state.internal_moment = base_moment*1000; // put it in N-mm
    ot_base_state.torsional_displacement = _ot_rotation;



    /** === Integrate the curved section of the outer tube === */

    // the final integration state at the end of the curved section (used to initialize the next part of the tube)
    TubeIntegrationState ot_curve_end_state;

    // compute the exposed length of the precurved part of the outer tube
    Real curved_length = (_ot_translation - _ot_distal_straight_length)*1000; // in mm

    // if there is some precurved part of the outer tube exposed (i.e. OT translation > the distal straight length)
    // integrate along the exposed precurved section of the outer tube
    if (curved_length > 0)
    {
        // the precurvature vector (u*), in mm
        // for the precurved section of the outer tube, there is a positive precurvature about the x-axis (in the YZ plane)
        // const Vec3r precurvature(1/_ot_r_curvature/1000, 0, 0);
        const Vec3r precurvature(65.0/1000.0, 0, 0);

        // compute the distance along the entire tube collection for placing the outer curve frames (in mm)
        std::vector<Real> ot_curve_s(NUM_OT_CURVE_FRAMES);  
        for (int i = 0; i < NUM_OT_CURVE_FRAMES; i++)
            ot_curve_s[i] = i* (curved_length / (NUM_OT_CURVE_FRAMES-1));

        // integrate with RK4 along the curved section of the outer tube
        std::vector<TubeIntegrationState::VecType> ot_curve_states = 
            _integrateTubeWithForceBoundariesRK4(ot_base_state, ot_curve_s, _ot_nodal_forces.cbegin(), ot_K_inv, precurvature);
        
        // set the transform along the curved part of the tube from the integration results
        for (int i = 0; i < NUM_OT_CURVE_FRAMES; i++)
        {
            TubeIntegrationState state = TubeIntegrationState::fromVec(ot_curve_states[i]);
            _ot_frames[i].setTransform(Geometry::TransformationMatrix(state.orientation, state.position/1000).asMatrix()); // convert position back to m
        }
        
        // final integration state
        ot_curve_end_state = TubeIntegrationState::fromVec(ot_curve_states.back());
    }
    // otherwise, the frames for the precurved part of the outer tube are identical to the frame at the outer tube base
    else
    {
        for (int i = 0; i < NUM_OT_CURVE_FRAMES; i++)
        {
            _ot_frames[i].setTransform(ot_base_frame.transform().asMatrix());
        }
        ot_curve_end_state = ot_base_state;
    }


    /** === Integrate the straight section of the outer tube === */

    Real straight_length = std::min(_ot_translation, _ot_distal_straight_length)*1000; // length of outer tube straight section (in mm)
    // the final integration state at the end of the outer tube (used to initialize the next part of the robot arm)
    TubeIntegrationState ot_end_state;

    // if there is some straight part of the outer tube exposed (i.e. if OT translation > 0)
    // integrate along the straight part of the outer tube
    if (straight_length > 0)
    {   
        // compute the distance along the entire tube collection for placing the outer tube straight frames (in mm)
        std::vector<Real> ot_straight_s(NUM_OT_STRAIGHT_FRAMES+1);
        for (int i = 0; i < NUM_OT_STRAIGHT_FRAMES+1; i++)
            ot_straight_s[i] = curved_length + i*straight_length / NUM_OT_STRAIGHT_FRAMES;

        // integrate with RK4 along the straight section of the outer tube
        std::vector<TubeIntegrationState::VecType> ot_straight_states = 
            _integrateTubeWithForceBoundariesRK4(ot_curve_end_state, ot_straight_s, _ot_nodal_forces.cbegin() + NUM_OT_CURVE_FRAMES, ot_K_inv, Vec3r(0,0,0));
        
        // set the transforms along the straight part of the otuer tube from the integration results
        for (int i = 0; i < NUM_OT_STRAIGHT_FRAMES; i++)
        {
            TubeIntegrationState state = TubeIntegrationState::fromVec(ot_straight_states[i+1]);
            _ot_frames[NUM_OT_CURVE_FRAMES + i].setTransform(Geometry::TransformationMatrix(state.orientation, state.position/1000).asMatrix()); // convert position back to m
        }

        // final integration state
        ot_end_state = TubeIntegrationState::fromVec(ot_straight_states.back());
    }
    // otherwise, the frames for the straight part of the outer tube are identical to the frame at the outer tube base
    else
    {
        for (int i = 0; i < NUM_OT_STRAIGHT_FRAMES; i++)
        {
            _ot_frames[NUM_OT_CURVE_FRAMES + i].setTransform(_ot_frames[NUM_OT_CURVE_FRAMES-1].transform().asMatrix());
        }
        
        ot_end_state = ot_curve_end_state;
    }

    
    /** === Integrate the inner tube === */

    // exposed length of the inner tube in mm
    const Real it_length = std::max(Real(0.0), _it_translation - _ot_translation) * 1000;      
    
    // the starting integration state of the inner tube
    // the outer and inner tube are not torsionally coupled, so we need to "unrotate" the outer tube end frame about the z-axis 
    TubeIntegrationState it_start_state = ot_end_state;
    it_start_state.orientation *= GeometryUtils::Rz(_it_rotation - it_start_state.torsional_displacement);
    it_start_state.torsional_displacement = _it_rotation;

    // the integration state at the end of the inner tube (used to initialize the next part of the robot arm)
    TubeIntegrationState it_end_state;

    // if there is some part of the inner tube exposed (i.e. if IT translation > OT translation)
    // integrate along the inner tube
    if (it_length > 0)
    {
        // compute the distance along the entire tube collection for placing the inner tube frames (in mm)
        std::vector<Real> it_s(NUM_IT_FRAMES);
        for (int i = 0; i < NUM_IT_FRAMES; i++)     
        {
            it_s[i] = _ot_translation*1000 + i*it_length / (NUM_IT_FRAMES - 1);
        }
        
        // integrate with RK4 along the inner tube
        std::vector<TubeIntegrationState::VecType> it_states = _integrateTubeWithForceBoundariesRK4(it_start_state, it_s, _it_nodal_forces.cbegin(), it_K_inv, Vec3r(0,0,0));

        // set the transforms along the inner tube from the integration results
        for (int i = 0; i < NUM_IT_FRAMES; i++)
        {
            TubeIntegrationState state = TubeIntegrationState::fromVec(it_states[i]);
            _it_frames[i].setTransform(Geometry::TransformationMatrix(state.orientation, state.position/1000).asMatrix()); // convert position back to m
        }

        // final integration state
        it_end_state = TubeIntegrationState::fromVec(it_states.back());

    }
    // otherwise, the frames for the inner tube are all the same, and equal to the inner tube base frame
    else
    {
        Geometry::TransformationMatrix it_start_transform(it_start_state.orientation, it_start_state.position/1000); // convert position back to m
        for (int i = 0; i < NUM_IT_FRAMES; i++)
        {
            _it_frames[i].setTransform(it_start_transform.asMatrix());
        }

        it_end_state = it_start_state;
    }


    /** === Integrate the tool tube === */
    // if (hasTool())
    // {
    //     const Vec3r tt_K_inv(1/(EI_tool_tube), 1/(EI_tool_tube), 1/(GJ_tool_tube));

    //     // compute the distance along the entire tube collection for placing the inner tube frames (in mm)
    //     std::vector<Real> tt_s(NUM_TT_FRAMES);
    //     for (int i = 0; i < NUM_TT_FRAMES; i++)     
    //     {
    //         tt_s[i] = _it_translation*1000 + i*_tool_tube_length*1000 / (NUM_TT_FRAMES - 1);
    //     }
        
    //     // integrate with RK4 along the tool tube
    //     std::vector<TubeIntegrationState::VecType> tt_states = _integrateTubeWithForceBoundariesRK4(it_end_state, tt_s, _tt_nodal_forces.cbegin(), tt_K_inv, Vec3r(0,0,0));

    //     // set the transforms along the tool tube from the integration results
    //     for (int i = 0; i < NUM_TT_FRAMES; i++)
    //     {
    //         TubeIntegrationState state = TubeIntegrationState::fromVec(tt_states[i]);
    //         _tt_frames[i].setTransform(Geometry::TransformationMatrix(state.orientation, state.position/1000).asMatrix()); // convert position back to m
    //     }
    // }
    // else
    // {
    //     Geometry::TransformationMatrix tt_start_transform(it_end_state.orientation, it_end_state.position/1000); // convert position back to m
    //     for (int i = 0; i < NUM_TT_FRAMES; i++)
    //     {
    //         _tt_frames[i].setTransform(tt_start_transform.asMatrix());
    //     }
    // }
    _stale_frames = false;
}

std::vector<VirtuosoArm::TubeIntegrationState::VecType> VirtuosoArm::_integrateTubeRK4(const VirtuosoArm::TubeIntegrationState& tube_base_state, const std::vector<Real>& s, const Vec3r& K_inv, const Vec3r& u_star) const
{

    // a function that uses Cosserat rod ODEs to return the arc length derivative of the state at a given point along the tube
    auto ode_func = [](Real /*s*/, const TubeIntegrationState::VecType& state_vec, const TubeIntegrationParams& params) -> TubeIntegrationState::VecType {
        
        const TubeIntegrationState state = TubeIntegrationState::fromVec(state_vec);
        TubeIntegrationState state_dot; // note that these are ARC LENGTH derivatives, not time derivatives

        // p_dot = R * e_3
        state_dot.position = state.orientation.col(2);  

        // u = u_star + K^-1 * R^T * m
        const Vec3r u = params.precurvature + params.K_inv.asDiagonal() * state.orientation.transpose() * state.internal_moment;

        // R_dot = R * u^
        state_dot.orientation = state.orientation * MathUtils::Skew3(u);

        // internal force is constant ==> n_dot = 0
        state_dot.internal_force = Vec3r::Zero();

        // m_dot = -p x n
        state_dot.internal_moment = -state_dot.position.cross( state.internal_force );

        // theta_dot = u_3
        state_dot.torsional_displacement = u[2];

        return TubeIntegrationState::toVec(state_dot);
    };

    // integration parameters for the tube consist of the precurvature of the tube and the inverse bending stiffnesses
    TubeIntegrationParams params;
    params.precurvature = u_star;
    params.K_inv = K_inv;

    return MathUtils::RK4<TubeIntegrationState::VecType, TubeIntegrationParams>(TubeIntegrationState::toVec(tube_base_state), s, params, ode_func);
}

template <typename ForceIterator>
std::vector<VirtuosoArm::TubeIntegrationState::VecType> VirtuosoArm::_integrateTubeWithForceBoundariesRK4(
        const TubeIntegrationState& tube_base_state, const std::vector<Real>& s, ForceIterator force_iterator,
        const Vec3r& K_inv, const Vec3r& u_star) const
{
    // a function that uses Cosserat rod ODEs to return the arc length derivative of the state at a given point along the tube
    auto ode_func = [](Real /*s*/, const TubeIntegrationState::VecType& state_vec, const TubeIntegrationParams& params) -> TubeIntegrationState::VecType {
        
        const TubeIntegrationState state = TubeIntegrationState::fromVec(state_vec);
        TubeIntegrationState state_dot; // note that these are ARC LENGTH derivatives, not time derivatives

        // p_dot = R * e_3
        state_dot.position = state.orientation.col(2);  

        // u = u_star + K^-1 * R^T * m
        const Vec3r u = params.precurvature + params.K_inv.asDiagonal() * state.orientation.transpose() * state.internal_moment;

        // R_dot = R * u^
        state_dot.orientation = state.orientation * MathUtils::Skew3(u);

        // internal force is constant ==> n_dot = 0
        state_dot.internal_force = Vec3r::Zero();

        // m_dot = -p x n
        state_dot.internal_moment = -state_dot.position.cross( state.internal_force );

        // theta_dot = u_3
        state_dot.torsional_displacement = u[2];

        return TubeIntegrationState::toVec(state_dot);
    };

    // we need to break up the integration of the tube depending on the number of applied forces along its length
    std::vector<unsigned> tube_intervals;
    tube_intervals.reserve(s.size());
    tube_intervals.push_back(0);
    for (unsigned i = 1; i < s.size()-1; i++)
    {
        const Vec3r& F = *(force_iterator + i);
        // if force at this node is not zero, add its index to intervals
        if (!F.isZero(1e-8))
        {
            tube_intervals.push_back(i);
        }
    }
    tube_intervals.push_back(s.size()-1);

    // REMOVE: print out tube intervals
    // std::cout << "Tube intervals: ";
    // for (unsigned i = 0; i < tube_intervals.size()-1; i++)
    // {
    //     unsigned int_start = tube_intervals[i];
    //     unsigned int_end = tube_intervals[i+1];
    //     std::cout << "[" << int_start << ", " << int_end << "]";
    // }
    // std::cout << std::endl;

    // integration parameters for the tube consist of the precurvature of the tube and the inverse bending stiffnesses
    TubeIntegrationParams params;
    params.precurvature = u_star;
    params.K_inv = K_inv;

    std::vector<TubeIntegrationState::VecType> states(s.size());
    states[0] = TubeIntegrationState::toVec(tube_base_state);

    // integrate through each interval
    for (unsigned i = 0; i < tube_intervals.size()-1; i++)
    {
        unsigned int_start = tube_intervals[i];
        unsigned int_end = tube_intervals[i+1];

        const Vec3r& applied_force = *(force_iterator + int_start);
        const Vec3r current_internal_force = TubeIntegrationState::internalForceFromVec(states[int_start]);
        TubeIntegrationState::setInternalForceInVec(states[int_start], current_internal_force - applied_force);

        MathUtils::RK4<TubeIntegrationState::VecType, TubeIntegrationParams>(states[int_start], s.cbegin()+int_start, s.cbegin()+int_end+1, params, ode_func, states.begin()+int_start);
    }

    return states;
}

Geometry::TransformationMatrix VirtuosoArm::_computeTipTransform(Real ot_rot, Real ot_trans, Real it_rot, Real it_trans)
{
    Geometry::CoordinateFrame arm_base_frame = Geometry::CoordinateFrame(Geometry::TransformationMatrix(_arm_base_rotation, _arm_base_position));

    // compute outer tube base frame
    // consists of rotating about the current z-axis according to outer tube rotation
    // and rotating about the new x-axis according to the clearance angle
    Geometry::TransformationMatrix T_rot_z(GeometryUtils::Rz(ot_rot), Vec3r::Zero());
    Geometry::TransformationMatrix T_rot_x(GeometryUtils::Rx(_outerTubeClearanceAngle(ot_trans)), Vec3r::Zero());
    Geometry::CoordinateFrame ot_base_frame = arm_base_frame * T_rot_z * T_rot_x;

    Real swept_angle = std::max(ot_trans - _ot_distal_straight_length, Real(0.0)) / _ot_r_curvature; 
    // frames for the curved section of the outer tube
    const Vec3r curve_point(0.0, -_ot_r_curvature +_ot_r_curvature*std::cos(swept_angle), _ot_r_curvature*std::sin(swept_angle));
    Geometry::TransformationMatrix T_curve(GeometryUtils::Rx(swept_angle), curve_point);
    Geometry::CoordinateFrame ot_curve_end_frame = ot_base_frame * T_curve;

    // frames for the straight section of the outer tube
    // relative transformation along z-axis
    Geometry::TransformationMatrix T_z_trans_ot(Mat3r::Identity(), Vec3r(0,0, std::min(ot_trans, _ot_distal_straight_length)));
    Geometry::CoordinateFrame ot_end_frame = ot_curve_end_frame * T_z_trans_ot;

    // frames for the inner tube
    // unrotate about z-axis for outer tube rotation, then rotate about z-axis with inner tube rotation to get to the beginning of the exposed part of the inner tube
    Geometry::TransformationMatrix T_rot_z_outer_to_inner(GeometryUtils::Rz(it_rot - ot_rot), Vec3r::Zero());
    const Real it_length = std::max(Real(0.0), it_trans - ot_trans);      // exposed length of the inner tube
    // relative transformation matrix along z-axis
    Geometry::TransformationMatrix T_z_trans_it(Mat3r::Identity(), Vec3r(0, 0, it_length));

    Geometry::CoordinateFrame it_end_frame = ot_end_frame * T_rot_z_outer_to_inner * T_z_trans_it;
    
    if (hasTool())
    {
        // if there is a tool being used, the tip of the robot arm is the tip of this tool
        // move along the local z-axis by the required amount specified by the tool in use
        Geometry::TransformationMatrix T_z_trans_tt(Mat3r::Identity(), _tool->tipOffset());
        Geometry::CoordinateFrame tt_end_frame = it_end_frame * T_z_trans_tt;
        return tt_end_frame.transform();
    }
    else
    {
        return it_end_frame.transform();
    }
    
}

void VirtuosoArm::_jacobianDifferentialInverseKinematics(const Vec3r& dx)
{
    // _recomputeCoordinateFrames();
    // _recomputeCoordinateFramesStaticsModelWithNodalForces();
    const Geometry::TransformationMatrix T_tip = _computeTipTransform(_ot_rotation, _ot_translation, _it_rotation, _it_translation);
    const Vec3r target_position = T_tip.translation() + dx;

    for (int i = 0; i < 10; i++)
    {
        const Geometry::TransformationMatrix T_tip = _computeTipTransform(_ot_rotation, _ot_translation, _it_rotation, _it_translation);
        const Vec3r pos_err = target_position - T_tip.translation();
        if (pos_err.norm() < 1e-10)
            break;

        const Mat3r J_a = _3DOFAnalyticalHybridJacobian();
        const Vec3r dq = J_a.colPivHouseholderQr().solve(pos_err);
        _ot_rotation += dq[0];
        _ot_translation += dq[1];
        _it_translation += dq[2];

        // enforce joint limits
        _ot_translation = std::clamp(_ot_translation, Real(0.0), Real(50.0e-3));
        _it_translation = std::clamp(_it_translation, Real(0.0), Real(50.0e-3));

        // _recomputeCoordinateFrames();
        _recomputeCoordinateFramesStaticsModelWithNodalForces();
    }

    _stale_frames = true;
}

void VirtuosoArm::_hybridDifferentialInverseKinematics(const Vec3r& dx)
{
    if (dx.norm() < 1e-6)
        return;

    // _recomputeCoordinateFrames();
    // _recomputeCoordinateFramesStaticsModelWithNodalForces();
    const Geometry::TransformationMatrix T_tip = _computeTipTransform(_ot_rotation, _ot_translation, _it_rotation, _it_translation);
    const Vec3r target_position = T_tip.translation() + dx;

    // solve for the outer tube rotation analytically
    Geometry::TransformationMatrix global_to_arm_base = _arm_base_frame.transform().inverse();
    const Vec3r arm_base_pt = global_to_arm_base.rotMat() * target_position + global_to_arm_base.translation();
    // the multiple of 2*pi to add to angles (avoids angle jumps from 180 to -180 and vice versa)
    int n_rev = static_cast<int>((_ot_rotation+M_PI) / (2*M_PI));
    Real target_angle = n_rev*2*M_PI + std::atan2(arm_base_pt[0], -arm_base_pt[1]);

    // avoid angle jumps from 180 to -180 and from -180 to 180
    if (_ot_rotation > n_rev*2*M_PI + M_PI/2.0 && target_angle < n_rev*2*M_PI - M_PI/2.0)
        target_angle += 2*M_PI;
    else if (_ot_rotation < n_rev*2*M_PI - M_PI/2.0 && target_angle > n_rev*2*M_PI + M_PI/2.0)
        target_angle += -2*M_PI;

    // compute max allowable changes in joint variables, given by motor limitations
    Real max_ot_rot_change = _max_ot_rotation_speed * _sim->wallClockdt(); // rad/s * s
    Real max_ot_trans_change = _max_ot_translation_speed * _sim->wallClockdt(); // m/s * s
    Real max_it_trans_change = _max_it_translation_speed * _sim->wallClockdt(); // m/s * s

    // clamp the change in outer tube rotation and update
    Real d_ot_rot = std::clamp(target_angle - _ot_rotation, -max_ot_rot_change, max_ot_rot_change);
    _ot_rotation += d_ot_rot;


    /** run 1 iteration of Jacobian-based differential inverse kinematics */
    // the solution may not be exact THIS frame, but will converge over a few successive frames
    // we can get away with this because we update at such a fast rate

    // compute position error
    const Geometry::TransformationMatrix T_tip_new = _computeTipTransform(_ot_rotation, _ot_translation, _it_rotation, _it_translation);
    const Vec3r pos_err = target_position - T_tip_new.translation();

    // get Jacobian and solve for joint updates (only outer tube translation and inner tube translation are used)
    const Mat3r J_a = _3DOFAnalyticalHybridJacobian();
    const Vec3r dq = J_a.colPivHouseholderQr().solve(pos_err);
    // update translational joint variables, clamping them to their acceptable ranges
    _ot_translation += std::clamp(dq[1], -max_ot_trans_change, max_ot_trans_change);
    _it_translation += std::clamp(dq[2], -max_it_trans_change, max_it_trans_change);

    // enforce constraints
    _ot_translation = std::clamp(_ot_translation, Real(0.0), Real(MAX_OT_TRANSLATION));
    _it_translation = std::clamp(_it_translation, Real(0.0), Real(MAX_IT_TRANSLATION));
    if (_it_translation < _ot_translation)
    {
        const Real d = _ot_translation - _it_translation;
        _it_translation += d/2;
        _ot_translation -= d/2;
    }
    

    // mark that the joint state has been updated so the coordinate frames are stale
    _stale_frames = true;
}

Eigen::Matrix<Real,6,3> VirtuosoArm::_3DOFSpatialJacobian()
{

    // because there are max() and min() expressions used, we have a couple different cases for the derivatives
    // when outer tube translation >= length of the outer tube straight section, we have some curve in the outer tube
    Real dalpha_d1, dmin_d1;
    if (_ot_translation >= _ot_distal_straight_length)
    {
        dalpha_d1 = 1/_ot_r_curvature;    // partial of alpha (outer tube curve swept angle) w.r.t outer tube translation
        dmin_d1 = 0;                // partial of min(length of OT straight section, outer tube translation) w.r.t. outer tube translation
    }
    else
    {                                
        dalpha_d1 = 0;
        dmin_d1 = 1;
    }

    // when outer tube translation >= inner tube translation, no inner tube is showing
    Real dmax_d1, dmax_d2;
    if (_ot_translation >= _it_translation)
    {
        dmax_d1 = 0;    // partial of max(0, inner tube translation - outer tube translation) w.r.t. outer tube translation
        dmax_d2 = 0;    // partial of max(0, inner tube translation - outer tube translation) w.r.t. inner tube translation
    }             
        
    else    
    {
        dmax_d1 = -1;
        dmax_d2 = 1;
    }

    // derviative of max(length of OT straight section - OT translation, 0)
    Real dtubed_length_d1;
    if (_ot_translation > _ot_distal_straight_length)
    {
        dtubed_length_d1 = 0;
    }
    else
    {
        dtubed_length_d1 = -1;
    }

    // derivative of clearance angle w.r.t. d1
    Real tubed_length = std::max(_ot_distal_straight_length - _ot_translation, Real(0.0));
    Real sqrt_arg = tubed_length*tubed_length + 2*OT_ENDOSCOPE_CLEARANCE*OT_RADIUS_OF_CURVATURE - OT_ENDOSCOPE_CLEARANCE*OT_ENDOSCOPE_CLEARANCE;
    Real atan_arg = (tubed_length - std::sqrt(sqrt_arg)) / (OT_ENDOSCOPE_CLEARANCE - 2*OT_RADIUS_OF_CURVATURE);
    Real dca_d1 = 2 / (1+atan_arg*atan_arg) / (OT_ENDOSCOPE_CLEARANCE - 2*OT_RADIUS_OF_CURVATURE) * (1 - tubed_length/std::sqrt(sqrt_arg)) * dtubed_length_d1;
    
    Real clearance_angle = _outerTubeClearanceAngle(_ot_translation);
    const Real alpha = std::max(_ot_translation - _ot_distal_straight_length, Real(0.0)) / _ot_r_curvature;    // angle swept by the outer tube curve
    const Real beta = _it_rotation - _ot_rotation;    // difference in angle between outer tube and inner tube
    const Real straight_length = std::min(_ot_distal_straight_length, _ot_translation) + std::max(Real(0.0), _it_translation - _ot_translation); // length of the straight section of the combined outer + inner tube

    // precompute some sin and cos
    const Real ct1 = std::cos(_ot_rotation);
    const Real st1 = std::sin(_ot_rotation);
    const Real ca = std::cos(alpha);
    const Real sa = std::sin(alpha);
    const Real cb = std::cos(beta);
    const Real sb = std::sin(beta);
    const Real cg = std::cos(clearance_angle);
    const Real sg = std::sin(clearance_angle);

    // the chain of transforms that make up forward kinematics
    Mat4r T1, T2, T3, T4;
    T1 <<   ct1, -st1, 0, 0,
            st1, ct1, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;
    
    T2 <<   1, 0, 0, 0,
            0, cg, -sg, 0,
            0, sg, cg, 0,
            0, 0, 0, 1;
    
    T3 <<   1, 0, 0, 0,
            0, ca, -sa, _ot_r_curvature*ca - _ot_r_curvature,
            0, sa, ca, _ot_r_curvature*sa,
            0, 0, 0, 1;
    
    T4 <<   cb, -sb, 0, 0,
            sb, cb, 0, 0,
            0, 0, 1, straight_length,
            0, 0, 0, 1;
            
    // derivatives w.r.t outer tube rotation (theta1)
    Mat4r dT1_dt1, dT4_dt1;
    dT1_dt1 <<  -st1, -ct1, 0, 0,
                ct1, -st1, 0, 0,
                0, 0, 0, 0,
                0, 0, 0, 0;
    
    dT4_dt1 <<  sb, cb, 0, 0,
                -cb, sb, 0, 0,
                0, 0, 0, 0,
                0, 0, 0, 0;

    // derivatives w.r.t. outer tube translation (d1)
    Mat4r dT2_dd1, dT3_dd1, dT4_dd1;
    dT2_dd1 <<  0, 0, 0, 0,
                0, -sg*dca_d1, -cg*dca_d1, 0,
                0, cg*dca_d1, -sg*dca_d1, 0, 
                0, 0, 0, 0;
    
    dT3_dd1 <<  0, 0, 0, 0,
                0, -sa*dalpha_d1, -ca*dalpha_d1, -_ot_r_curvature*sa*dalpha_d1,
                0, ca*dalpha_d1, -sa*dalpha_d1, _ot_r_curvature*ca*dalpha_d1,
                0, 0, 0, 0;
    
    dT4_dd1 <<  0, 0, 0, 0,
                0, 0, 0, 0,
                0, 0, 0, dmin_d1 + dmax_d1,
                0, 0, 0, 0;

    // derivatives w.r.t. inner tube translation (d2)
    Mat4r dT4_dd2;
    dT4_dd2 <<  0, 0, 0, 0,
                0, 0, 0, 0,
                0, 0, 0, dmax_d2,
                0, 0, 0, 0;


    // using product rule, evaluate dT/dq
    Mat4r dT_d_ot_rot, dT_d_ot_trans, dT_d_it_trans;

    if (hasTool())
    {
        // transform from inner tube tip to tool tip
        Mat4r T5 = Mat4r::Identity();
        T5.block<3,1>(0,3) = _tool->tipOffset();

        dT_d_ot_rot = dT1_dt1*T2*T3*T4*T5 + T1*T2*T3*dT4_dt1*T5;
        dT_d_ot_trans = T1*dT2_dd1*T3*T4*T5 + T1*T2*dT3_dd1*T4*T5 + T1*T2*T3*dT4_dd1*T5;
        dT_d_it_trans = T1*T2*T3*dT4_dd2*T5;
    }
    else
    {
        dT_d_ot_rot = dT1_dt1*T2*T3*T4 + T1*T2*T3*dT4_dt1;
        dT_d_ot_trans = T1*dT2_dd1*T3*T4 + T1*T2*dT3_dd1*T4 + T1*T2*T3*dT4_dd1;
        dT_d_it_trans = T1*T2*T3*dT4_dd2; 
    }

    

    // and assemble them into the spatial Jacobian
    const Geometry::TransformationMatrix T_tip = _computeTipTransform(_ot_rotation, _ot_translation, _it_rotation, _it_translation);
    const Geometry::TransformationMatrix T_inv = T_tip.inverse();
    
    Eigen::Matrix<Real,6,3> J_s;
    J_s.col(0) = GeometryUtils::Vee_SE3(_arm_base_frame.transform().asMatrix() * dT_d_ot_rot * T_inv.asMatrix());
    J_s.col(1) = GeometryUtils::Vee_SE3(_arm_base_frame.transform().asMatrix() * dT_d_ot_trans * T_inv.asMatrix());
    J_s.col(2) = GeometryUtils::Vee_SE3(_arm_base_frame.transform().asMatrix() * dT_d_it_trans * T_inv.asMatrix());

    return J_s;
}

Eigen::Matrix<Real,6,3> VirtuosoArm::_3DOFNumericalSpatialJacobian()
{
    if (_stale_frames)
        // _recomputeCoordinateFrames();
        _recomputeCoordinateFramesStaticsModelWithNodalForces();

    Eigen::Matrix<Real,6,3> J_s;

    const Geometry::TransformationMatrix orig_transform = innerTubeEndFrame().transform();

    // std::cout << "Tip transform (_recomputeCoordinateFrames):\n" << orig_transform.asMatrix() << std::endl;
    // std::cout << "Tip transform (_computeTipTransform):\n" << _computeTipTransform(_ot_rotation, _ot_translation, _it_rotation, _it_translation).asMatrix() << std::endl;
    const Geometry::TransformationMatrix orig_transform_inv = orig_transform.inverse();

    // amount to vary each joint variable
    const Real delta = 1e-6;

    {
        _ot_rotation += delta;
        _recomputeCoordinateFramesStaticsModelWithNodalForces();
        Geometry::TransformationMatrix new_transform = innerTubeEndFrame().transform();
        J_s.col(0) = GeometryUtils::Vee_SE3( (new_transform.asMatrix() - orig_transform.asMatrix())/delta * orig_transform_inv.asMatrix());
        _ot_rotation -= delta;
    }
    {
        _ot_translation += delta;
        _recomputeCoordinateFramesStaticsModelWithNodalForces();
        Geometry::TransformationMatrix new_transform = innerTubeEndFrame().transform();
        J_s.col(1) = GeometryUtils::Vee_SE3( (new_transform.asMatrix() - orig_transform.asMatrix())/delta * orig_transform_inv.asMatrix());
        _ot_translation -= delta;
    }
    {
        _it_translation += delta;
        _recomputeCoordinateFramesStaticsModelWithNodalForces();
        Geometry::TransformationMatrix new_transform = innerTubeEndFrame().transform();
        J_s.col(2) = GeometryUtils::Vee_SE3( (new_transform.asMatrix() - orig_transform.asMatrix())/delta * orig_transform_inv.asMatrix());
        _it_translation -= delta;
    }

    _recomputeCoordinateFramesStaticsModelWithNodalForces();

    return J_s;
    
}

Mat3r VirtuosoArm::_3DOFAnalyticalHybridJacobian()
{
    // make sure coordinate frames are up to date
    // if (_stale_frames)      
        // _recomputeCoordinateFrames();
        // _recomputeCoordinateFramesStaticsModelWithNodalForces();

    const Geometry::TransformationMatrix T_tip = _computeTipTransform(_ot_rotation, _ot_translation, _it_rotation, _it_translation);

    Eigen::Matrix<Real,3,6> C;
    C << Mat3r::Zero(), Mat3r::Identity();

    // std::cout << "C:\n" << C << std::endl;

    Eigen::Matrix<Real,6,6> R;
    R << T_tip.rotMat(), Mat3r::Zero(), Mat3r::Zero(), T_tip.rotMat();

    // std::cout << "R:\n" << R << std::endl;

    // std::cout << "J_s analytical:\n " << _3DOFSpatialJacobian() << std::endl;
    // std::cout << "J_s numerical:\n " << _3DOFNumericalSpatialJacobian() << std::endl;

    Eigen::Matrix<Real,6,3> J_b = T_tip.inverse().adjoint() * _3DOFSpatialJacobian();
    // std::cout << "J_b:\n" << J_b << std::endl;
    Mat3r J_a = C * R * J_b;

    // std::cout << "J_a:\n" << J_a << std::endl;
    return J_a;
}

void VirtuosoArm::serialize(std::vector<std::byte>& buf) const
{
    Object::serialize(buf);
    pack(buf, _it_outer_dia);
    pack(buf, _ot_outer_dia);
    pack(buf, _ot_inner_dia);
    pack(buf, _it_inner_dia);
    pack(buf, _ot_r_curvature);
    pack(buf, _it_translation);
    pack(buf, _it_rotation);
    pack(buf, _ot_translation);
    pack(buf, _ot_rotation);
    pack(buf, _ot_distal_straight_length);
    pack(buf, _tool_state);
    pack(buf, _last_tool_state);
    pack(buf, _tool_type);
    pack(buf, _cutting_model);
    pack(buf, _cutting_model_time_threshold);
    pack(buf, _tool_manipulated_object);
    pack(buf, _commanded_tip_position);
    pack(buf, _grasped_vertices);
    pack(buf, _grasping_constraints);
    pack(buf, _collision_constraints);
    pack(buf, _arm_base_position);
    pack(buf, _arm_base_rotation);
    pack(buf, _tip_force);
    pack(buf, _tip_moment);
    pack(buf, _unfiltered_collision_force);
    pack(buf, _filtered_collision_force);
    pack(buf, _arm_base_frame);
    pack(buf, _ot_frames);
    pack(buf, _it_frames);
    pack(buf, _ot_nodal_forces);
    pack(buf, _it_nodal_forces);
    pack(buf, _stale_frames);
    pack(buf, _sdf);
}

void VirtuosoArm::deserialize(const std::byte*& buf)
{
    Object::deserialize(buf);
    unpack(buf, _it_outer_dia);
    unpack(buf, _ot_outer_dia);
    unpack(buf, _ot_inner_dia);
    unpack(buf, _it_inner_dia);
    unpack(buf, _ot_r_curvature);
    unpack(buf, _it_translation);
    unpack(buf, _it_rotation);
    unpack(buf, _ot_translation);
    unpack(buf, _ot_rotation);
    unpack(buf, _ot_distal_straight_length);
    unpack(buf, _tool_state);
    unpack(buf, _last_tool_state);
    unpack(buf, _tool_type);
    unpack(buf, _cutting_model);
    unpack(buf, _cutting_model_time_threshold);
    unpack(buf, _tool_manipulated_object);
    unpack(buf, _commanded_tip_position);
    unpack(buf, _grasped_vertices);
    unpack(buf, _grasping_constraints);
    unpack(buf, _collision_constraints);
    unpack(buf, _arm_base_position);
    unpack(buf, _arm_base_rotation);
    unpack(buf, _tip_force);
    unpack(buf, _tip_moment);
    unpack(buf, _unfiltered_collision_force);
    unpack(buf, _filtered_collision_force);
    unpack(buf, _arm_base_frame);
    unpack(buf, _ot_frames);
    unpack(buf, _it_frames);
    unpack(buf, _ot_nodal_forces);
    unpack(buf, _it_nodal_forces);
    unpack(buf, _stale_frames);
    unpack(buf, _sdf);
}

} // namespace Sim