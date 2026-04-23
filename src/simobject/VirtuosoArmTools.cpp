#include "simobject/VirtuosoArmTools.hpp"
#include "simobject/VirtuosoArm.hpp"

#include <numeric>

namespace Sim
{

std::pair<Vec3r, Vec3r> VirtuosoArmTool_Base::collisionTipForceAndMoment()
{
    const Geometry::CoordinateFrame& it_tip_frame = _arm->innerTubeEndFrame();
    Vec3r total_force = Vec3r::Zero();
    Vec3r total_moment = Vec3r::Zero();
    for (const auto& collision_proj_ref : _collision_proj_refs)
    {
        if (collision_proj_ref.exists() && collision_proj_ref.isValid())
        {
            // the collision constraints give forces on the tissue, so we must negate them to get the forces on the Virtuoso
            std::vector<Vec3r> forces = collision_proj_ref.constraintForces();
            Vec3r net_force = -std::reduce(forces.cbegin(), forces.cend());
            
            Vec3r cp = collision_proj_ref.constraint()->staticObjectContactPoint();
            total_force += net_force;
            total_moment += (cp - it_tip_frame.origin()).cross(net_force);
        }
    }

    return {total_force, total_moment};
    
}

void VirtuosoArmTool_Base::serialize(std::vector<std::byte>& buf) const
{
    Object::serialize(buf);
    pack(buf, _arm);
    pack(buf, _collision_proj_refs);
}
void VirtuosoArmTool_Base::deserialize(const std::byte*& buf)
{
    Object::deserialize(buf);
    unpack(buf, _arm);
    unpack(buf, _collision_proj_refs);
}


////////////////////////////////////////////////////////////////////////////////////

Geometry::CoordinateFrame VirtuosoArmSpatulaTool::spatulaFrame() const
{
    // spatula frame is along the z-direction
    Geometry::TransformationMatrix T(Mat3r::Identity(), Vec3r(0, 0, LENGTH/2));
    return _arm->innerTubeEndFrame()*T;
}

Geometry::CoordinateFrame VirtuosoArmSpatulaTool::tipFrame() const
{
    // spatula frame is along the z-direction
    Geometry::TransformationMatrix T(Mat3r::Identity(), tipOffset());
    return _arm->innerTubeEndFrame()*T;
}

void VirtuosoArmSpatulaTool::serialize(std::vector<std::byte>& buf) const
{
    VirtuosoArmTool_Base::serialize(buf);
    pack(buf, _sdf);
}
void VirtuosoArmSpatulaTool::deserialize(const std::byte*& buf)
{
    VirtuosoArmTool_Base::deserialize(buf);
    unpack(buf, _sdf);
}

///////////////////////////////////////////////////////////////////////////////////////

Geometry::CoordinateFrame VirtuosoArmCauteryTool::tipFrame() const
{
    Geometry::TransformationMatrix T(Mat3r::Identity(), tipOffset());
    return _arm->innerTubeEndFrame()*T;
}

Geometry::CoordinateFrame VirtuosoArmCauteryTool::ceramicFrame() const
{
    Geometry::TransformationMatrix T(Mat3r::Identity(), Vec3r(0,0,CERAMIC_LENGTH/2));
    return _arm->innerTubeEndFrame()*T;
}

Geometry::CoordinateFrame VirtuosoArmCauteryTool::wireFrame() const
{
    Geometry::TransformationMatrix T(Mat3r::Identity(), Vec3r(0,0,CERAMIC_LENGTH + WIRE_LENGTH/2));
    return _arm->innerTubeEndFrame()*T;
}

/** Add a XPBD->static collision constraint projector.
     * Used for getting collision force with deformable objects.
     */
void VirtuosoArmCauteryTool::addCollisionConstraint(Solver::ConstraintProjectorReferenceWrapper<Solver::StaticDeformableCollisionConstraint>&& collision_proj_ref)
{
    // contact point on the deformable object
    Vec3r def_cp = collision_proj_ref.constraint()->deformableObjectContactPoint();

    bool closest_to_wire = _sdf->closestToWire(def_cp);
    _closest_to_wire.push_back(closest_to_wire);

    // evaluate the cautery SDF, to see if it is closest to the "Wire" part of the tool
    _collision_proj_refs.push_back(std::move(collision_proj_ref));
}

/** Returns the axis-aligned bounding-box (AABB) for this Object in global simulation coordinates. */
Geometry::AABB VirtuosoArmCauteryTool::boundingBox() const
{
    Geometry::TransformationMatrix T = _arm->innerTubeEndFrame().transform();
    // Vec3r halfsize = T.rotMat().col(0) * CERAMIC_DIA/2 + T.rotMat().col(1) * CERAMIC_DIA/2 + T.rotMat().col(2) * CERAMIC_LENGTH/2;
    Vec3r base = T.translation();
    Vec3r tip = tipFrame().origin();

    Geometry::AABB bbox(std::min(base[0], tip[0]), std::min(base[1], tip[1]), std::min(base[2], tip[2]),
                        std::max(base[0], tip[0]), std::max(base[1], tip[1]), std::max(base[2], tip[2]) );
    return bbox;
}

void VirtuosoArmCauteryTool::serialize(std::vector<std::byte>& buf) const
{
    VirtuosoArmTool_Base::serialize(buf);
    pack(buf, _sdf);
    pack(buf, _closest_to_wire);
}
void VirtuosoArmCauteryTool::deserialize(const std::byte*& buf)
{
    VirtuosoArmTool_Base::deserialize(buf);
    unpack(buf, _sdf);
    unpack(buf, _closest_to_wire);
}

} // namespace SimObject