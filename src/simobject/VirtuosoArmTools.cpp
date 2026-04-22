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

} // namespace SimObject