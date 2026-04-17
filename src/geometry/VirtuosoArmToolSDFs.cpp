#include "geometry/VirtuosoArmToolSDFs.hpp"

#include "simobject/VirtuosoArmTools.hpp"

namespace Geometry
{

VirtuosoArmSpatulaToolSDF::VirtuosoArmSpatulaToolSDF(const Sim::VirtuosoArmSpatulaTool* spatula)
    : _spatula(spatula)
{

}

void VirtuosoArmSpatulaToolSDF::serialize(std::vector<std::byte>& buf) const
{
    pack(buf, _spatula);
}

void VirtuosoArmSpatulaToolSDF::deserialize(const std::byte*& buf)
{
    unpack(buf, _spatula);
}

Real VirtuosoArmSpatulaToolSDF::evaluate(const Vec3r& x) const
{
    Geometry::TransformationMatrix T_spat = _spatula->spatulaFrame().transform();
    Vec3r x_body = T_spat.rotMat().transpose() * (x - T_spat.translation());

    // 2D rounded rect SDF in XY plane
    Vec2r q;
    q[0] = std::abs(x_body[0]) - Sim::VirtuosoArmSpatulaTool::WIDTH/2 + Sim::VirtuosoArmSpatulaTool::RADIUS;
    q[1] = std::abs(x_body[2]) - Sim::VirtuosoArmSpatulaTool::LENGTH/2 + Sim::VirtuosoArmSpatulaTool::RADIUS;

    Vec2r q_max( std::max(q[0], Real(0.0)), std::max(q[1], Real(0.0)) );
    Real d2 = q_max.norm() + std::min(std::max(q[0], q[1]), Real(0.0)) - Sim::VirtuosoArmSpatulaTool::RADIUS;

    // Extrude along Z
    Vec2r w = Vec2r(d2, std::abs(x_body[1]) - Sim::VirtuosoArmSpatulaTool::THICKNESS/2);
    Vec2r w_max( std::max(w[0], Real(0.0)), std::max(w[1], Real(0.0)) );
    return w_max.norm() + std::min(std::max(w[0], w[1]), 0.0);
}

Vec3r VirtuosoArmSpatulaToolSDF::gradient(const Vec3r& x) const
{
    Geometry::TransformationMatrix T_spat = _spatula->spatulaFrame().transform();
    Vec3r x_body = T_spat.rotMat().transpose() * (x - T_spat.translation());

    Vec2r s( (x_body[0] > 0 ? 1 : -1), (x_body[2] > 0 ? 1 : -1) );
    Vec2r q;
    q[0] = std::abs(x_body[0]) - Sim::VirtuosoArmSpatulaTool::WIDTH/2 + Sim::VirtuosoArmSpatulaTool::RADIUS;
    q[1] = std::abs(x_body[2]) - Sim::VirtuosoArmSpatulaTool::LENGTH/2 + Sim::VirtuosoArmSpatulaTool::RADIUS; 

    Vec2r q_max( std::max(q[0], Real(0.0)), std::max(q[1], Real(0.0)) );

    Real lq = q_max.norm();
    Real d2 = lq + std::min(std::max(q[0], q[1]), Real(0.0)) - Sim::VirtuosoArmSpatulaTool::RADIUS;

    // 2D gradient of d2
    Vec2r g2;
    if (lq > 1e-6) {
        g2 = s.array() * q_max.array() / lq;
    } else {
        g2 = s.array() * (q[0] > q[1] ? Vec2r(1, 0) : Vec2r(0, 1)).array();
    }

    // Extrusion: treat (d2, |pz| - halfThickness) as a 2D point w
    Real sz = x_body[1] > 0 ? 1 : -1;
    Real wz = std::abs(x_body[1]) - Sim::VirtuosoArmSpatulaTool::THICKNESS/2;
    Vec2r w = Vec2r(d2, wz);
    Vec2r w_max( std::max(w[0], Real(0.0)), std::max(w[1], Real(0.0)) );
    Real lw = w_max.norm();

    Vec3r grad;
    if (lw > 1e-6) {
        // Outside in the extrusion sense — chain rule through both terms
        Real dOuterDd2 = w_max[0] / lw;
        Real dOuterDz  = w_max[1] / lw * sz;
        Vec2r g2_dOuterDd2 = g2 * dOuterDd2;
        grad = Vec3r(g2_dOuterDd2[0], g2_dOuterDd2[1], dOuterDz);
    } else {
        // Inside the extruded shell — gradient from max(w.x, w.y)
        if (w[0] > w[1]) {
            grad = Vec3r(g2[0], 0.0, g2[1]);        // d2 is the dominant term
        } else {
            grad = Vec3r(0.0, sz, 0.0);   // z-thickness is the dominant term
        }
    }

    return T_spat.rotMat() * grad;
}

} // namespace Geometry