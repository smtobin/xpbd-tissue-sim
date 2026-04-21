#include "geometry/VirtuosoArmToolSDFs.hpp"

#include "simobject/VirtuosoArmTools.hpp"

namespace Geometry
{

Vec3r VirtuosoArmToolSDF::_iterativeClosestPointProjection(const SDF* sdf, const Vec3r& start_global, int num_iter) const
{
    Vec3r p = start_global;
    for (int i = 0; i < num_iter; i++)
    {
        Real d1 = sdf->evaluate(p);
        Vec3r n1 = sdf->gradient(p);
        // project towards other object surface
        p -= d1*n1;
        // project back to this SDF
        Real d2 = evaluate(p);
        Vec3r n2 = gradient(p);
        p -= d2*n2;
    }
    return p;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////

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
    Real k = Sim::VirtuosoArmSpatulaTool::THICKNESS/4;

    Real a = d2;
    Real b = w[1];

    // Smooth max blend - fillets the hard corners for collision stability
    Real h = std::clamp(0.5 + 0.5*(b - a)/k, Real(0), Real(1));
    Real d = a*(1 - h) + b*h + k*h*(1 - h);

    return d;
}

Vec3r VirtuosoArmSpatulaToolSDF::gradient(const Vec3r& x) const
{
    Geometry::TransformationMatrix T = _spatula->spatulaFrame().transform();

    // transform to body frame
    Vec3r p = T.rotMat().transpose() * (x - T.translation());

    // signs
    Real sx = (p[0] >= 0 ? 1.0 : -1.0);
    Real sy = (p[1] >= 0 ? 1.0 : -1.0);
    Real sz = (p[2] >= 0 ? 1.0 : -1.0);

    // side SDF (XZ plane rounded rectangle)
    Vec2r q;
    q[0] = std::abs(p[0]) - Sim::VirtuosoArmSpatulaTool::WIDTH/2 + Sim::VirtuosoArmSpatulaTool::RADIUS;
    q[1] = std::abs(p[2]) - Sim::VirtuosoArmSpatulaTool::LENGTH/2 + Sim::VirtuosoArmSpatulaTool::RADIUS;

    Vec2r qmax(std::max(q[0], 0.0),
               std::max(q[1], 0.0));

    Real lq = qmax.norm();

    Real a = lq + std::min(std::max(q[0], q[1]), 0.0) - Sim::VirtuosoArmSpatulaTool::RADIUS; // d2

    // gradient of a (2D rounded rectangle)
    Vec2r ga;
    if (lq > 1e-8)
    {
        ga = Vec2r(sx * qmax[0] / lq,
                   sz * qmax[1] / lq);
    }
    else
    {
        ga = (q[0] > q[1]) ? Vec2r(sx, 0.0)
                            : Vec2r(0.0, sz);
    }

    Vec3r grad_a(ga[0], 0.0, ga[1]);

    // cap SDF (Y axis thickness)
    Real b = std::abs(p[1]) - Sim::VirtuosoArmSpatulaTool::THICKNESS/2;

    Vec3r grad_b(0.0, sy, 0.0);

    // smooth max
    Real k = Sim::VirtuosoArmSpatulaTool::THICKNESS/4;

    Real t = 0.5 + 0.5 * (b - a) / k;

    // clamp regions
    if (t <= 0.0)
        return T.rotMat() * grad_a;

    if (t >= 1.0)
        return T.rotMat() * grad_b;

    Real h = t;

    Real coeff = (b - a) / (2.0 * k);

    Real da = (1.0 - h) - coeff * (1.0 - 2.0 * h);
    Real db = h         + coeff * (1.0 - 2.0 * h);

    Vec3r grad = da * grad_a + db * grad_b;

    return T.rotMat() * grad;
}

Vec3r VirtuosoArmSpatulaToolSDF::findContactPoint(const SDF* sdf) const
{
    return _iterativeClosestPointProjection(sdf, _spatula->spatulaFrame().origin());
}

/////////////////////////////////////////////////////////////////////////////////////////

VirtuosoArmCauteryToolSDF::VirtuosoArmCauteryToolSDF(const Sim::VirtuosoArmCauteryTool* cautery)
    : _cautery(cautery)
{
}

void VirtuosoArmCauteryToolSDF::serialize(std::vector<std::byte>& buf) const
{
    pack(buf, _cautery);
}

void VirtuosoArmCauteryToolSDF::deserialize(const std::byte*& buf)
{
    unpack(buf, _cautery);
}

Real VirtuosoArmCauteryToolSDF::evaluate(const Vec3r& x) const
{
    Geometry::TransformationMatrix it_tip_transform = _cautery->innerTubeTipFramePtr()->transform();
    Vec3r x_body = it_tip_transform.rotMat().transpose() * (x - it_tip_transform.translation());

    // dist to ceramic part
    Real ceramic_z = std::clamp(x_body[2], Real(0.0), Sim::VirtuosoArmCauteryTool::CERAMIC_LENGTH);
    Real ceramic_dist = (x_body - Vec3r(0,0,ceramic_z)).norm() - Sim::VirtuosoArmCauteryTool::CERAMIC_DIA/2;

    // dist to wire part
    Real wire_z = std::clamp(x_body[2], Sim::VirtuosoArmCauteryTool::CERAMIC_LENGTH, Sim::VirtuosoArmCauteryTool::CERAMIC_LENGTH+Sim::VirtuosoArmCauteryTool::WIRE_LENGTH);
    Real wire_dist = (x_body - Vec3r(0,0,wire_z)).norm() - Sim::VirtuosoArmCauteryTool::WIRE_DIA/2;

    return std::min(ceramic_dist, wire_dist);
}

Vec3r VirtuosoArmCauteryToolSDF::gradient(const Vec3r& x) const
{
    Geometry::TransformationMatrix it_tip_transform = _cautery->innerTubeTipFramePtr()->transform();
    Vec3r x_body = it_tip_transform.rotMat().transpose() * (x - it_tip_transform.translation());

    // dist to ceramic part
    Real ceramic_z = std::clamp(x_body[2], Real(0.0), Sim::VirtuosoArmCauteryTool::CERAMIC_LENGTH);
    Vec3r cp_ceramic = Vec3r(0,0,ceramic_z);
    Real ceramic_dist = (x_body - cp_ceramic).norm();

    // dist to wire part
    Real wire_z = std::clamp(x_body[2], Sim::VirtuosoArmCauteryTool::CERAMIC_LENGTH, Sim::VirtuosoArmCauteryTool::CERAMIC_LENGTH+Sim::VirtuosoArmCauteryTool::WIRE_LENGTH);
    Vec3r cp_wire = Vec3r(0,0,wire_z);
    Real wire_dist = (x_body - Vec3r(0,0,wire_z)).norm();

    Vec3r grad;
    if (ceramic_dist < wire_dist)
    {
        if (ceramic_dist < 1e-6)
            grad = it_tip_transform.rotMat().col(0);
        else
            grad = (x_body - cp_ceramic) / ceramic_dist;
    }
    else
    {
        if (wire_dist < 1e-6)
            grad = it_tip_transform.rotMat().col(0);
        else
            grad = (x_body - cp_wire) / wire_dist;
    }

    return it_tip_transform.rotMat() * grad;
}

Vec3r VirtuosoArmCauteryToolSDF::findContactPoint(const SDF* sdf) const
{
    // sample a few points along the length of the cautery tool to find the best starting point
    Real ceramic_d = sdf->evaluate(_cautery->ceramicFrame().origin());
    Real wire_d = sdf->evaluate(_cautery->wireFrame().origin());
    Real wire_tip_d = sdf->evaluate(_cautery->tipFrame().origin());

    if (ceramic_d < wire_d && ceramic_d < wire_tip_d)
        return _iterativeClosestPointProjection(sdf, _cautery->ceramicFrame().origin());
    else if (wire_d < ceramic_d && wire_d < wire_tip_d)
        return _iterativeClosestPointProjection(sdf, _cautery->wireFrame().origin());
    else
        return _iterativeClosestPointProjection(sdf, _cautery->tipFrame().origin());
}

} // namespace Geometry