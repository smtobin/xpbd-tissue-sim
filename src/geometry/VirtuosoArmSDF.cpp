#include "geometry/VirtuosoArmSDF.hpp"

#include "simobject/VirtuosoArm.hpp"

namespace Geometry
{

VirtuosoArmSDF::VirtuosoArmSDF(const Sim::VirtuosoArm* arm)
    : _virtuoso_arm(arm)
{
}

// Real VirtuosoArmSDF::evaluate(const Vec3r& x) const
// {
//     // const Vec3r x_sdf = _globalToBodyInnerTube(x);
//     // const Real l = std::max(_virtuoso_arm->innerTubeTranslation() - _virtuoso_arm->outerTubeTranslation(), Real(0.0));
//     // return _cylinderSDFDistance(x_sdf, _virtuoso_arm->innerTubeOuterDiameter()/2.0, l);
//     const Sim::VirtuosoArm::InnerTubeFramesArray& it_frames = _virtuoso_arm->innerTubeFrames();
//     return _capsuleSDFDistance(x, it_frames.front().origin(), it_frames.back().origin(), 0.5*_virtuoso_arm->innerTubeOuterDiameter());
// }

// Vec3r VirtuosoArmSDF::gradient(const Vec3r& x) const
// {
//     // const Vec3r x_sdf = _globalToBodyInnerTube(x);
//     // const Real l = std::max(_virtuoso_arm->innerTubeTranslation() - _virtuoso_arm->outerTubeTranslation(), Real(0.0));
//     // const Vec3r body_grad = _cylinderSDFGradient(x_sdf, _virtuoso_arm->innerTubeOuterDiameter()/2.0, l);
//     // return _bodyToGlobalInnerTubeGradient(body_grad);

//     const Sim::VirtuosoArm::InnerTubeFramesArray& it_frames = _virtuoso_arm->innerTubeFrames();
//     return _capsuleSDFGradient(x, it_frames.front().origin(), it_frames.back().origin(), 0.5*_virtuoso_arm->innerTubeOuterDiameter());
// }

Real VirtuosoArmSDF::evaluate(const Vec3r& x) const
{
    return evaluateWithGradientAndNodeInfo(x).distance;
    // const Sim::VirtuosoArm::InnerTubeFramesArray& it_frames = _virtuoso_arm->innerTubeFrames();
    // return _capsuleSDFDistanceAndGradient(x, it_frames.front().origin(), it_frames.back().origin(), 0.5*_virtuoso_arm->innerTubeOuterDiameter()).first;
}

Vec3r VirtuosoArmSDF::gradient(const Vec3r& x) const
{
    return evaluateWithGradientAndNodeInfo(x).gradient;
    // const Sim::VirtuosoArm::InnerTubeFramesArray& it_frames = _virtuoso_arm->innerTubeFrames();
    // return _capsuleSDFDistanceAndGradient(x, it_frames.front().origin(), it_frames.back().origin(), 0.5*_virtuoso_arm->innerTubeOuterDiameter()).second;
}

VirtuosoArmSDF::DistanceAndGradientWithNodeInfo VirtuosoArmSDF::evaluateWithGradientAndNodeInfo(const Vec3r& x, bool only_tool) const
{
    const Sim::VirtuosoArm::OuterTubeFramesArray& ot_frames = _virtuoso_arm->outerTubeFrames();
    const Sim::VirtuosoArm::InnerTubeFramesArray& it_frames = _virtuoso_arm->innerTubeFrames();
    const Sim::VirtuosoArm::ToolTubeFramesArray& tt_frames = _virtuoso_arm->toolTubeFrames();

    DistanceAndGradientWithNodeInfo result;
    result.distance = std::numeric_limits<Real>::max();
    result.gradient = Vec3r::Zero();
    result.node_index = 0;
    result.interp_factor = Real(0.0);

    // iterate through tool tube segments (if the tool tube exists)
    if (_virtuoso_arm->hasTool())
    {
        Real tool_tube_radius = _virtuoso_arm->toolTube().outer_dia/2.0;
        for (unsigned i = 0; i < tt_frames.size()-1; i++)
        {
            const Vec3r& pos_i = tt_frames[i].origin();
            const Vec3r& pos_iplus1 = tt_frames[i+1].origin();

            
            auto capsule_result = _capsuleSDFDistanceGradientAndInterpFactor(x, pos_i, pos_iplus1, tool_tube_radius);
            
            
            if (capsule_result.distance < result.distance)
            {
                result.distance = capsule_result.distance;
                result.gradient = capsule_result.gradient;
                result.node_index = ot_frames.size() + it_frames.size() + i;
                result.interp_factor = capsule_result.interp_factor;
            }

            if (result.distance < 0)
                return result;
        }
    }

    if (only_tool)
        return result;
    

    // iterate through inner tube segments
    for (unsigned i = 0; i < it_frames.size()-1; i++)
    {
        const Vec3r& pos_i = it_frames[i].origin();
        const Vec3r& pos_iplus1 = it_frames[i+1].origin();

        auto capsule_result = _capsuleSDFDistanceGradientAndInterpFactor(x, pos_i, pos_iplus1, 0.5*_virtuoso_arm->innerTubeOuterDiameter());
        
        
        if (capsule_result.distance < result.distance)
        {
            result.distance = capsule_result.distance;
            result.gradient = capsule_result.gradient;
            result.node_index = ot_frames.size() + i;
            result.interp_factor = capsule_result.interp_factor;
        }

        if (result.distance < 0)
            return result;
            
    }

    // iterate through outer tube segments
    for (unsigned i = 0; i < ot_frames.size()-1; i++)
    {
        const Vec3r& pos_i = ot_frames[i].origin();
        const Vec3r& pos_iplus1 = ot_frames[i+1].origin();

        auto capsule_result = _capsuleSDFDistanceGradientAndInterpFactor(x, pos_i, pos_iplus1, 0.5*_virtuoso_arm->outerTubeOuterDiameter());

        if (capsule_result.distance < result.distance)
        {
            result.distance = capsule_result.distance;
            result.gradient = capsule_result.gradient;
            result.node_index = i;
            result.interp_factor = capsule_result.interp_factor;
        }

        if (result.distance < 0)
            return result;
            
    }

    return result;
}

// taken from https://iquilezles.org/articles/distfunctions/
Real VirtuosoArmSDF::_capsuleSDFDistance(const Vec3r& p, const Vec3r& a, const Vec3r& b, Real radius) const
{
    const Vec3r pa = p - a;
    const Vec3r ba = b - a;
    const Real h = std::clamp(pa.dot(ba) / ba.squaredNorm(), Real(0.0), Real(1.0) );
    return (pa - ba*h).norm() - radius;
}

// derived from the distance function above
Vec3r VirtuosoArmSDF::_capsuleSDFGradient(const Vec3r& p, const Vec3r& a, const Vec3r& b, Real radius) const
{
    const Vec3r pa = p - a;
    const Vec3r ba = b - a;
    const Real h = std::clamp(pa.dot(ba) / ba.squaredNorm(), Real(0.0), Real(1.0) );
    return (pa - ba*h).normalized();
}

VirtuosoArmSDF::DistanceAndGradientWithNodeInfo VirtuosoArmSDF::_capsuleSDFDistanceGradientAndInterpFactor(const Vec3r& p, const Vec3r& a, const Vec3r& b, Real radius) const
{
    const Vec3r pa = p - a;
    const Vec3r ba = b - a;
    const Real h = std::clamp(pa.dot(ba) / ba.squaredNorm(), Real(0.0), Real(1.0) );
    const Vec3r vec = pa - ba*h;
    const Real vec_norm = vec.norm();

    DistanceAndGradientWithNodeInfo result;
    result.distance = vec_norm - radius;
    result.gradient = vec/vec_norm;
    result.node_index = -1; // fill it out but this shouldn't be used
    result.interp_factor = h;
    return result;
}

void VirtuosoArmSDF::serialize(std::vector<std::byte>& buf) const
{
    pack(buf, _virtuoso_arm);
}

void VirtuosoArmSDF::deserialize(const std::byte*& buf)
{
    unpack(buf, _virtuoso_arm);
}

} // namespace Geometry