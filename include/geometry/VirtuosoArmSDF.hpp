#ifndef __VIRTUOSO_ARM_SDF_HPP
#define __VIRTUOSO_ARM_SDF_HPP

#include "geometry/SDF.hpp"
#include "geometry/TransformationMatrix.hpp"
// #include "simobject/VirtuosoArm.hpp"

namespace Sim
{
    class VirtuosoArm;
}

namespace Geometry
{

class VirtuosoArmSDF : public SDF
{
    public:
    /** Useful struct for returning SDF query info that includes information about where on the Virtuoso arm backbone
     * is closest to the query point. See evaluateWithGradientAndNodeInfo.
     */
    struct DistanceAndGradientWithNodeInfo
    {
        Real distance;
        Vec3r gradient;
        int node_index;
        Real interp_factor;
    };

    public:
    VirtuosoArmSDF() = default; // required for deserialization
    VirtuosoArmSDF(const Sim::VirtuosoArm* arm);

    void serialize(std::vector<std::byte>& buf) const;
    void deserialize(const std::byte*& buf);

    /** Evaluates F(x) for a Virtuoso Arm in its current state.
     * @param x - the point at which to evaluate the SDF
     * @returns the distance from x to the shape boundary ( F(x) )
    */
    virtual Real evaluate(const Vec3r& x) const override;

    /** Evaluates the gradient of F at x.
     * @param x - the point at which to evaluate the graient of the SDF
     * @returns the gradient of the SDF at x.
     */
    virtual Vec3r gradient(const Vec3r& x) const override;

    /** Evaluates the distance and gradient, and also returns the node index that corresponds to the closest segment, and an interpolation parameter
     * indicating how far along the closest segment the closest surface point is.
     * 
     * Special method for VirtuosoArmSDF that is helpful for applying collision forces to the appropriate point(s) along the tubes.
     * 
     * The node index is the global node index (i.e. between 0 and NUM_OT_FRAMES + NUM_IT_FRAMES - 1), and corresponds to the
     * node index of the beginning of the segment that is closest to x.
     * 
     * The interpolation parameter is between 0 and 1, and represents how far along the segment the closest surface point to the
     * queried point is. E.g. a value of 0 means that the closest surface point to x is on the oriented circle around the base of the closest segment,
     * and a value of 0.5 means the the closest surface point is "halfway" between the base and tip of the closest segment.
     * 
     * When only_tool = true, only the tool tube is considered. This is useful e.g. for mesh refinement around the cautery tool tip.
     * If there is no tool tube and only_tool = true, no collisions will be detected (a very large postiive distance will be returned).
     */
    DistanceAndGradientWithNodeInfo evaluateWithGradientAndNodeInfo(const Vec3r& x, bool only_tool=false) const;

    #ifdef HAVE_CUDA
    virtual void createGPUResource() override { assert(0); /* not implemented */ }
    #endif

    private:
    /** Computes the SDF distance of a capsule with endpoints a and b and radius radius, given a query point x. */
    Real _capsuleSDFDistance(const Vec3r& x, const Vec3r& a, const Vec3r& b, Real radius) const;
    
    /** Computes the SDF gradient of a capsule with endpoints a and b and radius radius, given a query point x. */
    Vec3r _capsuleSDFGradient(const Vec3r& x, const Vec3r& a, const Vec3r& b, Real radius) const;

    /** Computes the SDF distance, gradient, and interpolation factor along the capsule for a capsule with endpoints a and b and radius radius,
     * given a query point x.
     * 
     * The interpolation factor is a value between 0 and 1 that indicates which point on the line AB the query point is closest to. E.g. interpolation
     * factor being 0 means that the query point is closest to point A and 1 means that the query point is closest to point B
     */
    DistanceAndGradientWithNodeInfo _capsuleSDFDistanceGradientAndInterpFactor(const Vec3r& x, const Vec3r& a, const Vec3r& b, Real radius) const;

    private:
    const Sim::VirtuosoArm* _virtuoso_arm;
};

} // namespace Geometry


#endif // __VIRTUOSO_ARM_SDF_HPP