#ifndef __VIRTUOSO_ARM_TOOL_SDFS_HPP
#define __VIRTUOSO_ARM_TOOL_SDFS_HPP

#include "geometry/SDF.hpp"

namespace Sim
{
    class VirtuosoArmSpatulaTool;
}

namespace Geometry
{

class VirtuosoArmSpatulaToolSDF : public SDF
{
public:
    VirtuosoArmSpatulaToolSDF() = default;

    VirtuosoArmSpatulaToolSDF(const Sim::VirtuosoArmSpatulaTool* spatula);

    virtual void serialize(std::vector<std::byte>& buf) const;
    virtual void deserialize(const std::byte*& buf);

    /** Evaluates F(x) for a box with arbitrary position and orientation and size
     * @param x - the point at which to evaluate the SDF
     * @returns the distance from x to the shape boundary ( F(x) )
     */
    virtual Real evaluate(const Vec3r& x) const override;

    /** Evaluates the gradient of F at x.
     * @param x - the point at which to evaluate the graient of the SDF
     * @returns the gradient of the SDF at x.
     */
    virtual Vec3r gradient(const Vec3r& x) const override;

    const Sim::VirtuosoArmSpatulaTool* spatula() const { return _spatula; }

protected:
    /** Pointer to box needed for box's current position, orientation and size */
    const Sim::VirtuosoArmSpatulaTool* _spatula;

};


} // namespace Geometry

#endif // __VIRTUOSO_ARM_TOOL_SDFS_HPP