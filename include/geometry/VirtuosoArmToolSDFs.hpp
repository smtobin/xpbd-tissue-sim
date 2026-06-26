#ifndef __VIRTUOSO_ARM_TOOL_SDFS_HPP
#define __VIRTUOSO_ARM_TOOL_SDFS_HPP

#include "geometry/SDF.hpp"

namespace Sim
{
    class VirtuosoArmSpatulaTool;
    class VirtuosoArmCauteryTool;
    class VirtuosoArmGraspingTool;
}

namespace Geometry
{

class VirtuosoArmToolSDF : public SDF
{
public:
    /** Given another SDF, finds the closest point on the tool geometry.
     * Then, this point can be evaluated against the SDF to determine if there is a collision or not.
     * Useful for colliding tools against one another and with rigid meshes represented by SDFs.
     */
    virtual Vec3r findContactPoint(const SDF* sdf) const = 0;

protected:
    Vec3r _iterativeClosestPointProjection(const SDF* sdf, const Vec3r& start_global, int num_iter=3) const;
};

class VirtuosoArmSpatulaToolSDF : public VirtuosoArmToolSDF
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

    virtual Vec3r findContactPoint(const SDF* sdf) const override;

    const Sim::VirtuosoArmSpatulaTool* spatula() const { return _spatula; }

protected:
    /** Pointer to box needed for box's current position, orientation and size */
    const Sim::VirtuosoArmSpatulaTool* _spatula;

};


/////////////////////////////////////////////////////////////////////////////////////

class VirtuosoArmCauteryToolSDF : public VirtuosoArmToolSDF
{
public:
    VirtuosoArmCauteryToolSDF() = default;

    VirtuosoArmCauteryToolSDF(const Sim::VirtuosoArmCauteryTool* cautery);

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

    virtual Vec3r findContactPoint(const SDF* sdf) const override;

    /** Returns true if the queried point is closest to the "wire" part of the cautery tool.
     * Returns false otherwise (i.e. when it is closest to the cermaic part).
     */
    bool closestToWire(const Vec3r& x) const;

    const Sim::VirtuosoArmCauteryTool* cautery() const { return _cautery; }

protected:
    /** Pointer to box needed for box's current position, orientation and size */
    const Sim::VirtuosoArmCauteryTool* _cautery;

};

/////////////////////////////////////////////////////////////////////////////////////

class VirtuosoArmGraspingToolSDF : public VirtuosoArmToolSDF
{
public:
    VirtuosoArmGraspingToolSDF() = default;

    VirtuosoArmGraspingToolSDF(const Sim::VirtuosoArmGraspingTool* grasper);

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

    virtual Vec3r findContactPoint(const SDF* sdf) const override;

    const Sim::VirtuosoArmGraspingTool* grasper() const { return _grasper; }

protected:
    /** Pointer to box needed for box's current position, orientation and size */
    const Sim::VirtuosoArmGraspingTool* _grasper;

};


} // namespace Geometry

#endif // __VIRTUOSO_ARM_TOOL_SDFS_HPP