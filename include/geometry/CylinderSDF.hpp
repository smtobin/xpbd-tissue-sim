#ifndef __CYLINDER_SDF_HPP
#define __CYLINDER_SDF_HPP

#include "geometry/SDF.hpp"

namespace Sim
{
   class RigidCylinder;
}

namespace Geometry
{

/** Implements a signed distance function for a rigid cylinder.
 * Assumes the cylinder in its un-transformed configuration is a vertical cylinder centered about the origin, with its height in the z-direction.
 */
class CylinderSDF : public SDF
{
    public:
    CylinderSDF() = default;

    CylinderSDF(const Sim::RigidCylinder* cyl);

    virtual void serialize(std::vector<std::byte>& buf) const;
    virtual void deserialize(const std::byte*& buf);

    /** Evaluates F(x) for a cylinder with arbitrary position, orientation, radius and height
     * @param x - the point at which to evaluate the SDF
     * @returns the distance from x to the shape boundary ( F(x) )
    */
    virtual Real evaluate(const Vec3r& x) const override;

    /** Evaluates the gradient of F at x.
     * @param x - the point at which to evaluate the graient of the SDF
     * @returns the gradient of the SDF at x.
     */
    virtual Vec3r gradient(const Vec3r& x) const override;

    const Sim::RigidCylinder* cylinder() const { return _cyl; }

 #ifdef HAVE_CUDA
    virtual void createGPUResource() override;
 #endif

    protected:
    /** Pointer to cylinder for the cylinder's current position, orientation, radius and height. */
    const Sim::RigidCylinder* _cyl;
    

};

} // namespace Geometry

#endif // __CYLINDER_SDF_HPP