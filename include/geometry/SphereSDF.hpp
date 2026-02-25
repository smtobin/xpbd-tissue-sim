#ifndef __SPHERE_SDF_HPP
#define __SPHERE_SDF_HPP

#include "geometry/SDF.hpp"

namespace Sim
{
   class RigidSphere;
}

namespace Geometry
{

/** Implements a signed distance function for a rigid sphere. */
class SphereSDF : public SDF
{
    public:
    SphereSDF() = default;
    
    SphereSDF(const Sim::RigidSphere* sphere);

    virtual void serialize(std::vector<std::byte>& buf) const;
    virtual void deserialize(const std::byte*& buf);

    /** Evaluates F(x) for a sphere.
     * @param x - the point at which to evaluate the SDF
     * @returns the distance from x to the shape boundary ( F(x) )
    */
    virtual Real evaluate(const Vec3r& x) const override;

    /** Evaluates the gradient of F at x.
     * @param x - the point at which to evaluate the graient of the SDF
     * @returns the gradient of the SDF at x.
     */
    virtual Vec3r gradient(const Vec3r& x) const override;

    const Sim::RigidSphere* sphere() const { return _sphere; }

 #ifdef HAVE_CUDA
    virtual void createGPUResource() override;
 #endif

    protected:
    /** Pointer to sphere needed for sphere's current position and radius. */
    const Sim::RigidSphere* _sphere;
};

} // namespace Geometry

#endif // __SPHERE_SDF_HPP