#ifndef __BOX_SDF_HPP
#define __BOX_SDF_HPP

#include "geometry/SDF.hpp"

namespace Sim
{
    class RigidBox;
}

namespace Geometry
{

/** Implements a signed distance function for a rigid box. */
class BoxSDF : public SDF
{
    public:
    BoxSDF() = default;

    BoxSDF(const Sim::RigidBox* box);

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

    const Sim::RigidBox* box() const { return _box; }

 #ifdef HAVE_CUDA
    virtual void createGPUResource() override;
 #endif

    protected:
    /** Pointer to box needed for box's current position, orientation and size */
    const Sim::RigidBox* _box;

};

} // namespace Geometry

#endif // __BOX_SDF_HPP