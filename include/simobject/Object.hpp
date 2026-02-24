#ifndef __OBJECT_HPP
#define __OBJECT_HPP

#include <string>

#include "common/types.hpp"

#include "geometry/AABB.hpp"
#include "geometry/SDF.hpp"
#include "config/simobject/ObjectConfig.hpp"
#include "simobject/MaterialClass.hpp"

#ifdef HAVE_CUDA
#include "gpu/resource/GPUResource.hpp"
#endif

namespace Sim
{

class Simulation;

class Object
{
    // public typedefs
    public:
    using ConfigType = Config::ObjectConfig;

    public:
    Object(const Simulation* sim, const ConfigType* config);

    virtual ~Object() = default;

    virtual void serialize(std::vector<std::byte>& buf) const;
    virtual void deserialize(const std::byte*& buf);

    /** Returns a string with all relevant information about this object. 
     * @param indent : the level of indentation to use for formatting new lines of the string
    */
    virtual std::string toString(const int indent) const
    {
        return std::string(indent, '\t') + "Name: " + _name;
    }
    
    /** Returns a string with the type of the object. */
    virtual std::string type() const { return "Object"; }

    /** Returns the name of this object. */
    std::string name() const { return _name; }

    /** The simulation that this object belongs to. */
    const Simulation* sim() const { return _sim; }

    /** The "material" that this object is made out of. */
    const MaterialClass* materialClass() const { return _material_class; }

    /** Returns the characteristic dimension of the object. */
    Real characteristicDimension() const { return _char_dim; }

    /** Set the characteristic dimension of the SDF.
     * This is only necessary if the object has some special known structure that needs to be accounted for during collision detection.
     * E.g. a mesh with small features
     */
    void setCharacteristicDimension(Real new_char_dim) { _char_dim = new_char_dim; }

    /** Performs any necessary setup for this object.
     * Called after instantiation (i.e. outside the constructor) and before update() is called for the first time.
     */
    virtual void setup() = 0;

    /** Evolves this object one time step forward in time. 
     * Completely up to the derived classes to decide how they should step forward in time.
    */
    virtual void update() = 0;

    /** Update the velocity of this object */
    virtual void velocityUpdate() = 0;
    
    /** Returns the axis-aligned bounding-box (AABB) for this Object in global simulation coordinates. */
    virtual Geometry::AABB boundingBox() const = 0;

    virtual void createSDF() = 0;
    virtual const Geometry::SDF* SDF() const = 0;

 #ifdef HAVE_CUDA
    virtual void createGPUResource() = 0;
    virtual const HostReadableGPUResource* gpuResource() const { assert(_gpu_resource); return _gpu_resource.get(); }
    virtual HostReadableGPUResource* gpuResource() { assert(_gpu_resource); return _gpu_resource.get(); }
 #endif

    protected:
    /** Name of the object */
    std::string _name;

    /** The characteristic dimension of the object. Used to determine the sample density during collision detection.
     * I.e. for an object with a smaller characteristic dimension, the faces will need to be more highly subsampled to prevent tunneling
     * 
     * For example, the characteristic dimension of a sphere is its diameter, and the characteristic dimension of a box is its smallest side length.
     * 
     * This can be set explicitly for objects with some known structure (e.g. a spiky mesh).
     * 
     */
    Real _char_dim = 1;

    /** The material class associated with the object. */
    const MaterialClass* _material_class;

    /** Pointer to the Simulation object that created this Object.
     * Useful for querying things like current sim time, sim time step, or acceleration due to gravity.
    */
    const Simulation* _sim;

#ifdef HAVE_CUDA
    std::unique_ptr<HostReadableGPUResource> _gpu_resource;
#endif
};

} // namespace Simulation

#endif // __OBJECT_HPP