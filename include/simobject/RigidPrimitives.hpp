#ifndef __RIGID_PRIMITIVES_HPP
#define __RIGID_PRIMITIVES_HPP

#include "simobject/RigidObject.hpp"

#include "geometry/SphereSDF.hpp"
#include "geometry/BoxSDF.hpp"
#include "geometry/CylinderSDF.hpp"

#include "config/simobject/RigidPrimitiveConfigs.hpp"

namespace Sim
{

class RigidSphere : public RigidObject
{
    // public typedefs
    public:
    using SDFType = Geometry::SphereSDF;
    using ConfigType = Config::RigidSphereConfig;

    public:
    RigidSphere(const Simulation* sim, const ConfigType* config);

    virtual void serialize(std::vector<std::byte>& buf) const override;
    virtual void deserialize(const std::byte*& buf) override;

    /** Returns a string with all relevant information about this object. 
     * @param indent : the level of indentation to use for formatting new lines of the string
    */
    virtual std::string toString(const int indent) const override;
    
    /** Returns a string with the type of the object. */
    virtual std::string type() const override { return "RigidObject"; }

    void setRadius(Real new_radius) { _radius = new_radius; }
    Real radius() const { return _radius; }

    virtual void setup() override;

    virtual Geometry::AABB boundingBox() const override;

    virtual void createSDF() override 
    { 
        if (!_sdf.has_value()) 
            _sdf = SDFType(this); 
    }
    virtual const SDFType* SDF() const override { return _sdf.has_value() ? &_sdf.value() : nullptr; };

 #ifdef HAVE_CUDA
    virtual void createGPUResource() override { assert(0); /* not implemented */ }
 #endif

    protected:
    /** Radius of the sphere */
    Real _radius;

    /** Signed Distance Field for the sphere. Must be created explicitly with createSDF(). */
    std::optional<SDFType> _sdf;

};


/////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////

class RigidBox : public RigidObject
{
    // public typedefs
    public:
    using SDFType = Geometry::BoxSDF;
    using ConfigType = Config::RigidBoxConfig;

    public:
    RigidBox(const Simulation* sim, const ConfigType* config);

    virtual void serialize(std::vector<std::byte>& buf) const override;
    virtual void deserialize(const std::byte*& buf) override;

    /** Returns a string with all relevant information about this object. 
     * @param indent : the level of indentation to use for formatting new lines of the string
    */
    virtual std::string toString(const int indent) const override;
    
    /** Returns a string with the type of the object. */
    virtual std::string type() const override { return "RigidObject"; }

    Vec3r size() const { return _size; }

    virtual void setup() override;

    virtual Geometry::AABB boundingBox() const override;

    virtual void createSDF() override 
    { 
        if (!_sdf.has_value()) 
            _sdf = SDFType(this); 
    }
    virtual const SDFType* SDF() const override { return _sdf.has_value() ? &_sdf.value() : nullptr; }

 #ifdef HAVE_CUDA
    virtual void createGPUResource() override { assert(0); /* not implemented */ }
 #endif  

    protected:
    /** 3D size of the box */
    Vec3r _size;
    /** Original bounding box points (these will just get transformed along with the box points) */
    Eigen::Matrix<Real, 3, 8> _origin_bbox_points;

    /** Signed Distance Field for the box. Must be created explicitly with createSDF(). */
    std::optional<SDFType> _sdf;
};

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////

class RigidCylinder : public RigidObject
{
    // public typedefs
    public:
    using SDFType = Geometry::CylinderSDF;
    using ConfigType = Config::RigidCylinderConfig;

    public:
    RigidCylinder(const Simulation* sim, const ConfigType* config);

    virtual void serialize(std::vector<std::byte>& buf) const override;
    virtual void deserialize(const std::byte*& buf) override;

    /** Returns a string with all relevant information about this object. 
     * @param indent : the level of indentation to use for formatting new lines of the string
    */
    virtual std::string toString(const int indent) const override;
    
    /** Returns a string with the type of the object. */
    virtual std::string type() const override { return "RigidObject"; }

    Real radius() const { return _radius; }

    Real height() const { return _height; }

    virtual void setup() override;

    virtual Geometry::AABB boundingBox() const override;

    virtual void createSDF() override 
    { 
        if (!_sdf.has_value()) 
            _sdf = SDFType(this); 
    }

    virtual const SDFType* SDF() const override { return _sdf.has_value() ? &_sdf.value() : nullptr; }

 #ifdef HAVE_CUDA
    virtual void createGPUResource() override { assert(0); /* not implemented */ }
 #endif

    protected:
    /** Radius of the cylinder */
    Real _radius;
    /** Height of the cylinder */
    Real _height;
    /** Oiriginal bounding box points (these will just get transformed along with the cylinder). */
    Eigen::Matrix<Real, 3, 8> _origin_bbox_points;

    /** Signed Distance Field for the cylinder. Must be created explicitly with createSDF(). */
    std::optional<SDFType> _sdf;
};

} //namespace Sim

#endif // __RIGID_PRIMITIVES_HPP