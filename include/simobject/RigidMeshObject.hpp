#ifndef __RIGID_MESH_OBJECT_HPP
#define __RIGID_MESH_OBJECT_HPP

#include "simobject/RigidObject.hpp"
#include "simobject/MeshObject.hpp"

#include "geometry/Mesh.hpp"
#include "geometry/MeshSDF.hpp"

#include "config/simobject/RigidMeshObjectConfig.hpp"

namespace Sim
{

class RigidMeshObject : public RigidObject, public MeshObject
{
    // public typedefs
    public:
    using SDFType = Geometry::MeshSDF;
    using ConfigType = Config::RigidMeshObjectConfig;

    public:
    RigidMeshObject(const Simulation* sim, const ConfigType* config);

    virtual void serialize(std::vector<std::byte>& buf) const override;
    virtual void deserialize(const std::byte*& buf) override;

    virtual std::string type() const override { return "RigidMeshObject"; }

    virtual std::string toString(const int indent) const override;

    virtual Geometry::AABB boundingBox() const override;

    virtual void setup() override;

    virtual void update() override;

    virtual void setPosition(const Vec3r& position) override;

    virtual void setOrientation(const Vec4r& orientation) override;

    void rotateAboutOrigin(const Mat3r& rot_mat);

    /** TODO: propogate the config object somehow so that we can pass on the SDF filename */
    virtual void createSDF() override 
    { 
        if(!_sdf.has_value()) 
            _sdf = SDFType(this, nullptr); 
    }

    virtual const SDFType* SDF() const override { return _sdf.has_value() ? &_sdf.value() : nullptr; }

 #ifdef HAVE_CUDA
    virtual void createGPUResource() override { assert(0); /* not implemented */ }
 #endif

    protected:
    Real _density;
    std::unique_ptr<Geometry::Mesh> _initial_mesh;

    /** Signed Distance Field for the mesh. Must be created explicitly with createSDF(). */
    std::optional<SDFType> _sdf;
};

} // namespace Simulation

#endif // __RIGID_MESH_OBJECT