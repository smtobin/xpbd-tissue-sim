#ifndef __MESH_SDF_HPP
#define __MESH_SDF_HPP

// define MESH2SDF_DOUBLE_PRECISION so the Mesh2SDF library compiles with Real precision
#ifndef HAVE_CUDA
#define MESH2SDF_DOUBLE_PRECISION
#endif

#include <Mesh2SDF/MeshSDF.hpp>

#include "geometry/SDF.hpp"
#include "geometry/AABB.hpp"
#include "config/simobject/RigidMeshObjectConfig.hpp"

namespace Sim
{
   class RigidMeshObject;
}

namespace Geometry
{

class MeshSDF : public SDF
{
    public:
    MeshSDF(const Sim::RigidMeshObject* mesh_obj, const Config::RigidMeshObjectConfig* config);

    virtual void serialize(std::vector<std::byte>& buf) const;
    virtual void deserialize(const std::byte*& buf);

    virtual Real evaluate(const Vec3r& x) const override;

    virtual Vec3r gradient(const Vec3r& x) const override;

    const Vec3r& gridCellSize() const { return _sdf.gridCellSize(); }
    AABB gridBoundingBox() const { const mesh2sdf::BoundingBox bbox = _sdf.gridBoundingBox(); return AABB(bbox.first, bbox.second); }
    const mesh2sdf::Array3<Real>& distanceGrid() const { return _sdf.distanceGrid(); }
    const mesh2sdf::Array3<Vec3r>& gradientGrid() const { return _sdf.gradientGrid(); }

    const Sim::RigidMeshObject* meshObj() const { return _mesh_obj; }
 #ifdef HAVE_CUDA
    virtual void createGPUResource() override;
 #endif

    protected:
    const Sim::RigidMeshObject* _mesh_obj;
    mesh2sdf::MeshSDF _sdf;
    bool _from_file;

};

} // namespace Geometry

#endif // __MESH_SDF_HPP