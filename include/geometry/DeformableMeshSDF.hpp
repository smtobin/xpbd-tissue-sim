#ifndef __DEFORMABLE_SDF_HPP
#define __DEFORMABLE_SDF_HPP

// define MESH2SDF_DOUBLE_PRECISION so the Mesh2SDF library compiles with same precision as Real datatype
#ifndef HAVE_CUDA
#define MESH2SDF_DOUBLE_PRECISION
#endif

#include <Mesh2SDF/MeshSDF.hpp>

#include "geometry/SDF.hpp"
#include "geometry/embree/EmbreeScene.hpp"

#include "simobject/MeshObject.hpp"

namespace Geometry
{

class DeformableMeshSDF : public SDF
{
    public:
    DeformableMeshSDF(const Sim::TetMeshObject* mesh_obj, const EmbreeScene* embree_scene);

    // DeformableMeshSDF(const DeformableMeshSDF& other);

    // DeformableMeshSDF(DeformableMeshSDF&& other);

    // Explicitly define move operations if they're missing
    DeformableMeshSDF(DeformableMeshSDF&&) = default;
    DeformableMeshSDF& operator=(DeformableMeshSDF&&) = default;

    virtual Real evaluate(const Vec3r& x) const override;
    virtual Vec3r gradient(const Vec3r& x) const override;

    int closestSurfaceFaceToPointInTet(const Vec3r& x, int tet_index) const;

    virtual std::pair<int, Vec3r> closestSurfacePoint(const Vec3r& x) const;

    private:
    // Real _evaluateStaticSDF(const Vec3r& X_m) const;
    // Vec3r _gradientStaticSDF(const Vec3r& X_m) const;

    const Sim::TetMeshObject* _mesh_obj;
    const Geometry::Mesh::vertices_vec_type _initial_vertices;
    // mesh2sdf::MeshSDF _sdf;
    // bool _from_file;

    /** For making BVH point queries. We assume that the object has been added to the Embree scene. */
    const EmbreeScene* _embree_scene;
};

} // namespace Geometry

#endif // __DEFORMABLE_SDF_HPP