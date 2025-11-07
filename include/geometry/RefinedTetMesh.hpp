#ifndef __REFINED_TET_MESH_HPP
#define __REFINED_TET_MESH_HPP

#include "geometry/TetMesh.hpp"

namespace Geometry
{

class RefinedTetMesh : public TetMesh
{
    public:
    /** Constructs a refineable tetrahedral mesh, initialized from a set of vertices, faces, and elements.
     */
    RefinedTetMesh(const std::vector<Vec3r>& vertices, const std::vector<Vec3i>& faces, const std::vector<Vec4i>& elements);

    /** Initializes a refineable tetrahedral mesh from a basic tetrahedral mesh. */
    RefinedTetMesh(const TetMesh& tet_mesh);

    /** Copy constructor */
    RefinedTetMesh(const RefinedTetMesh& other);

    /** Move constructor */
    RefinedTetMesh(RefinedTetMesh&& other);

    virtual ~RefinedTetMesh() = default;

    protected:

};

} // namespace Geometry

#endif // __REFINED_TET_MESH_HPP