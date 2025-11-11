#ifndef __REFINED_TET_MESH_HPP
#define __REFINED_TET_MESH_HPP

#include "geometry/TetMesh.hpp"

#include "common/EigenHash.hpp"

#include <unordered_map>

namespace Geometry
{

struct RefinedElement
{
    /** Tracks information associated with a child vertex created for a refined element. */
    struct ChildVertex
    {
        int index;
        int level;
        TetEdge edge = TetEdge::NONE;
        TetFace face = TetFace::NONE;

        ChildVertex(int index_, int level_)
            : index(index_), level(level_)
        {}

        ChildVertex(int index_, int level_, TetEdge edge_, TetFace face_)
            : index(index_), level(level_), edge(edge_), face(face_)
        {}
    };

    /** Keep track of the child vertices, faces, and elements.
     * These are the vertices, faces and elements created for the refined element.
     * Tracked as the indices in the global lists of vertices, faces, and elements.
     */
    std::unordered_map<int, ChildVertex> child_vertices;
    std::vector<int> child_faces;
    std::vector<int> child_elements;

    /** Maps a "parent" edge to the vertex defined on its midpoint (for vertices in this refined element).
     * This allows to easily check if we've already created a vertex on a given edge.
      */
    std::unordered_map<Edge, int, EdgeHash> edge_to_vertex_map;

    /** Store the original base element so that we can restore it when the refined element is no longer needed. */
    Vec4i parent_element;

    /** The refinement level of the element (i.e. the number of recursive hierarchical subdivisions) */
    int refinement_level;

    RefinedElement(const Vec4i& parent_element_, int refinement_level_)
        : parent_element(parent_element_), refinement_level(refinement_level_)
    {
    }
};

class RefinedTetMesh : public TetMesh
{
public:
    /** Tracks information associated with a refined (added) vertex that is on an edge of the original tet mesh. */
    struct EdgeVertex
    {
        /** The index of this vertex (in the global vertices vector). */
        int index;

        /** Number of refined tets that share this vertex. */
        int shared_count;

        /** The edge in the base tet mesh that this vertex is on
         */
        Edge edge;

        /** The direct parents of this vertex (indices in the global vertices vector).
         * parent_index1 < parent_index2 always
         */
        int parent_index1, parent_index2;

        /** The refinement level of this vertex.
         * 0 = this is one of the original tet vertices
         * 1 = this is the midpoint on an edge between origin tet vertices
         * 2 = etc.
         */
        int refinement_level;

        /** Whether or not this vertex is hanging. */
        bool hanging;
    };

    /** Tracks information associated with a refined (added) vertex that is on a face (interior or exterior) of the original tet mesh. */
    struct FaceVertex
    {
        /** The index of this vertex (in the global vertices vector). */
        int index;

        /** The edge in the base tet mesh that this vertex is on
         */
        Face face;

        /** The direct parents of this vertex (indices in the global vertices vector).
         * parent_index1 < parent_index2 always
         */
        int parent_index1, parent_index2;

        /** The refinement level of this vertex.
         * 0 = this is one of the original tet vertices
         * 1 = this is the midpoint on an edge between origin tet vertices
         * 2 = etc.
         */
        int refinement_level;

        /** Whether or not this vertex is hanging. */
        bool hanging;
    };

    /** Constructs a refineable tetrahedral mesh, initialized from a set of vertices, faces, and elements.
     */
    RefinedTetMesh(const std::vector<Vec3r>& vertices, const std::vector<Vec3i>& faces, const std::vector<Vec4i>& elements);

    /** Initializes a refineable tetrahedral mesh from a basic tetrahedral mesh. */
    RefinedTetMesh(const TetMesh& tet_mesh);

    /** Copy constructor */
    // RefinedTetMesh(const RefinedTetMesh& other);

    /** Move constructor */
    // RefinedTetMesh(RefinedTetMesh&& other);

    virtual ~RefinedTetMesh() = default;

    void refineElement(int element_index, int refinement_level);

private:

    int _addRefinedVertex(int parent_index1, int parent_index2,
        const Vec4i& base_element, RefinedElement& refined_element);

protected:
    
    /** Stores all refined vertices that are created on an edge in the original tet mesh.
     * Useful for determining which vertices are "hanging".
     */
    std::unordered_multimap<Edge, EdgeVertex, EdgeHash> _edge_vertices;

    /** Stores all refined vertices that are created on a face (internal or external) in the original tet mesh.
     * Useful for determing which vertices are "hanging".
     */
    std::unordered_multimap<Face, FaceVertex, FaceHash> _face_vertices;

    /** Stores the refined element structs, according to their "base" elements in the original tet mesh. */
    std::unordered_map<Vec4i, RefinedElement, EigenHash<Vec4i>> _refined_elements;

};

} // namespace Geometry

#endif // __REFINED_TET_MESH_HPP