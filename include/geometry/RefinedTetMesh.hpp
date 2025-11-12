#ifndef __REFINED_TET_MESH_HPP
#define __REFINED_TET_MESH_HPP

#include "geometry/TetMesh.hpp"

#include "common/EigenHash.hpp"

#include <unordered_map>
#include <unordered_set>

namespace Geometry
{

// struct RefinedElement
// {
//     /** Tracks information associated with a child vertex created for a refined element. */
//     struct ChildVertex
//     {
//         int index;
//         int level;
//         TetEdge edge = TetEdge::NONE;
//         TetFace face = TetFace::NONE;

//         ChildVertex(int index_, int level_)
//             : index(index_), level(level_)
//         {}

//         ChildVertex(int index_, int level_, TetEdge edge_, TetFace face_)
//             : index(index_), level(level_), edge(edge_), face(face_)
//         {}
//     };

//     /** Keep track of the child vertices, faces, and elements.
//      * These are the vertices, faces and elements created for the refined element.
//      * Tracked as the indices in the global lists of vertices, faces, and elements.
//      */
//     std::unordered_map<int, ChildVertex> child_vertices;
//     std::vector<int> child_faces;
//     std::vector<int> child_elements;

//     /** Maps a "parent" edge to the vertex defined on its midpoint (for vertices in this refined element).
//      * This allows to easily check if we've already created a vertex on a given edge.
//       */
//     std::unordered_map<Edge, int, EdgeHash> edge_to_vertex_map;

//     /** Store the original base element so that we can restore it when the refined element is no longer needed. */
//     Vec4i parent_element;

//     /** The refinement level of the element (i.e. the number of recursive hierarchical subdivisions) */
//     int refinement_level;

//     RefinedElement(const Vec4i& parent_element_, int refinement_level_)
//         : parent_element(parent_element_), refinement_level(refinement_level_)
//     {
//     }
// };

class RefinedTetMesh : public TetMesh
{
public:
    /** Represents a node in the tree structure that represents the hierarchical refinement. */
    struct ElementTreeNode
    {
        static constexpr int INVALID_INDEX = -1;

        Vec4i vertices;     // the vertex indices of the element
        int parent;         // the index of the parent TreeNode
        std::vector<int> children;  // the TreeNode children indices - up to 8 children
        int level;          // the level of refinement this node is at. Level 0 = base tetrahedron

        bool f123_on_surface = false;
        bool f124_on_surface = false;
        bool f134_on_surface = false;
        bool f234_on_surface = false;

        ElementTreeNode(const Vec4i& vertices_, int parent_, int level_)
            : vertices(vertices_), parent(parent_), level(level_)
        {
            children.reserve(8);
        }

        ElementTreeNode(const Vec4i& vertices_, int parent_, int level_, bool f123, bool f124, bool f134, bool f234)
            : vertices(vertices_), parent(parent_), level(level_),
                f123_on_surface(f123), f124_on_surface(f124), f134_on_surface(f134), f234_on_surface(f234)
        {
            children.reserve(8);
        }
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

    const std::unordered_set<int>& hangingVertices() const { return _hanging_vertices; }

    virtual void removeElement(int elem_index) override;

protected:
    /** Updates the vertex -> element map when we are removing an element with index element_index.
     * This implements additional logic to update the parent edge -> child vertex map when a vertex is removed from the mesh.
     */
    virtual void _updateVertexElementMapForRemovedElement(int element_index) override;

private:

    int _addRefinedVertex(int parent_index1, int parent_index2);

    int _addNewElementFromElementTreeNode(int tree_node_index);
    int _addNewElement(const Vec4i& new_element, bool f123_on_surface, bool f124_on_surface, bool f134_on_surface, bool f234_on_surface);

protected:

    /** Stores the refined element structs, according to their "base" elements in the original tet mesh. */
    // std::unordered_map<Vec4i, RefinedElement, EigenHash<Vec4i>> _refined_elements;
    
    /** Stores the recursive refinement tree structure. This enables us to coarsen the mesh (i.e. undo refinement). */
    TombstoneVector<ElementTreeNode> _element_tree_nodes;

    /** Maps element indices (in the _elements vector) back to the associated node in the tree.
     * Only elements that were created from refinement will have entries in the map.
     */
    std::unordered_map<int, int> _element_to_tree_node_map;

    /** Stores child vertices that were created from refining the original mesh.
     * Each vertex is stored under the "parent edge" that it was created on.
     * (Each refined vertex is created as the midpoint of an edge between two parent vertices).
     * 
     * This is a one-to-one mapping, i.e. each parent edge should only have one child vertex.
     */
    std::unordered_map<Edge, int, EdgeHash> _parent_edge_to_child_vertex_map;

    /** Stores the indices of vertices that are "hanging".
     * A hanging vertex is one that is in the middle of an edge.
     * 
     *  *---*
     *  |\  |
     *  *-* | <----- the right-middle vertex is hanging
     *  |/ \|
     *  *---*
     * 
     * This can happen when we refine one element but the adjacent element is unrefined, or not refined to the same level.
     */
    std::unordered_set<int> _hanging_vertices;

};

} // namespace Geometry

#endif // __REFINED_TET_MESH_HPP