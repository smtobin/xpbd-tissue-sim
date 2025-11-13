#ifndef __REFINED_TET_MESH_HPP
#define __REFINED_TET_MESH_HPP

#include "geometry/TetMesh.hpp"

#include "common/EigenHash.hpp"

#include <unordered_map>
#include <unordered_set>

namespace Geometry
{

class RefinedTetMesh : public TetMesh
{
public:
    /** Represents a node in the tree structure that represents the hierarchical refinement. */
    struct ElementTreeNode
    {
        static constexpr int INVALID_INDEX = -1;

        Vec4i vertices;     // the vertex indices of the element
        int element_index = INVALID_INDEX;  // the index of the element in the _elements vector (only applicable for leaf nodes)
        int parent;         // the index of the parent TreeNode
        std::vector<int> children;  // the TreeNode children indices - up to 8 children
        int level;          // the level of refinement this node is at. Level 0 = base tetrahedron

        bool f123_on_surface = false;
        bool f124_on_surface = false;
        bool f134_on_surface = false;
        bool f234_on_surface = false;

        bool isLeaf() const { return children.size() == 0; }

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

    /** Recursively subdivides the specified element refinement_level times.
     * Each parent tetrahedron at each level is split into 8 equal volume tetrahedra by introducing 6 new vertices at edge midpoints.
     * No duplicate vertices are created, and hanging vertices are tracked.
    */
    void refineElement(int element_index, int refinement_level);

    /** Recursively coarsens the specified element coarsening_level times.
     * This function assumes that the element was created from hiearchical subdivision. (i.e. from the refineElement function)
     * If coarsening one level, the element and all of its siblings will be replaced by their parent element (8 elements -> 1 element)
     * If coarsening two levels, the element and all of its siblings and cousins will be replaced by their grandparent element (64 elements -> 1 element)
     * 
     * If coarsening_level is set to -1, all refinement that resulted in the leaf element is undone.
     * 
     * If the specified element was not created with mesh refinement, this function does nothing.
     */
    void coarsenElement(int element_index, int coarsening_level);

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