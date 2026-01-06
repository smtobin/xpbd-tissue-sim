#ifndef __REFINED_TET_MESH_HPP
#define __REFINED_TET_MESH_HPP

#include "geometry/TetMesh.hpp"

#include "common/EigenHash.hpp"

#include <unordered_map>
#include <unordered_set>
#include <array>
#include <limits>

namespace Geometry
{


/** Represents a selectively hierarchically refined tetrahedral mesh.
 * 
 * A single tetrahedral refinement consists of so-called "red" refinement: splitting the tetrahedra into 8 equal volume children by
 * introducing vertices at the edge midpoints. There are 4 "corner" tetrahedra and an octahedron in the center which is split into 4 tetrahedra.
 * Note that these child tetrahedra are NOT similar (i.e. it is not the same as dividing a triangle into 4 similar subtriangles).
 * 
 * The elements can be recursively split to produce a nested, tree-like structure. This is indeed how it is represented in the code, by an
 * octtree data structure. Each "element node" has 8 children tree nodes corresponding to the 8 children tetrahedron from splitting. Each of
 * those elements may also have 8 children, and so on. The leaf nodes in this tree are the elements that are actually in the mesh.
 * 
 * We can undo refinement by "coarsening" the mesh, essentially reversing a certain number of levels of refinement involving some tetrahedron.
 * 
 * While we refine/coarsen, we keep track of which vertices are "hanging", i.e. are on an edge in the mesh. This happens when face- or edge- adjacent
 * elements have different levels of refinement, leading to non-conforming (hanging) vertices. Tracking these is important for accurately computing
 * deformation, heat and voltage.
 * 
 * In order to track which vertices are hanging, a "feature hierarchy" data structure is employed. When splitting happens, each edge and face in the 
 * original tetrahedron is split as well into child features, due to the addition of vertices at edge midpoints.
 * For example, each edge is split into 2 child edges. Each triangular face is split into 4 subfaces, but also has 3 child edges (the edges that divide
 * the original face into 4 subfaces. These edges are between the midpoints that belong the face being split.).
 * 
 * By storing the feature hierarchy in this way, we can track which edges and faces actually have elements associated with them. For example, for 
 * an edge that is actively being "used" by an element in the mesh, any child vertices on the edge and all of its descendant edges are hanging!
 * 
 * =====================================================================================
 * Illustrative example of face being split
 *          1 *-----------*                     1  *-----------*
 *           / \         /                        / \         /
 *          /   \       /                        /   \       /
 *         /     \     /     ==>              4 *-----* 5   /
 *        /       \   /                        / \   / \   /
 *       /         \ /                        /   \ /   \ /
 *    2 *-----------* 3                    2 *-----*-----* 3
 *                                                 6
 * Parent face node: F123
 *   Children edge nodes: E45, E46, E56
 *   Children face nodes: F145, F246, F356, F456
 * 
 * Parent edge node: E12
 *   Children edge nodes: E14, E24
 * 
 * Parent edge node: E13
 *   Children edge nodes: E15, E35
 * 
 * Parent edge node: E23
 *   Children edge nodes: E26, E36
 * 
 * Edge 13 is still in the mesh ==> Vertex 5 is hanging!
 * ====================================================================================
 * 
 */
class RefinedTetMesh : public TetMesh
{
public:
    /** Represents a node in the tree structure that represents the hierarchical refinement. */
    struct ElementTreeNode
    {
        static constexpr int INVALID_INDEX = -1;

        Vec4i vertices;     // the vertex indices of the element
        int element_index = INVALID_INDEX;  // the index of the element in the _elements vector (only applicable for leaf nodes)
        int parent = INVALID_INDEX;         // the index of the parent TreeNode
        std::vector<int> children;  // the TreeNode children indices - up to 8 children
        int level;          // the level of refinement this node is at. Level 0 = base tetrahedron

        // the edge nodes corresponding to the edges in this element
        // stored in a specific order: E01, E02, E03, E12, E13, E23 (numbers refer to indices in the "vertices" member)
        std::array<int,6> edge_nodes = {INVALID_INDEX, INVALID_INDEX, INVALID_INDEX, INVALID_INDEX, INVALID_INDEX, INVALID_INDEX};
        // the face nodes corresponding to the faces in this elements
        // stored in a specific order: F012, F013, F023, F123 (numbers refer to indices in the "vertices" member)
        std::array<int,4> face_nodes = {INVALID_INDEX, INVALID_INDEX, INVALID_INDEX, INVALID_INDEX};

        // if this element has no children, then it is a leaf node!
        bool isLeaf() const { return children.size() == 0; }

        /** Initialize the node from the vertices in the element, the parent tree node index, and the depth in the tree this node is at.
         * This will initialize the edge_nodes and face_nodes to be all invalid.
         */
        ElementTreeNode(const Vec4i& vertices_, int parent_, int level_)
            : vertices(vertices_), parent(parent_), level(level_)
        {
            // preemptively reserve space for the children
            children.reserve(8);
        }

        /** Initial the node from the vertices in the element, parent tree node index, depth in the tree,
         * and the edge nodes and face nodes for the features in this element.
         */
        ElementTreeNode(const Vec4i& vertices_, int parent_, int level_,
            const std::array<int,6>& edge_nodes_, const std::array<int,4>& face_nodes_)
            : vertices(vertices_), parent(parent_), level(level_),
                edge_nodes(edge_nodes_), face_nodes(face_nodes_)
        {
            // preemptively reserve space for the children
            children.reserve(8);
        } 
    };

    /** Represents a edge feature in the feature hierarchy tree.
     * Each edge node has a parent (an edge or a face), two child edge nodes and a child midpoint vertex.
     */
    struct EdgeNode
    {
        // an edge in the mesh can "belong" to a parent face or a parent edge
        int parent_face_node = ElementTreeNode::INVALID_INDEX;
        int parent_edge_node = ElementTreeNode::INVALID_INDEX;
        // the actual edge
        Edge edge;
        // if this edge is not split (i.e. has no children)
        bool is_leaf = true;
        // if this edge, or any of its ancestor features are "in" the mesh (i.e. if this edge is part of an active tet in the mesh)
        // this ultimately dictates whether or not the child vertex is hanging or not
        // e.g. if the grandparent edge to this one is an edge for element 10, then in_mesh = true for this edge node and its parent
        bool in_mesh = false;

        // child edge nodes
        int child_edge_node1 = ElementTreeNode::INVALID_INDEX;
        int child_edge_node2 = ElementTreeNode::INVALID_INDEX;

        // child (midpoint) vertex. this is the index in the _vertices vector
        int child_vertex = ElementTreeNode::INVALID_INDEX;

        EdgeNode(const Edge& edge_)
            : edge(edge_)
        {
        }

        EdgeNode(int v1, int v2)
            : edge(v1, v2)
        {
        }
    };

    /** Represents a face feature in the feature hierarchy tree.
     * Each face node has a parent face node, 4 child face nodes and 3 child edge nodes.
     */
    struct FaceNode
    {
        // a face in the mesh can only "belong" to a parent face
        int parent_face_node = ElementTreeNode::INVALID_INDEX;
        // the actual face
        Face face;
        // if this face is not split (i.e. has no children)
        bool is_leaf = true;
        // if this face, or any of its ancestor faces are "in" the mesh (i.e. if this face is part of an active tet in the mesh)
        // this ultimately dictates whether vertices on the midpoint of descendant edges are hanging or not
        // e.g. if the grandparent face to this one is an face for element 10, then in_mesh = true for this face node and its parent 
        bool in_mesh = false;
        // whether this face is on the outer surface of the mesh
        bool on_surface = false;

        // the child edge nodes
        std::array<int,3> child_edge_nodes = {ElementTreeNode::INVALID_INDEX, ElementTreeNode::INVALID_INDEX, ElementTreeNode::INVALID_INDEX};
        // the child face nodes
        std::array<int,4> child_face_nodes = {ElementTreeNode::INVALID_INDEX, ElementTreeNode::INVALID_INDEX, ElementTreeNode::INVALID_INDEX, ElementTreeNode::INVALID_INDEX};

        FaceNode(const Face& face_)
            : face(face_)
        {
        }

        FaceNode(int v1, int v2, int v3)
            : face(v1, v2, v3)
        {
        }
    };

    /** Simple struct to store information about an element that was removed. */
    struct RemovedElement
    {
        int index;
        const Vec4i vertices;
        Real rest_volume;

        RemovedElement(int index_, const Vec4i& vertices_, Real rest_volume_)
            : index(index_), vertices(vertices_), rest_volume(rest_volume_)
        {}
    };

    /** Simple struct to store information about a vertex that was added. */
    struct NewVertex
    {
        int index;
        int parent1;
        int parent2;

        NewVertex(int index_, int parent1_, int parent2_)
            : index(index_), parent1(parent1_), parent2(parent2_)
        {}
    };

    /** Simple struct to store information about a vertex that was removed. */
    struct RemovedVertex
    {
        int index;
        int parent1;
        int parent2;

        RemovedVertex(int index_, int parent1_, int parent2_)
            : index(index_), parent1(parent1_), parent2(parent2_)
        {}
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

    /** Essentially "sets up" the mesh - treats the current state as the initial, undeformed state of the mesh.
     * This should be called after performing the initial translations and rotations setting up the mesh.
     */
    virtual void setCurrentStateAsUndeformedState() override;

    Vec3r initialVertex(int index) const { return _initial_vertices[index]; }

    int elementRefinementLevel(int index) const { return _element_refinement_level[index]; }

    /** Recursively subdivides the specified element refinement_level times.
     * Each parent tetrahedron at each level is split into 8 equal volume tetrahedra by introducing 6 new vertices at edge midpoints.
     * No duplicate vertices are created, and hanging vertices are tracked.
    */
    bool refineElement(int element_index, int refinement_level, bool absolute=false);

    /** Recursively coarsens the specified element coarsening_level times.
     * This function assumes that the element was created from hiearchical subdivision. (i.e. from the refineElement function)
     * If coarsening one level, the element and all of its siblings will be replaced by their parent element (8 elements -> 1 element)
     * If coarsening two levels, the element and all of its siblings and cousins will be replaced by their grandparent element (64 elements -> 1 element)
     * 
     * To undo all refinement that resulted in the leaf element, use coarsening_level=0 and absolute=true.
     * 
     * If the specified element was not created with mesh refinement, this function does nothing.
     */
    bool coarsenElement(int element_index, int coarsening_level, bool absolute=false);

    /** Returns the current set of vertex indices that are hanging (i.e. non-conforming). */
    const std::unordered_map<int, Edge>& hangingVertices() const { return _hanging_vertices; }

    /** Goes through the entire mesh and finds all hanging vertices.
     * Useful for verifying that we are marking the hanging vertices correctly.
     */
    std::unordered_set<int> verifyHangingVertices() const;

    /** Removes an element from the mesh. */
    virtual void removeElement(int elem_index) override;

    /** Accessors for querying info about last refine/coarsen operation. */
    const std::vector<NewVertex>& latestAddedVertices() const { return _latest_new_vertices; }
    const std::vector<RemovedVertex>& latestRemovedVertices() const { return _latest_removed_vertices; }
    const std::vector<int>& latestAddedFaces() const { return _latest_new_faces; }
    const std::vector<int>& latestAddedElements() const { return _latest_new_elements; }
    const std::vector<RemovedElement>& latestRemovedElements() const { return _latest_removed_elements; }
    const std::vector<NewVertex>& latestAddedHangingVertices() const { return _latest_new_hanging_vertices; }
    const std::vector<int>& latestRemovedHangingVertices() const { return _latest_removed_hanging_vertices; }

protected:
    
    /** Updates the vertex -> element map when we are removing an element with index element_index.
     * This implements additional logic to update the parent edge -> child vertex map when a vertex is removed from the mesh.
     */
    virtual void _updateVertexElementMapForRemovedElement(int element_index) override;

private:

    /** Updates the feature hierarchy for a removed element tree node.
     * 
     * For an edge or face to be removed, it must:
     *   - not have any children (i.e. it is a leaf)
     *   - not be in the mesh itself
     */
    void _updateFeatureHierarchyForRemovedElementTreeNode(int element_tree_node_index);

    /** Adds a new element to the mesh given an ElementTreeNode.
     * An element tree node has all the information we need to add a new element to the mesh.
     * Calls _addNewElement under the hood.
     */
    int _addNewElementFromElementTreeNode(int tree_node_index);

    /** Adds a new element to the mesh. Resizes element properties accordingly.
     * Calculates the rest volume and inverse undeformed basis for the element, using the initial vertex positions.
     * Updates element maps.
     * Does NOT update adjacent vertices. This happens where the edge nodes are created.
     * @param new_element : the vertices of the new element
     * @param f012_on_surface, f013_on_surface, f023_on_surface, f123_on_surface : whether a given face is on the outer surface of the mesh. If it is, we need to add a new surface face to the mesh appropriately.
     */
    int _addNewElement(const Vec4i& new_element, bool f012_on_surface, bool f013_on_surface, bool f023_on_surface, bool f123_on_surface);


    /** Adds a new vertex to the mesh at the midpoint between the two specified parents. Resizes vertex properties accordingly.
     * Adds an entry to the _initial_vertices vector by interpolating the initial positions of the parents.
     * @returns the new vertex index
     */
    int _addVertex(int parent1, int parent2);

    /** Adds a new face to the mesh. Resizes face properties accordingly.
     * 
     * @returns the new face index
     */
    int _addFace(const Vec3i& new_face);

    /** Helper functions for element refinement */

    /** When we are preparing to refine an element, we need to update child features (up to the refinement).
     * 
     * We update only the child features of the features in the element who either don't have a parent feature or have a parent feature
     * who with in_mesh = false (i.e. features with no ancestor features in the mesh). This means that when we remove this element and
     * replace it with smaller subelements, those features and their children should have in_mesh = false (until we get to the depth of refinement
     * where in_mesh = true because those descendant features will all be in the mesh).
     * 
     * Ultimately this is needed so that the hanging vertices can be determined appropriately.
     * 
     * @param element_tree_node_index : the index of the element tree node that we are about to refine
     * @param depth : the depth of refinement we are preparing to do
     */
    void _prepareFeatureTreeForRefinedElement(int element_tree_node_index, int depth);

    void _createMidpointVerticesAndChildEdgeNodesForElement(int element_tree_node_index, std::array<int,6>& midpoint_vertices, bool at_refinement_depth);

    void _createChildFaceNodesForElement(int element_tree_node_index, const std::array<int,6>& midpoint_vertices, bool at_refinement_depth);

    std::pair<int,int> _matchChildEdgeNodeIndices(int parent_edge_node_index, int lower);
    std::tuple<int,int,int> _matchFaceNodeToChildEdgeNodeIndices(int parent_face_node_index, int lower, int middle);
    std::tuple<int,int,int,int> _matchFaceNodeToChildFaceNodeIndices(int parent_face_node_index, int v0, int v1, int v2, int m01, int m02, int m12);

protected:
    
    /** Store the initial vertices so that when we add new vertices, we can interpolate where their initial positions would be.
     * This is useful for calculating the inverse undeformed basis (Q in XPBD) for each new element. (used in deformation gradient computation)
     * The initial vertices are reset every time setCurrentStateAsUndeformedState() is called.
     */
    std::vector<Vec3r> _initial_vertices;

    /** Stores the refinement level of each element. Unreefined elements in the base tet mesh have refinement level = 0. */
    std::vector<int> _element_refinement_level;

    /** Stores the recursive refinement tree structure. This enables us to coarsen the mesh (i.e. undo refinement). */
    TombstoneVector<ElementTreeNode> _element_tree_nodes;

    /** Maps element indices (in the _elements vector) back to the associated node in the tree.
     * Only elements that were created from refinement will have entries in the map.
     */
    std::unordered_map<int, int> _element_to_tree_node_map;

    /** Stores the indices of vertices that are "hanging", and maps that to their parent edge.
     *  (the parent edge is needed to constrain the hanging node in XPBD and in FEM)
     * 
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
    std::unordered_map<int, Edge> _hanging_vertices;

    /** Store feature hierarchy */
    TombstoneVector<EdgeNode> _edge_nodes;
    TombstoneVector<FaceNode> _face_nodes;

    /** Map edges -> index in the edge node vector */
    std::unordered_map<Edge, int, EdgeHash> _edge_to_edge_node_map;

    /** Map faces -> index in the face node vector */
    std::unordered_map<Face, int, FaceHash> _face_to_face_node_map;

    /** Stores the most recently added vertices (from a refineElement or coarsenElement call) */
    std::vector<NewVertex> _latest_new_vertices;
    std::vector<RemovedVertex> _latest_removed_vertices;

    /** Stores the most recently added faces (from a refineElement or coarsenElement call) */
    std::vector<int> _latest_new_faces;

    /** Stores the most recently added elements (from a refineElement or coarsenElement call) */
    std::vector<int> _latest_new_elements;
    std::vector<RemovedElement> _latest_removed_elements;

    /** Stores the most recently added hanging vertices (from a refineElement or coarsenElement call) */
    std::vector<NewVertex> _latest_new_hanging_vertices;
    std::vector<int> _latest_removed_hanging_vertices;

};

} // namespace Geometry

#endif // __REFINED_TET_MESH_HPP