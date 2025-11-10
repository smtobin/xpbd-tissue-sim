#ifndef __REFINED_FEM_ELEMENT_HPP
#define __REFINED_FEM_ELEMENT_HPP

#include "common/types.hpp"

#include "common/TombstoneVector.hpp"

#include <vector>
#include <map>
#include <unordered_map>

namespace FEM
{



class RefinedFEMElement
{
    public:

    enum class TetEdge
    {
        E12 = 0,
        E13,
        E14,
        E23,
        E24,
        E34,
        NONE
    };

    enum class TetFace
    {
        F123 = 0,
        F124,
        F134,
        F234,
        NONE 
    };

    struct OuterSurfaceVertex
    {
        /** The index of this vertex (in the global vertices vector). */
        int index;

        /** The direct parents of this vertex (indices in the global vertices vector). */
        int parent_index1, parent_index2;

        /** The refinement level of this vertex.
         * 0 = this is one of the original tet vertices
         * 1 = this is the midpoint on an edge between origin tet vertices
         * 2 = etc.
         */
        int refinement_level;

        /** Whether or not this vertex is hanging. */
        bool hanging = false;

        // The outer surface vertex is either on an outer edge, or an outer face
        // By tracking this, we can easily determine whether this node should be hanging or not when adjacent elements are refined/unrefined

        /** What outer edge (if any) this vertex is on.
         * If the vertex is one of the original tet vertices, then it is not on an edge.
         */
        TetEdge edge = TetEdge::NONE;

        /** What outer face (if any) this vertex is on.
         * If the vertex is on an edge, then it is defined to not be on any outer face.
         */
        TetFace face = TetFace::NONE;

        OuterSurfaceVertex(int index_, int parent1_, int parent2_, int refinement_level_, bool hanging_, TetEdge edge_, TetFace face_)
            : index(index_), parent_index1(parent1_), parent_index2(parent2_), refinement_level(refinement_level_),
              hanging(hanging_), edge(edge_), face(face_)
        {}
    };

    RefinedFEMElement(TombstoneVector<Vec3r>* global_vertices, TombstoneVector<Vec3i>* global_faces, TombstoneVector<Vec4i>* global_elements,
        const Vec4i& parent_element, int refinement_level,
        std::array<int,6> adj_edge_refinements, std::array<int,4> adj_face_refinements);

    const Vec4i& originalParentElement() const { return _parent_element; }

    private:
    void _createChildren();

    /** Adds a new vertex from the two specified parent vertex indices. */
    void _addVertex(int parent_ind1, int parent_ind2);

    void _createOuterSurfaceVertex();

    /** The vector of vertices for the overall mesh.
     * This class will add new vertices it creates directly to this vector.
     */
    TombstoneVector<Vec3r>* _global_vertices;

    /** The vector of surfaces for the overall mesh.
     * This class will add new surface faces it creates directly to this vector.
     */
    TombstoneVector<Vec3i>* _global_faces;

    /** The vector of elements for the overall mesh.
     * This class will add new elements it creates directly to this vector.
     */
    TombstoneVector<Vec4i>* _global_elements;

    /** Store the original parent element entry so that we can restore it if this element is reset. */
    Vec4i _parent_element;

    /** The number of recursive hierarchical refinements to perform. */
    int _refinement_level;

    /** Minimum refinement of tets that share each edge of this tet.
     * The array is indexed by the class enum TetEdge
     * I.e. the array stores the adjacent refinement levels for the edges in the following order:
     *   E12, E13, E14, E23, E24, E34
     */
    std::array<int, 6> _adj_edge_refinements;

    /** Minimum refinement of tets that share each face of this tet.
     * The array is indexed by the class enum TetFace
     * I.e. the array stores the adjacent refinement levels for the faces in the following order:
     *   F123, F124, F134, F234
     */
    std::array<int, 3> _adj_face_refinements;

    /** Stores the child vertices (all vertices for this element) */
    std::vector<Vec3r> _child_vertices;

    /** Stores the child faces (the most refined level of surface faces for this element, if there are any) */
    std::vector<Vec3i> _child_faces;

    /** Stores the child elements (the most refined level of elements) */
    std::vector<Vec4i> _child_elements;

    /** Tracks which vertices are on the outer surface of the original tetrahedron.
     * This is useful for updating which nodes are hanging when the refinement of adjacent tets changes.
     */
    std::vector<bool> _is_outer_surface_vertex;

    /** Stores information for each vertex on the outer surface of the original tetrahedron. */
    std::map<int, OuterSurfaceVertex> _outer_surface_vertices;

    
};

} // namespace FEM

#endif // __REFINED_FEM_ELEMENT_HPP