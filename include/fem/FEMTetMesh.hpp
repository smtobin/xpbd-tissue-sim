#ifndef __FEM_MESH_HPP
#define __FEM_MESH_HPP

#include "geometry/Mesh.hpp"
#include "geometry/TetMesh.hpp"

#include "common/EigenHash.hpp"

#include <array>
#include <unordered_map>
#include <vector>

namespace FEM
{
    
/** A wrapper around Geometry::TetMesh that implements the finite element method for scalar-valued functions (i.e. temperature, voltage). */
class FEMTetMesh
{
public:
    struct Edge
    {
        int index1;
        int index2;

        Edge(int i1, int i2)
            : index1(std::min(i1,i2)), index2(std::max(i1,i2))
        {}

        Edge()
            : index1(-1), index2(-1)
        {}

        bool operator==(const Edge& other) const
        {
            return index1 == other.index1 && index2 == other.index2;
        }
    };
    struct EdgeHash
    {
        size_t operator()(const Edge& e) const {
            auto h1 = std::hash<int>{}(e.index1);
            auto h2 = std::hash<int>{}(e.index2);
            // Better mixing than simple XOR
            return h1 ^ (h2 + 0x9e3779b9 + (h1 << 6) + (h1 >> 2));
        }
    };
    
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

    //////////////////////

    struct Face
    {
        int index1, index2, index3;

        Face(int i1, int i2, int i3)
        {
            index1 = std::min({i1, i2, i3});    // index1 is minimum index
            index3 = std::max({i1, i2, i3});    // index3 is maximum index
            index2 = i1 + i2 + i3 - index1 - index3;   // index2 is in the middle
        }

        Face()
            : index1(-1), index2(-1), index3(-1)
        {}

        bool operator==(const Face& other) const
        {
            return index1 == other.index1 && index2 == other.index2 && index3 == other.index3;
        }
    };
    struct FaceHash
    {
        size_t operator()(const Face& f) const {
            auto h1 = std::hash<int>{}(f.index1);
            auto h2 = std::hash<int>{}(f.index2);
            auto h3 = std::hash<int>{}(f.index3);
            // Better mixing than simple XOR
            size_t seed = h1;
            seed ^= h2 + 0x9e3779b9 + (seed << 6) + (seed >> 2);
            seed ^= h3 + 0x9e3779b9 + (seed << 6) + (seed >> 2);
            return seed;
        }
    };

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

    /////////////////////////

    struct RefinedElement
    {

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
         */
        std::unordered_map<int, ChildVertex> child_vertices;
        /** Maps a refined vertex to its parent edge (for vertices in this refined element).  */
        std::unordered_map<Edge, int, EdgeHash> edge_to_vertex_map;
        std::vector<int> child_faces;
        std::vector<int> child_elements;

        /** Store the original base element so that we can restore it when the refined element is no longer needed. */
        Vec4i parent_element;

        /** The refinement level of the element (i.e. the number of recursive hierarchical subdivisions) */
        int refinement_level;

        

        RefinedElement(const Vec4i& parent_element_, int refinement_level_)
            : parent_element(parent_element_), refinement_level(refinement_level_)
        {
        }
    };

    //////////////////////////////////


    /** Typedef for the gradients of the element shape functions. Note that the gradient as column vector convention is used. */
    using ElementShapeFunctionGradientsMat = Eigen::Matrix<Real, 3, 4>;
    
    /** Typedef for the gradients of the face shape functions. Note that the gradient as column vector convention is used. */
    using FaceJacobianMat = Eigen::Matrix<Real, 2, 3>;

    /** Gauss quadrature points and weights */
    // for triangular faces
    static constexpr int NUM_FACE_QUADRATURE_PTS = 3;
    static constexpr std::array<Real, 3> FACE_QUADRATURE_e1 = {1.0/6.0, 2.0/3.0, 1.0/6.0};
    static constexpr std::array<Real, 3> FACE_QUADRATURE_e2 = {1.0/6.0, 1.0/6.0, 2.0/3.0};
    static constexpr std::array<Real, 3> FACE_QUADRATURE_weights = {1.0/6.0, 1.0/6.0, 1.0/6.0};
    // for tetrahedral elements
    static constexpr int NUM_ELEMENT_QUADRATURE_PTS = 1;
    static constexpr std::array<Real, 1> ELEMENT_QUADRATURE_e1 = {0.25};
    static constexpr std::array<Real, 1> ELEMENT_QUADRATURE_e2 = {0.25};
    static constexpr std::array<Real, 1> ELEMENT_QUADRATURE_e3 = {0.25};
    static constexpr std::array<Real, 1> ELEMENT_QUADRATURE_weights = {0.25};

    FEMTetMesh(Geometry::TetMesh* mesh);

    /** Element-related computations */

    /** Computes the element Jacobian (for linear tetrahedra) for the specified element.
     * Useful for computing volume integrals and tetrahedral shape function derivatives:
     *  - The determinant of this Jacobian maps volumes in the parent domain to the physical domain.
     *  - The inverse of this Jacobian maps shape function derivatives w.r.t natural coordinates to shape function derivatives w.r.t physical coordinates.
     */
    Mat3r elementJacobian(int element_index) const;

    /** Computes the shape function matrix at the specified natural coordinates (e1, e2, e3).
     */
    Vec4r elementShapeFunctions(Real e1, Real e2, Real e3) const;

    /** Computes the gradients of the shape functions for the specified element.
     * These are constant throughout the element, so no natural coordinates need to be given.
     */
    ElementShapeFunctionGradientsMat elementShapeFunctionGradients(int element_index) const;


    /** Face-related computations */

    /** Computes the face Jacobian (i.e. for a straight-sided triangle) for the specified face.
     * Useful for computing area (surface) integrals and surface shape function derivatives used by the natural boundary term:
     *  - The determinant of this Jacobian maps areas in the parent domain to the physical domain.
     *  - The inverse of this Jacobian maps shape function derivatives w.r.t natural coordinates to shape function derivatives w.r.t physical coordinates.
     */
    FaceJacobianMat faceJacobian(int face_index) const;

    /** Computes the shape function for the specified face, at the specified natural coordinates (e1, e2). */
    Vec3r faceShapeFunctions(Real e1, Real e2) const;
    

    /** Refinement */

    void refineElement(int element_index, int refinement_level);
    
private:

    int _addRefinedVertex(int parent_index1, int parent_index2,
     const Vec4i& base_element, RefinedElement* refined_element);
    

private:
    Geometry::TetMesh* _mesh;

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

    /** Stores the actual elements associated with refinement. */
    TombstoneVector<Vec4i> _temp_elements;
};

} // namespace FEM

#endif // __FEM_MESH_HPP