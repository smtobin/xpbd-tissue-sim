#ifndef __TET_MESH_HPP
#define __TET_MESH_HPP

#include "geometry/Mesh.hpp"

#include <unordered_map>

namespace Geometry
{

/** A class for a tetrahedral mesh which consists of a set of vertices, and a set of volumetric tetrahedral elements connecting those vertices.
 * Note that this class extends the Mesh base class, meaning that it also has a matrix for faces.
 *  - These faces are only SURFACE faces - this is useful for things like visualization and collision detection.
 * The elements are specified as 4-vectors of element indices.
 */
class TetMesh : public Mesh
{
    public:
    /** Constructs a tetrahedral mesh from a set of vertices, faces, and elements.
     * This is usually done using the helper methods in the MeshUtils library.
     */
    TetMesh(const std::vector<Vec3r>& vertices, const std::vector<Vec3i>& faces, const std::vector<Vec4i>& elements);

    TetMesh(const TetMesh& other);

    TetMesh(TetMesh&& other);

    virtual ~TetMesh() = default;

    /** Essentially "sets up" the mesh - treats the current state as the initial, undeformed state of the mesh.
     * This should be called after performing the initial translations and rotations setting up the mesh.
     */
    virtual void setCurrentStateAsUndeformedState() override;

    /** Returns the rest volume associated with the specified vertex.
     * (1/4 the rest volume of all attached elements to the vertex)
     */
    Real vertexRestVolume(int index) const { return _vertex_rest_volumes[index]; }

    /** Returns a const-reference to the elements of the mesh. */
    const elements_vec_type& elements() const { return _elements; }

    /** Returns a non-const-reference to the elements of the mesh. */
    elements_vec_type& elements() { return _elements; }

    /** Returns the number of elements in the mesh. */
    int numElements() const { return _elements.size(); }

    /** Returns a single element as an Eigen 4-vector, given the element index. */
    Vec4i element(int index) const { return _elements.at(index); }

    /** Returns whether or not the index corresponds to a valid element. */
    bool elementValid(int index) const { return _elements.indexValid(index); }

    /** Returns the current volume of the specified element. */
    Real elementVolume(int index) const;
    Real elementVolume(const Vec3r& v1, const Vec3r& v2, const Vec3r& v3, const Vec3r& v4) const;

    /** Returns the rest volume of the specified element. */
    Real elementRestVolume(int index) const { return _element_rest_volumes[index]; }

    const Mat3r& elementInvUndeformedBasis(int index) const { return _element_inv_undeformed_basis[index]; }

    /** Returns deformation gradient for the specified element.
     * Assumes linear shape functions (deformation gradient is constant throughout the element)
     */
    Mat3r elementDeformationGradient(int index) const;

    /** Returns the element index corresponding to a surface face. */
    int elementWithFace(int face_index) const { return _surface_face_to_element_map.at(face_index); }

    bool elementOnSurface(int element_index) const { return _element_to_surface_faces_map.count(element_index) > 0; }

    /** Returns the element indices for elements that are face-adjacent (share a face) with an element specified by its index. */
    std::vector<int> faceAdjacentElements(int elem_index);

    /** Removes the element that corresponds to a surface face.
     * All surface faces associated with the removed element are removed.
     * New surface faces are added to fill the hole - these faces will be faces from adjacent elements.
     */
    void removeElementWithFace(int face_index);

    /** Removes an element from the mesh.
     * All surface faces associated with the removed element are removed.
     * New surface faces are added to fill the hole - these faces will be faces from adjacent elements.
     */
    virtual void removeElement(int elem_index);

    /** Returns the number of edges along with the average edge length in the tetrahedra of the mesh.
     * Note that this is different from averageFaceEdgeLength, which only returns the average edge length in the faces (i.e. the surface) of the mesh.
     */
    std::pair<int, Real> averageTetEdgeLength() const;

#ifdef HAVE_CUDA
    virtual void createGPUResource() override;
#endif

    const std::vector<int>& vertexAttachedElements(int vertex_index) const { return _vertex_to_elements_map[vertex_index]; }

    /** Creates an element property with the specified name, and optional default value. */
    template <typename T>
    void addElementProperty(const std::string &name, std::optional<T> default_value = std::nullopt, bool is_field = false)
    {
        static_assert(type_list_contains_v<T, MeshPropertyTypeList> && "Mesh property type not supported!");

        // make sure name doesn't already exist
        for (const auto& eprop : _element_properties.get<MeshProperty<T>>())
        {
            bool exists = name == eprop.name();
            if (exists)
                throw std::runtime_error("Element property with name already exists!");
        }
    
        if (default_value.has_value())
        {
            _element_properties.template emplace_back<MeshProperty<T>>(name, numElements(), default_value.value(), is_field);
        }
        else
        {
            _element_properties.template emplace_back<MeshProperty<T>>(name, numElements(), is_field);
        }
    }

    /** Fetches an element property with name. */
    template <typename T>
    const MeshProperty<T>& getElementProperty(const std::string& name) const
    {
        static_assert(type_list_contains_v<T, MeshPropertyTypeList> && "Mesh property type not supported!");

        for (const auto& fprop : _element_properties.template get<MeshProperty<T>>())
        {
            if (name == fprop.name())
                return fprop;
        }
    
        throw std::runtime_error("Element property not found!");
        return _element_properties.template get<MeshProperty<T>>().front();
    }

    template <typename T>
    MeshProperty<T>& getElementProperty(const std::string& name)
    {
        static_assert(type_list_contains_v<T, MeshPropertyTypeList> && "Mesh property type not supported!");
        
        for (auto& fprop : _element_properties.template get<MeshProperty<T>>())
        {
            if (name == fprop.name())
                return fprop;
        }
    
        throw std::runtime_error("Element property not found!");
        return _element_properties.template get<MeshProperty<T>>().front();
    }

    template <typename T>
    bool hasElementProperty(const std::string& name) const
    {
        static_assert(type_list_contains_v<T, MeshPropertyTypeList> && "Mesh property type not supported!");

        for (const auto& fprop : _element_properties.template get<MeshProperty<T>>())
        {
            if (name == fprop.name())
                return true;
        }

        return false;
    }

    protected:

    /** Helper function to add a new surface face to the mesh.
     *   - Updates the surface face -> element map and the element -> surface face map.
     *   - Resizes face properties.
     * 
     * Returns the index of the new face.
     */
    virtual int _addFace(const Vec3i& new_face, int elem_with_face);

    /** Finds adjacent vertices for each vertex in the mesh.
     * Two vertices are "adjacent" if they are connected by a face or element.
     * 
     * Overrides the behavior in Geometry::Mesh to consider tetrahedral elements instead of only surface faces.
     */
    virtual void _computeAdjacentVertices() override;

    /** Updates all the element maps when a new element is created.
     * Specifically, updates:
     *   - vertex -> element map (4 entries added, one for each vertex in the new element)
     *   - edge -> element map   (6 entries added, one for each edge in the new element)
     *   - face -> element map   (4 entries added, one for each face in the new element)
     */
    void _updateElementMapsForNewElement(int element_index);

    /** Updates all the element maps when an element is removed from the mesh.
     * Specifically, updates:
     *   - vertex -> element map (4 entries removed, one for each vertex in the removed element)
     *   - edge -> element map   (6 entries removed, one for each edge in the removed element)
     *   - face -> element map   (4 entries removed, one for each face in the removed element)
     *   - element -> surface face map (all entries associated with the removed element are removed)
     */
    void _updateElementMapsForRemovedElement(int element_index);

    /** Updates the vertex -> element map when we are removing an element. */
    virtual void _updateVertexElementMapForRemovedElement(int element_index);
    /** Updates the edge -> element map when we are removing an element. */
    void _updateEdgeElementMapForRemovedElement(int element_index);
    /** Updates the face -> element map when we are removing an element. */
    void _updateFaceElementMapForRemovedElement(int element_index);
    /** Updates the element -> surface face map when we are removing an element. */
    void _updateElementSurfaceFaceMapForRemovedElement(int element_index);

    /** Simple helper to subtract 1/4 the element volume from its vertices */
    void _updateVertexVolumesForRemovedElement(int element_index);

    /** Matrix of tetrahedral elements - each column is 4 integers corresponding to the vertex indices */
    elements_vec_type _elements;

    /** Per-element properties */
    PropertyContainer<MeshPropertyTypeList> _element_properties;

    /** The rest volumes for each element */
    std::vector<Real> _element_rest_volumes;

    /** inverse undeformed basis for each element
     *   - used in calculating the deformation gradient (F = XQ) where X is current deformed basis, Q is inverse undeformed basis
     *  calculated [v1 - v4, v2 - v4, v3 - v4]
   */
    std::vector<Mat3r> _element_inv_undeformed_basis;  

    /** The rest volume associated with a vertex
     * (1/4 the volume of the elements attached to the vertex)
     */
    std::vector<Real> _vertex_rest_volumes;

    /** A list of elements that are on the surface, i.e. one of their faces is on the surface.
     * Entry i is the index of the element that corresponds to surface face i.
     * A single element may have multiple faces exposed to the surface, thus there may be duplicate indices in the vector
     */
    std::vector<int> _surface_face_to_element_map;

    /** Maps elements to their surface faces. */
    std::unordered_multimap<int, int> _element_to_surface_faces_map;
    

    /** Lists the elements (by index) attached to a vertex. */
    std::vector<std::vector<int>> _vertex_to_elements_map; 

    /** Maps interior edges to the elements that share that edge.
     * I.e. given an edge (v1,v2), the multimap stores all the indices for the elements that share that edge
     */
    std::unordered_multimap<Edge, int, EdgeHash> _edge_to_elements_map;

    /** Maps faces (interior and exterior) to the elements that share that face.
     * I.e. given a face (v1,v2,v3), the multimap stores all the indices for the elements that share that face.
     * This is either 0 (key is not in the map), 1, or 2 elements.
     */
    std::unordered_multimap<Face, int, FaceHash> _face_to_elements_map;
};

} // namespace Geometry

#endif // __TET_MESH_HPP
