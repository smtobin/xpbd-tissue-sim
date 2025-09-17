#ifndef __TET_MESH_HPP
#define __TET_MESH_HPP

#include "geometry/Mesh.hpp"

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

    /** Returns a const-reference to the elements of the mesh. */
    const elements_vec_type& elements() const { return _elements; }

    /** Returns a non-const-reference to the elements of the mesh. */
    elements_vec_type& elements() { return _elements; }

    /** Returns the number of elements in the mesh. */
    int numElements() const { return _elements.size(); }

    /** Returns a single element as an Eigen 4-vector, given the element index. */
    Vec4i element(int index) const { return _elements.at(index); }

    /** Returns the current volume of the specified element. */
    Real elementVolume(int index) const;

    /** Returns the rest volume of the specified element. */
    Real elementRestVolume(int index) const { return _element_rest_volumes[index]; }

    /** Returns deformation gradient for the specified element.
     * Assumes linear shape functions (deformation gradient is constant throughout the element)
     */
    Mat3r elementDeformationGradient(int index) const;

    /** Returns the element index corresponding to a surface face. */
    int elementWithFace(int face_index) const { return _surface_elements[face_index]; }

    /** Removes the element that corresponds to a surface face.
     * All surface faces associated with the removed element are removed.
     * New surface faces are added to fill the hole - these faces will be faces from adjacent elements.
     * The element will not be removed, but rather just marked invalid.
     */
    void removeElementWithFace(int face_index);

    /** Returns the number of edges along with the average edge length in the tetrahedra of the mesh.
     * Note that this is different from averageFaceEdgeLength, which only returns the average edge length in the faces (i.e. the surface) of the mesh.
     */
    std::pair<int, Real> averageTetEdgeLength() const;

#ifdef HAVE_CUDA
    virtual void createGPUResource() override;
#endif

    const std::vector<int>& vertexAttachedElements(int vertex_index) const { return _attached_elements_to_vertex[vertex_index]; }

    /** Creates an element property with the specified name, and optional default value. */
    template <typename T>
    void addElementProperty(const std::string &name, std::optional<T> default_value = std::nullopt)
    {
        static_assert(type_list_contains_v<T, MeshPropertyTypeList> && "Mesh property type not supported!");

        // make sure name doesn't already exist
        for (const auto& fprop : _element_properties.get<MeshProperty<T>>())
        {
            assert(name != fprop.name() && "Vertex property with name already exists!");
        }
    
        if (default_value.has_value())
        {
            _element_properties.template emplace_back<MeshProperty<T>>(name, numElements(), default_value.value());
        }
        else
        {
            _element_properties.template emplace_back<MeshProperty<T>>(name, numElements());
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
    
        assert(0 && "Element property not found!");
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
    
        assert(0 && "Element property not found!");
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
    /** Finds adjacent vertices for each vertex in the mesh.
     * Two vertices are "adjacent" if they are connected by a face or element.
     * 
     * Overrides the behavior in Geometry::Mesh to consider tetrahedral elements instead of only surface faces.
     */
    virtual void _computeAdjacentVertices() override;


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

    /** lists the elements (by index) attached to a vertex */
    std::vector<std::vector<int>> _attached_elements_to_vertex; 

    /** lists the elements (by index) that share a face with an element */
    std::vector<std::vector<int>> _face_adjacent_elements;

    /** A list of elements that are on the surface, i.e. one of their faces is on the surface.
     * Entry i is the index of the element that corresponds to surface face i.
     * A single element may have multiple faces exposed to the surface, thus there may be duplicate indices in the vector
     */
    std::vector<int> _surface_elements;
};

} // namespace Geometry

#endif // __TET_MESH_HPP
