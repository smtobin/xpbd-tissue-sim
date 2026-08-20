#ifndef __TET_MESH_GRAPHICS_OBJECT_HPP
#define __TET_MESH_GRAPHICS_OBJECT_HPP

#include "graphics/GraphicsObject.hpp"
#include "geometry/TetMesh.hpp"
#include "config/simobject/MeshObjectConfig.hpp"

#include "common/types.hpp"

namespace Graphics
{

/** Handles visualization for mesh-based simulation objects.
 * 
 * - Because the visualization only deals with vertices/edges/faces, this class does not care if the mesh
 *   is deformable or not
 * - Does not implement the update() method - this requires knowing the specific graphics engine used (i.e. Easy3D, OptiX, etc.)
 * - Stores all visual information associated with a mesh (i.e. texture coordinates, vertex coloring)
 * 
 */
class TetMeshGraphicsObject : public GraphicsObject
{
    public:

    /** A struct for storing just the information needed for rendering a mesh. */
    struct RenderInfo
    {
        Geometry::Mesh::vertices_vec_type vertices;
        Geometry::Mesh::faces_vec_type faces;
        Geometry::Mesh::elements_vec_type elements;
        std::vector<std::vector<Vec3i>> interior_faces;
        std::vector<Vec3r> vertex_normals;
        Geometry::PropertyContainer<Geometry::MeshPropertyTypeList> vertex_properties;
        Geometry::PropertyContainer<Geometry::MeshPropertyTypeList> face_properties;
        Geometry::PropertyContainer<Geometry::MeshPropertyTypeList> element_properties;
        unsigned long topology_version;

        template <typename T>
        const Geometry::MeshProperty<T>& getVertexProperty(const std::string& name) const
        {
            static_assert(type_list_contains_v<T, Geometry::MeshPropertyTypeList> && "Mesh property type not supported!");

            for (const auto& vprop : vertex_properties.template get<Geometry::MeshProperty<T>>())
            {
                if (name == vprop.name())
                    return vprop;
            }
        
            assert(0 && "Vertex property not found!");
        }

        template <typename T>
        bool hasVertexProperty(const std::string& name) const
        {
            static_assert(type_list_contains_v<T, Geometry::MeshPropertyTypeList> && "Mesh property type not supported!");

            for (auto& vprop : vertex_properties.template get<Geometry::MeshProperty<T>>())
            {
                if (name == vprop.name())
                    return true;
            }

            return false;
        }

        template <typename T>
        const Geometry::MeshProperty<T>& getFaceProperty(const std::string& name) const
        {
            static_assert(type_list_contains_v<T, Geometry::MeshPropertyTypeList> && "Mesh property type not supported!");

            for (const auto& fprop : face_properties.template get<Geometry::MeshProperty<T>>())
            {
                if (name == fprop.name())
                    return fprop;
            }
        
            assert(0 && "Vertex property not found!");
        }

        template <typename T>
        bool hasFaceProperty(const std::string& name) const
        {
            static_assert(type_list_contains_v<T, Geometry::MeshPropertyTypeList> && "Mesh property type not supported!");

            for (auto& fprop : face_properties.template get<Geometry::MeshProperty<T>>())
            {
                if (name == fprop.name())
                    return true;
            }

            return false;
        }

        template <typename T>
        const Geometry::MeshProperty<T>& getElementProperty(const std::string& name) const
        {
            static_assert(type_list_contains_v<T, Geometry::MeshPropertyTypeList> && "Mesh property type not supported!");
            
            for (auto& fprop : element_properties.template get<Geometry::MeshProperty<T>>())
            {
                if (name == fprop.name())
                    return fprop;
            }
        
            throw std::runtime_error("Element property not found!");
            return element_properties.template get<Geometry::MeshProperty<T>>().front();
        }

        template <typename T>
        bool hasElementProperty(const std::string& name) const
        {
            static_assert(type_list_contains_v<T, Geometry::MeshPropertyTypeList> && "Mesh property type not supported!");

            for (const auto& fprop : element_properties.template get<Geometry::MeshProperty<T>>())
            {
                if (name == fprop.name())
                    return true;
            }

            return false;
        }
    };

    /** Creates a TetMeshGraphicsObject with a given name and for a given MeshObject
     * @param name : the name of the new TetMeshGraphicsObject
     * @param mesh_object : the simulation MeshObject to get mesh information from
     */
    explicit TetMeshGraphicsObject(const std::string& name, const Geometry::TetMesh* mesh);

    /** Creates a TetMeshGraphicsObject with a given name and for a given MeshObject, and sets additional parameters
     * using a MeshObjectConfig object.
     * @param name : the name of the new TetMeshGraphicsObject
     * @param mesh_object : the simulation MeshObject to get mesh information from
     * @param mesh_object_config : the MeshObjectConfig file to get additional parameters from
     */
    explicit TetMeshGraphicsObject(const std::string& name, const Geometry::TetMesh* mesh_object, const Config::MeshObjectConfig* mesh_object_config);

    virtual ~TetMeshGraphicsObject();

    /** Updates the snapshot of the mesh that should be rendered. */
    void update() override;

    /** Returns the underlying simulation MeshObject
     * @returns the underlying simulation MeshObject
     */
    const Geometry::TetMesh* mesh() { return _mesh; }

    protected:
    /** The underlying simulation MeshObject */
    const Geometry::TetMesh* _mesh;

    /** Double-buffered render meshes. Updated via the update() function. */
    RenderInfo _rmesh1;
    RenderInfo _rmesh2;

    /** Atomic variable to synchronize double buffer */
    std::atomic<RenderInfo*> _latest_rmesh;

    /** Whether or not to draw mesh vertices on screen */
    bool _draw_points;
    /** Whether or not to draw mesh edges on screen */
    bool _draw_edges;
    /** Whether or not to draw mesh faces on screen */
    bool _draw_faces;

    /** For now, mesh has a constant coloring
     * Use a 4-vector, RGBA format
     */
    Vec4r _color;
};

} // namespace Graphics


#endif // __MESH_GRAPHICS_OBJECT_HPP