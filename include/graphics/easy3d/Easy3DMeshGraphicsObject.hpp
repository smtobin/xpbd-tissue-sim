#ifndef __EASY3D_MESH_GRAPHICS_OBJECT_HPP
#define __EASY3D_MESH_GRAPHICS_OBJECT_HPP

#include "graphics/MeshGraphicsObject.hpp"
#include "config/render/ObjectRenderConfig.hpp"

#include <easy3d/core/types.h>
#include <easy3d/core/model.h>

namespace Graphics
{

/** Handles visualization for mesh-based simulation objects using Easy3D.
 * 
 * - Inherits easy3d::Model so that it handles its own rendering by adding Drawables for points, edges, and faces 
 * - Implements the update() method which will update the Easy3D buffers to update the renderer
 *   according to the changes in the mesh
 * 
 */
class Easy3DMeshGraphicsObject : public MeshGraphicsObject, public easy3d::Model
{
    public:

    /** Creates a Easy3DMeshGraphicsObject with a given name and for a given MeshObject, and sets additional parameters
     * using a MeshObjectConfig object.
     * @param name : the name of the new MeshGraphicsObject
     * @param mesh_object : the simulation MeshObject to get mesh information from
     * @param mesh_object_config : the MeshObjectConfig file to get additional parameters from
     */
    explicit Easy3DMeshGraphicsObject(const std::string& name, const Geometry::Mesh* mesh_object, const Config::ObjectRenderConfig& render_config);

    virtual ~Easy3DMeshGraphicsObject();

    /** Updates graphics buffers associated with this object */
    virtual void updateGraphicsBuffers() override;

    /** Returns the easy3d::vec3 vertex cache.
     * Does NOT check if vertices are stale.
     * 
     * A required override for the easy3d::Model class.
     */
    std::vector<easy3d::vec3>& points() override { return _vertex_cache; };

    /** Returns the easy3d::vec3 vertex cache.
     * Does NOT check if vertices are stale.
     * 
     * A required override for the easy3d::Model class.
     */
    const std::vector<easy3d::vec3>& points() const override { return _vertex_cache; };

    /** Returns the faces of the mesh as a flat vector of vertex indices. Used for moving face information to GPU via Easy3d.
     * As per specified by TrianglesDrawable docs, each 3 consecutive vertices represents a face.
     * @returns a 1d vecor of vertex indices - 3 consecutive entries corresponds to a face to be rendered.
     */
    std::vector<unsigned int> facesAsFlatList() const;

    /** Returns the surface edges of the mesh as a flat vector of vertex indices. Used for moving edge information to GPU via Easy3d.
     * As per specified by LinesDrawable docs, each 2 consecutive vertices represents an edge.
     * @returns a 1d vector of vertex indicies - 2 consecutive entries corresponds to a edge to be rendered.
     */
    std::vector<unsigned int> edgesAsFlatList() const;

    protected:
    /** Updates the vertex cache. Called before redrawing to get latest updates to mesh vertices drawn.
     * Each vertex is written over the top of the previous version of itself.
     */
    void _updateVertexCache();

    void _init(const Config::ObjectRenderConfig& config);

    protected:
    /** Vector of easy3d::vec3 vertices that is updated before every redraw.
     * Used by easy3d to update the vertex buffers (i.e. the positions) of the geometry on the graphics side.
     */
    std::vector<easy3d::vec3> _vertex_cache;

};

}

#endif // __EASY3D_MESH_GRAPHICS_OBJECT_HPP