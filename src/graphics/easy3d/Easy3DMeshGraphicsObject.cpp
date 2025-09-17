#include "graphics/easy3d/Easy3DMeshGraphicsObject.hpp"

#include <easy3d/renderer/drawable_lines.h>
#include <easy3d/renderer/drawable_points.h>
#include <easy3d/renderer/drawable_triangles.h>
#include <easy3d/renderer/renderer.h>

namespace Graphics {

Easy3DMeshGraphicsObject::Easy3DMeshGraphicsObject(const std::string& name, const Geometry::Mesh* mesh, const Config::ObjectRenderConfig& render_config)
    : MeshGraphicsObject(name, mesh)
{
    _init(render_config);
}

void Easy3DMeshGraphicsObject::_init(const Config::ObjectRenderConfig& config)
{
    // first ensure that the vertex cache has enough space for each vertex
    _vertex_cache.resize(_mesh->numVertices());
    // then update the vertex cache to populate it initially
    _updateVertexCache();

    // create a new Renderer for this Model so that the Drawables (below) get updated
    set_renderer(std::make_shared<easy3d::Renderer>(this, true));

    if (config.drawFaces())
    {
        // create a TrianglesDrawable for the faces of the tetrahedral mesh
        easy3d::TrianglesDrawable* tri_drawable = renderer()->add_triangles_drawable("faces");
        // specify the update function for the faces
        tri_drawable->set_update_func([](easy3d::Model* m, easy3d::Drawable* d) {
            // downcast to MeshObject for access to facesAsFlatList
            Easy3DMeshGraphicsObject* mo = dynamic_cast<Easy3DMeshGraphicsObject*>(m);
            if (mo)
            {
                // update the vertex buffer and element buffer
                d->update_vertex_buffer(mo->points(), true);
                d->update_element_buffer(mo->facesAsFlatList());
            }
            
        });
        // set a uniform color for the mesh
        if (config.color().has_value())
        {
            easy3d::vec4 color(config.color().value()[0], config.color().value()[1], config.color().value()[2], config.opacity());
            tri_drawable->set_uniform_coloring(color);
        }
    }

    if (config.drawPoints())
    {
        // create a PointsDrawable for the points of the tetrahedral mesh
        easy3d::PointsDrawable* points_drawable = renderer()->add_points_drawable("vertices");
        // specify the update function for the points
        points_drawable->set_update_func([](easy3d::Model* m, easy3d::Drawable* d) {
            // update the vertex buffer with the vertices of the mesh
            d->update_vertex_buffer(m->points(), true);
        });
    }

    if (config.drawEdges())
    {
        easy3d::LinesDrawable* lines_drawable = renderer()->add_lines_drawable("lines");
        lines_drawable->set_update_func([](easy3d::Model* m, easy3d::Drawable* d) {
            // downcast to MeshObject for access to facesAsFlatList
            Easy3DMeshGraphicsObject* mo = dynamic_cast<Easy3DMeshGraphicsObject*>(m);
            if (mo)
            {
                // update the vertex buffer and element buffer
                d->update_vertex_buffer(mo->points(), true);
                d->update_element_buffer(mo->edgesAsFlatList());
            }
        });
    }
}

Easy3DMeshGraphicsObject::~Easy3DMeshGraphicsObject()
{

}


void Easy3DMeshGraphicsObject::update()
{
    // update the vertex cache, which is what the renderer uses to update the vertex positions
    _updateVertexCache();
    // then call update on the renderer, which will invoke the Drawable update functions
    renderer()->update();
}


std::vector<unsigned int> Easy3DMeshGraphicsObject::facesAsFlatList() const
{
    // each face (triangle) has 3 vertices
    std::vector<unsigned int> faces_flat_list;
    faces_flat_list.reserve(_mesh->numFaces()*3);

    bool has_draw_property = _mesh->hasFaceProperty<bool>("draw");

    if (has_draw_property)
    {
        const std::vector<bool>& draw_face = _mesh->getFaceProperty<bool>("draw").properties();
        // iterate through faces and add them to 1D list
        for (const auto& i : _mesh->faces().validIndices())
        {
            if (!draw_face[i])
                continue;
            
            const Vec3i& face = _mesh->face(i); 
            faces_flat_list.insert(faces_flat_list.end(), {static_cast<unsigned>(face(0)), static_cast<unsigned>(face(1)), static_cast<unsigned>(face(2))});
        }
    }
    else
    {
        // iterate through faces and add them to 1D list
        for (const auto& face : _mesh->faces())
        {
            faces_flat_list.insert(faces_flat_list.end(), {static_cast<unsigned>(face(0)), static_cast<unsigned>(face(1)), static_cast<unsigned>(face(2))});
        }
    }
    

    return faces_flat_list;
}


std::vector<unsigned int> Easy3DMeshGraphicsObject::edgesAsFlatList() const
{
    // TODO: filter duplicate edges
    std::vector<unsigned int> edges_flat_list;
    edges_flat_list.reserve(_mesh->numFaces()*6);

    bool has_draw_property = _mesh->hasFaceProperty<bool>("draw");

    if (has_draw_property)
    {
        const std::vector<bool>& draw_face = _mesh->getFaceProperty<bool>("draw").properties();
        // iterate through faces and add them to 1D list
        for (const auto& i : _mesh->faces().validIndices())
        {
            if (!draw_face[i])
                continue;
            
            const Vec3i& face = _mesh->face(i); 
            edges_flat_list.insert(edges_flat_list.end(), {static_cast<unsigned>(face(0)), static_cast<unsigned>(face(1)),
                static_cast<unsigned>(face(1)), static_cast<unsigned>(face(2)),
                static_cast<unsigned>(face(0)), static_cast<unsigned>(face(2))});
        }
    }
    else
    {
        // iterate through faces and add each edge to 1D list
        for (const auto& face : _mesh->faces())
        {
            edges_flat_list.insert(edges_flat_list.end(), {static_cast<unsigned>(face(0)), static_cast<unsigned>(face(1)),
                static_cast<unsigned>(face(1)), static_cast<unsigned>(face(2)),
                static_cast<unsigned>(face(0)), static_cast<unsigned>(face(2))});
        }
    }

    return edges_flat_list;
}


void Easy3DMeshGraphicsObject::_updateVertexCache()
{
    if (!_mesh)
        return;

    // make sure the vertex cache is big enough for all the vertices
    if(_vertex_cache.size() != static_cast<unsigned>(_mesh->numVertices()))
    {
        _vertex_cache.resize(_mesh->numVertices());
    }

    // loop through and update each vertex in the cache
    for (const auto& i : _mesh->vertices().validIndices())
    {
        const Vec3r& v = _mesh->vertex(i);
        _vertex_cache.at(i) = (easy3d::vec3(v[0], v[1], v[2]));
    }
}


} // namespace Graphics