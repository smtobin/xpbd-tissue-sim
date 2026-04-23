#include "graphics/easy3d/Easy3DVirtuosoArmGraphicsObject.hpp"

#include "geometry/TransformationMatrix.hpp"
#include "geometry/CoordinateFrame.hpp"

#include <easy3d/algo/surface_mesh_factory.h>
#include <easy3d/renderer/renderer.h>
#include <easy3d/renderer/drawable_points.h>
#include <easy3d/renderer/drawable_triangles.h>

namespace Graphics
{

Easy3DVirtuosoArmGraphicsObject::Easy3DVirtuosoArmGraphicsObject(const std::string& name, const Sim::VirtuosoArm* virtuoso_arm)
    : VirtuosoArmGraphicsObject(name, virtuoso_arm)
{
    _generateInitialMesh();

    std::shared_ptr<easy3d::Renderer> renderer = std::make_shared<easy3d::Renderer>(&_e3d_mesh, true);
    _e3d_mesh.set_renderer(renderer);

    // std::shared_ptr<easy3d::Renderer> renderer = std::make_shared<easy3d::Renderer>(this, true);

    // create a PointsDrawable for the points of the tetrahedral mesh
    // easy3d::PointsDrawable* points_drawable = renderer->add_points_drawable("vertices");
    // // specify the update function for the points
    // points_drawable->set_update_func([](easy3d::Model* m, easy3d::Drawable* d) {
    //     // update the vertex buffer with the vertices of the mesh
    //     d->update_vertex_buffer(m->points(), true);
    // });

    // set a uniform color for the mesh
    // easy3d::vec4 color(0, 0, 0, 0);
    // points_drawable->set_uniform_coloring(color);

    set_renderer(renderer);

    

    // color the mesh gray
    for (auto& tri_drawable : renderer->triangles_drawables())
    {
        easy3d::vec4 color(0.7, 0.7, 0.7, 1.0);
        tri_drawable->set_uniform_coloring(color);
    }
}

void Easy3DVirtuosoArmGraphicsObject::updateGraphicsBuffers()
{
    _updateMesh();
    renderer()->update();
}

void Easy3DVirtuosoArmGraphicsObject::_generateInitialMesh()
{
    const Sim::VirtuosoArm::OuterTubeFramesArray& ot_frames = _virtuoso_arm->outerTubeFrames();
    const Sim::VirtuosoArm::InnerTubeFramesArray& it_frames = _virtuoso_arm->innerTubeFrames();

    int num_ot_vertices = (ot_frames.size() * _OT_TUBULAR_RES + 2);
    int num_it_vertices = (it_frames.size() * _IT_TUBULAR_RES + 2);

    const Real ot_r = _virtuoso_arm->outerTubeOuterDiameter() / 2.0;
    const Real it_r = _virtuoso_arm->innerTubeOuterDiameter() / 2.0;
    const Real ot_trans = _virtuoso_arm->outerTubeTranslation();
    const Real it_trans = _virtuoso_arm->innerTubeTranslation();
    for (unsigned fi = 0; fi < ot_frames.size(); fi++)
    {
        for (int i = 0; i < _OT_TUBULAR_RES; i++)
        {
            Real angle = i * 2 * M_PI / _OT_TUBULAR_RES;
            _e3d_mesh.add_vertex(easy3d::vec3(ot_r * std::cos(angle), ot_r * std::sin(angle), fi*ot_trans/(ot_frames.size() - 1)));
        }
    }

    _e3d_mesh.add_vertex(easy3d::vec3(0.0, 0.0, 0.0));
    _e3d_mesh.add_vertex(easy3d::vec3(0.0, 0.0, ot_trans));


    for (unsigned fi = 0; fi < it_frames.size(); fi++)
    {
        for (int i = 0; i < _IT_TUBULAR_RES; i++)
        {
            Real angle = i* 2 * M_PI / _IT_TUBULAR_RES;
            _e3d_mesh.add_vertex(easy3d::vec3(it_r * std::cos(angle), it_r * std::sin(angle), ot_trans + fi*it_trans/(it_frames.size() - 1)));
        }
    }

    _e3d_mesh.add_vertex(easy3d::vec3(0.0, 0.0, ot_trans));
    _e3d_mesh.add_vertex(easy3d::vec3(0.0, 0.0, ot_trans + it_trans));


    // add face information

    // outer tube faces
    for (unsigned fi = 0; fi < ot_frames.size()-1; fi++)
    {
        for (int ti = 0; ti < _OT_TUBULAR_RES; ti++)
        {
            const int v1 = fi*_OT_TUBULAR_RES + ti;
            const int v2 = (ti != _OT_TUBULAR_RES-1) ? v1 + 1 : fi*_OT_TUBULAR_RES;
            const int v3 = v2 + _OT_TUBULAR_RES;
            const int v4 = v1 + _OT_TUBULAR_RES;
            _e3d_mesh.add_quad(easy3d::SurfaceMesh::Vertex(v1), easy3d::SurfaceMesh::Vertex(v2), easy3d::SurfaceMesh::Vertex(v3), easy3d::SurfaceMesh::Vertex(v4));
        }
    }

    // end cap faces
    for (int ti = 0; ti < _OT_TUBULAR_RES; ti++)
    {
        const int v1 = num_ot_vertices - 2;
        const int v2 = (ti != _OT_TUBULAR_RES-1) ? ti + 1 : 0;
        const int v3 = ti;
        _e3d_mesh.add_triangle(easy3d::SurfaceMesh::Vertex(v1), easy3d::SurfaceMesh::Vertex(v2), easy3d::SurfaceMesh::Vertex(v3));
    }

    for (int ti = 0; ti < _OT_TUBULAR_RES; ti++)
    {
        const int v1 = num_ot_vertices - 1;
        const int v2 = (ot_frames.size()-1)*_OT_TUBULAR_RES + ti;
        const int v3 = (ti != _OT_TUBULAR_RES-1) ? v2 + 1 : (ot_frames.size()-1)*_OT_TUBULAR_RES;
        _e3d_mesh.add_triangle(easy3d::SurfaceMesh::Vertex(v1), easy3d::SurfaceMesh::Vertex(v2), easy3d::SurfaceMesh::Vertex(v3));
    }


    // inner tube faces
    for (unsigned fi = 0; fi < it_frames.size()-1; fi++)
    {
        for (int ti = 0; ti < _IT_TUBULAR_RES; ti++)
        {
            const int v1 = fi*_IT_TUBULAR_RES + ti;
            const int v2 = (ti != _IT_TUBULAR_RES-1) ? v1 + 1 : fi*_IT_TUBULAR_RES;
            const int v3 = v2 + _IT_TUBULAR_RES;
            const int v4 = v1 + _IT_TUBULAR_RES;
            _e3d_mesh.add_quad(easy3d::SurfaceMesh::Vertex(num_ot_vertices+v1), 
                easy3d::SurfaceMesh::Vertex(num_ot_vertices+v2), 
                easy3d::SurfaceMesh::Vertex(num_ot_vertices+v3), 
                easy3d::SurfaceMesh::Vertex(num_ot_vertices+v4));
        }
    }

    // end cap faces
    for (int ti = 0; ti < _IT_TUBULAR_RES; ti++)
    {
        const int v1 = num_it_vertices - 2;
        const int v2 = (ti != _IT_TUBULAR_RES-1) ? ti + 1 : 0;
        const int v3 = ti;
        _e3d_mesh.add_triangle(easy3d::SurfaceMesh::Vertex(num_ot_vertices+v1), easy3d::SurfaceMesh::Vertex(num_ot_vertices+v2), easy3d::SurfaceMesh::Vertex(num_ot_vertices+v3));
    }

    for (int ti = 0; ti < _IT_TUBULAR_RES; ti++)
    {
        const int v1 = num_it_vertices - 1;
        const int v2 = (it_frames.size()-1)*_IT_TUBULAR_RES + ti;
        const int v3 = (ti != _IT_TUBULAR_RES-1) ? v2 + 1 : (it_frames.size()-1)*_IT_TUBULAR_RES;
        _e3d_mesh.add_triangle(easy3d::SurfaceMesh::Vertex(num_ot_vertices+v1), easy3d::SurfaceMesh::Vertex(num_ot_vertices+v2), easy3d::SurfaceMesh::Vertex(num_ot_vertices+v3));
    }
    
}

void Easy3DVirtuosoArmGraphicsObject::_updateMesh()
{
    const Sim::VirtuosoArm::OuterTubeFramesArray& ot_frames = _virtuoso_arm->outerTubeFrames();
    const Sim::VirtuosoArm::InnerTubeFramesArray& it_frames = _virtuoso_arm->innerTubeFrames();

    // make an "outer tube" circle in the XY plane
    Real ot_r = _virtuoso_arm->outerTubeOuterDiameter() / 2.0;
    std::array<Vec3r, _OT_TUBULAR_RES> ot_circle_pts;
    for (int i = 0; i < _OT_TUBULAR_RES; i++)
    {
        Real angle = i * 2 * 3.1415 / _OT_TUBULAR_RES;
        Vec3r circle_pt(ot_r * std::cos(angle), ot_r * std::sin(angle), 0.0);
        ot_circle_pts[i] = circle_pt;
    }

    // make an "inner tube" circle in the XY plane
    Real it_r = _virtuoso_arm->innerTubeOuterDiameter() / 2.0;
    std::array<Vec3r, _IT_TUBULAR_RES> it_circle_pts;
    for (int i = 0; i < _IT_TUBULAR_RES; i++)
    {
        Real angle = i * 2 * 3.1415 / _IT_TUBULAR_RES;
        Vec3r circle_pt(it_r * std::cos(angle), it_r * std::sin(angle), 0.0);
        it_circle_pts[i] = circle_pt;
    } 

    // outer tube
    for (unsigned fi = 0; fi < ot_frames.size(); fi++)
    {
        const Mat3r rot_mat = ot_frames[fi].transform().rotMat();
        const Vec3r translation = ot_frames[fi].transform().translation();
        // std::cout << "ot_trans " << fi << ": " << translation[0] << ", " << translation[1] << ", " << translation[2] << std::endl;
        // std::cout << "ot_rot " << fi << ":\n" << rot_mat << std::endl;
        for (unsigned pi = 0; pi < ot_circle_pts.size(); pi++)
        {
            Vec3r transformed_pt = rot_mat * ot_circle_pts[pi] + translation;
            _e3d_mesh.points()[fi*_OT_TUBULAR_RES + pi] = easy3d::vec3(transformed_pt[0], transformed_pt[1], transformed_pt[2]);
        }
    }

    _e3d_mesh.points()[ot_frames.size()*_OT_TUBULAR_RES] = easy3d::vec3(ot_frames[0].origin()[0], ot_frames[0].origin()[1], ot_frames[0].origin()[2]);
    _e3d_mesh.points()[ot_frames.size()*_OT_TUBULAR_RES+1] = easy3d::vec3(ot_frames.back().origin()[0], ot_frames.back().origin()[1], ot_frames.back().origin()[2]);

    // inner tube
    int it_index_offset = ot_frames.size()*_OT_TUBULAR_RES + 2;
    for (unsigned fi = 0; fi < it_frames.size(); fi++)
    {
        const Mat3r rot_mat = it_frames[fi].transform().rotMat();
        const Vec3r translation = it_frames[fi].transform().translation();
        for (unsigned pi = 0; pi < it_circle_pts.size(); pi++)
        {
            Vec3r transformed_pt = rot_mat * it_circle_pts[pi] + translation;
            _e3d_mesh.points()[it_index_offset + fi*_IT_TUBULAR_RES + pi] = easy3d::vec3(transformed_pt[0], transformed_pt[1], transformed_pt[2]);
        }
    }
    

    _e3d_mesh.points()[it_index_offset + it_frames.size()*_IT_TUBULAR_RES] = easy3d::vec3(it_frames[0].origin()[0], it_frames[0].origin()[1], it_frames[0].origin()[2]);
    _e3d_mesh.points()[it_index_offset + it_frames.size()*_IT_TUBULAR_RES+1] = easy3d::vec3(it_frames.back().origin()[0], it_frames.back().origin()[1], it_frames.back().origin()[2]);


    // update face normals
    _e3d_mesh.update_face_normals();
}

} // namespace Graphics