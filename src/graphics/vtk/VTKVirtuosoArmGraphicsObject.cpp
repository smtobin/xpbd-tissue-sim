#include "graphics/vtk/VTKVirtuosoArmGraphicsObject.hpp"
#include "graphics/vtk/VTKUtils.hpp"

#include <vtkPolyDataMapper.h>
#include <vtkPolyDataNormals.h>
#include <vtkPointData.h>

#include <vtkTriangle.h>
#include <vtkPolygon.h>
#include <vtkQuad.h>
#include <vtkCellArray.h>
#include <vtkFloatArray.h>
#include <vtkProperty.h>

#include <vtkTexture.h>
#include <vtkTriangleFilter.h>
#include <vtkPolyDataTangents.h>
#include <vtkPolyDataMapper.h>
#include <vtkPNGReader.h>
#include <vtkCleanPolyData.h>
#include <vtkImageData.h>

#include <vtkNew.h>

namespace Graphics
{

VTKVirtuosoArmGraphicsObject::VTKVirtuosoArmGraphicsObject(const std::string& name, const Sim::VirtuosoArm* arm, const Config::ObjectRenderConfig& render_config)
    : VirtuosoArmGraphicsObject(name, arm)
{
    _vtk_poly_data = vtkSmartPointer<vtkPolyData>::New();

    _generateInitialPolyData();

    vtkNew<vtkPolyDataMapper> data_mapper;
    if (render_config.smoothNormals())
    {
        // smooth normals
        vtkNew<vtkPolyDataNormals> normal_generator;
        normal_generator->SetInputData(_vtk_poly_data);
        normal_generator->SetFeatureAngle(30.0);
        normal_generator->SplittingOff();
        normal_generator->ConsistencyOn();
        normal_generator->ComputePointNormalsOn();
        normal_generator->ComputeCellNormalsOff();
        normal_generator->Update();

        data_mapper->SetInputConnection(normal_generator->GetOutputPort());
    }
    else
    {
        data_mapper->SetInputData(_vtk_poly_data);
    }
    
    _vtk_actor = vtkSmartPointer<vtkActor>::New();
    _vtk_actor->SetMapper(data_mapper);

    VTKUtils::setupActorFromRenderConfig(_vtk_actor.Get(), render_config);
}

void VTKVirtuosoArmGraphicsObject::updateGraphicsBuffers()
{
    _updatePolyData();
}

void VTKVirtuosoArmGraphicsObject::_generateInitialPolyData()
{
    const Sim::VirtuosoArm::OuterTubeFramesArray& ot_frames = _virtuoso_arm->outerTubeFrames();
    const Sim::VirtuosoArm::InnerTubeFramesArray& it_frames = _virtuoso_arm->innerTubeFrames();

    vtkNew<vtkPoints> vtk_points;

    // make an "outer tube" circle in the XY plane
    Real ot_r = _virtuoso_arm->outerTubeOuterDiameter() / 2.0;
    std::array<Vec3r, _OT_TUBULAR_RES> ot_circle_pts;
    for (int i = 0; i < _OT_TUBULAR_RES; i++)
    {
        Real angle = i * 2 * M_PI / _OT_TUBULAR_RES;
        Vec3r circle_pt(ot_r * std::cos(angle), ot_r * std::sin(angle), 0.0);
        ot_circle_pts[i] = circle_pt;
    }

    // make an "inner tube" circle in the XY plane
    Real it_r = _virtuoso_arm->innerTubeOuterDiameter() / 2.0;
    std::array<Vec3r, _IT_TUBULAR_RES> it_circle_pts;
    for (int i = 0; i < _IT_TUBULAR_RES; i++)
    {
        Real angle = i * 2 * M_PI / _IT_TUBULAR_RES;
        Vec3r circle_pt(it_r * std::cos(angle), it_r * std::sin(angle), 0.0);
        it_circle_pts[i] = circle_pt;
    }

    // outer tube
    for (unsigned fi = 0; fi < ot_frames.size(); fi++)
    {
        const Mat3r rot_mat = ot_frames[fi].transform().rotMat();
        const Vec3r translation = ot_frames[fi].transform().translation();
        for (unsigned pi = 0; pi < ot_circle_pts.size(); pi++)
        {
            Vec3r transformed_pt = rot_mat * ot_circle_pts[pi] + translation;
            vtk_points->InsertNextPoint(transformed_pt[0], transformed_pt[1], transformed_pt[2]);
        }
    }

    vtk_points->InsertNextPoint(ot_frames[0].origin()[0], ot_frames[0].origin()[1], ot_frames[0].origin()[2]);
    vtk_points->InsertNextPoint(ot_frames.back().origin()[0], ot_frames.back().origin()[1], ot_frames.back().origin()[2]);

    // inner tube
    for (unsigned fi = 0; fi < it_frames.size(); fi++)
    {
        const Mat3r rot_mat = it_frames[fi].transform().rotMat();
        const Vec3r translation = it_frames[fi].transform().translation();
        for (unsigned pi = 0; pi < it_circle_pts.size(); pi++)
        {
            Vec3r transformed_pt = rot_mat * it_circle_pts[pi] + translation;
            vtk_points->InsertNextPoint(transformed_pt[0], transformed_pt[1], transformed_pt[2]);
        }
    }

    vtk_points->InsertNextPoint(it_frames[0].origin()[0], it_frames[0].origin()[1], it_frames[0].origin()[2]);
    vtk_points->InsertNextPoint(it_frames.back().origin()[0], it_frames.back().origin()[1], it_frames.back().origin()[2]);
    
    
    int num_ot_vertices = (ot_frames.size() * _OT_TUBULAR_RES + 2); // number of vertices for the outer tube
    int num_it_vertices = (it_frames.size() * _IT_TUBULAR_RES + 2); // number of vertices for the inner tube

    // add face information
    vtkNew<vtkCellArray> vtk_faces;
    // outer tube faces
    for (unsigned fi = 0; fi < ot_frames.size()-1; fi++)
    {
        for (int ti = 0; ti < _OT_TUBULAR_RES; ti++)
        {
            const int v1 = fi*_OT_TUBULAR_RES + ti;
            const int v2 = (ti != _OT_TUBULAR_RES-1) ? v1 + 1 : fi*_OT_TUBULAR_RES;
            const int v3 = v2 + _OT_TUBULAR_RES;
            const int v4 = v1 + _OT_TUBULAR_RES;

            vtkNew<vtkTriangle> tri1;
            tri1->GetPointIds()->SetId(0,v1);
            tri1->GetPointIds()->SetId(1,v2);
            tri1->GetPointIds()->SetId(2,v3);

            vtkNew<vtkTriangle> tri2;
            tri2->GetPointIds()->SetId(0,v1);
            tri2->GetPointIds()->SetId(1,v3);
            tri2->GetPointIds()->SetId(2,v4);
            
            vtk_faces->InsertNextCell(tri1);
            vtk_faces->InsertNextCell(tri2);
        }
    }

    // end cap faces
    for (int ti = 0; ti < _OT_TUBULAR_RES; ti++)
    {
        const int v1 = num_ot_vertices - 2;
        const int v2 = (ti != _OT_TUBULAR_RES-1) ? ti + 1 : 0;
        const int v3 = ti;
        
        vtkNew<vtkTriangle> tri1;
        tri1->GetPointIds()->SetId(0,v1);
        tri1->GetPointIds()->SetId(1,v2);
        tri1->GetPointIds()->SetId(2,v3);
        vtk_faces->InsertNextCell(tri1);
    }

    for (int ti = 0; ti < _OT_TUBULAR_RES; ti++)
    {
        const int v1 = num_ot_vertices - 1;
        const int v2 = (ot_frames.size()-1)*_OT_TUBULAR_RES + ti;
        const int v3 = (ti != _OT_TUBULAR_RES-1) ? v2 + 1 : (ot_frames.size()-1)*_OT_TUBULAR_RES;
        vtkNew<vtkTriangle> tri1;
        tri1->GetPointIds()->SetId(0,v1);
        tri1->GetPointIds()->SetId(1,v2);
        tri1->GetPointIds()->SetId(2,v3);
        vtk_faces->InsertNextCell(tri1);
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
            
            vtkNew<vtkTriangle> tri1;
            tri1->GetPointIds()->SetId(0,num_ot_vertices + v1);
            tri1->GetPointIds()->SetId(1,num_ot_vertices + v2);
            tri1->GetPointIds()->SetId(2,num_ot_vertices + v3);

            vtkNew<vtkTriangle> tri2;
            tri2->GetPointIds()->SetId(0,num_ot_vertices + v1);
            tri2->GetPointIds()->SetId(1,num_ot_vertices + v3);
            tri2->GetPointIds()->SetId(2,num_ot_vertices + v4);
            
            vtk_faces->InsertNextCell(tri1);
            vtk_faces->InsertNextCell(tri2);
        }
    }

    // end cap faces
    for (int ti = 0; ti < _IT_TUBULAR_RES; ti++)
    {
        const int v1 = num_it_vertices - 2;
        const int v2 = (ti != _IT_TUBULAR_RES-1) ? ti + 1 : 0;
        const int v3 = ti;
        
        vtkNew<vtkTriangle> tri1;
        tri1->GetPointIds()->SetId(0,num_ot_vertices+v1);
        tri1->GetPointIds()->SetId(1,num_ot_vertices+v2);
        tri1->GetPointIds()->SetId(2,num_ot_vertices+v3);
        vtk_faces->InsertNextCell(tri1);
    }

    for (int ti = 0; ti < _IT_TUBULAR_RES; ti++)
    {
        const int v1 = num_it_vertices - 1;
        const int v2 = (it_frames.size()-1)*_IT_TUBULAR_RES + ti;
        const int v3 = (ti != _IT_TUBULAR_RES-1) ? v2 + 1 : (it_frames.size()-1)*_IT_TUBULAR_RES;
        
        vtkNew<vtkTriangle> tri1;
        tri1->GetPointIds()->SetId(0,num_ot_vertices+v1);
        tri1->GetPointIds()->SetId(1,num_ot_vertices+v2);
        tri1->GetPointIds()->SetId(2,num_ot_vertices+v3);
        vtk_faces->InsertNextCell(tri1);
    }

    _vtk_poly_data->SetPoints(vtk_points);
    _vtk_poly_data->SetPolys(vtk_faces);

}

void VTKVirtuosoArmGraphicsObject::_updatePolyData()
{
    RenderInfo* rinfo = _latest_rinfo.load(std::memory_order_acquire);

    const Sim::VirtuosoArm::OuterTubeFramesArray& ot_frames = rinfo->ot_frames;
    const Sim::VirtuosoArm::InnerTubeFramesArray& it_frames = rinfo->it_frames;

    vtkPoints* vtk_points = _vtk_poly_data->GetPoints();

    // make an "outer tube" circle in the XY plane
    Real ot_r = rinfo->ot_outer_diameter / 2.0;
    std::array<Vec3r, _OT_TUBULAR_RES> ot_circle_pts;
    for (int i = 0; i < _OT_TUBULAR_RES; i++)
    {
        Real angle = i * 2 * M_PI / _OT_TUBULAR_RES;
        Vec3r circle_pt(ot_r * std::cos(angle), ot_r * std::sin(angle), 0.0);
        ot_circle_pts[i] = circle_pt;
    }

    // make an "inner tube" circle in the XY plane
    Real it_r = rinfo->it_outer_diameter / 2.0;
    std::array<Vec3r, _IT_TUBULAR_RES> it_circle_pts;
    for (int i = 0; i < _IT_TUBULAR_RES; i++)
    {
        Real angle = i * 2 * M_PI / _IT_TUBULAR_RES;
        Vec3r circle_pt(it_r * std::cos(angle), it_r * std::sin(angle), 0.0);
        it_circle_pts[i] = circle_pt;
    } 

    // outer tube
    for (unsigned fi = 0; fi < ot_frames.size(); fi++)
    {
        const Mat3r rot_mat = ot_frames[fi].transform().rotMat();
        const Vec3r translation = ot_frames[fi].transform().translation();
        for (unsigned pi = 0; pi < ot_circle_pts.size(); pi++)
        {
            Vec3r transformed_pt = rot_mat * ot_circle_pts[pi] + translation;
            vtk_points->SetPoint(fi*_OT_TUBULAR_RES + pi, transformed_pt[0], transformed_pt[1], transformed_pt[2]);
        }
    }

    vtk_points->SetPoint(ot_frames.size()*_OT_TUBULAR_RES, ot_frames[0].origin()[0], ot_frames[0].origin()[1], ot_frames[0].origin()[2]);
    vtk_points->SetPoint(ot_frames.size()*_OT_TUBULAR_RES+1, ot_frames.back().origin()[0], ot_frames.back().origin()[1], ot_frames.back().origin()[2]);

    // inner tube
    int it_index_offset = ot_frames.size()*_OT_TUBULAR_RES + 2;
    for (unsigned fi = 0; fi < it_frames.size(); fi++)
    {
        const Mat3r rot_mat = it_frames[fi].transform().rotMat();
        const Vec3r translation = it_frames[fi].transform().translation();
        for (unsigned pi = 0; pi < it_circle_pts.size(); pi++)
        {
            Vec3r transformed_pt = rot_mat * it_circle_pts[pi] + translation;
            vtk_points->SetPoint(it_index_offset + fi*_IT_TUBULAR_RES + pi, transformed_pt[0], transformed_pt[1], transformed_pt[2]);
        }
    }
    

    vtk_points->SetPoint(it_index_offset + it_frames.size()*_IT_TUBULAR_RES, it_frames[0].origin()[0], it_frames[0].origin()[1], it_frames[0].origin()[2]);
    vtk_points->SetPoint(it_index_offset + it_frames.size()*_IT_TUBULAR_RES+1, it_frames.back().origin()[0], it_frames.back().origin()[1], it_frames.back().origin()[2]);
    
    vtk_points->Modified();
}

} // namespace Graphics