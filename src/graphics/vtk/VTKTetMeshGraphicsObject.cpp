#include "graphics/vtk/VTKTetMeshGraphicsObject.hpp"
#include "graphics/vtk/VTKUtils.hpp"

#include "common/colors.hpp"

#include <vtkPolyDataMapper.h>
#include <vtkPointData.h>

#include <vtkTriangle.h>
#include <vtkPolygon.h>
#include <vtkQuad.h>
#include <vtkCellArray.h>
#include <vtkFloatArray.h>
#include <vtkProperty.h>
#include <vtkCellData.h>

#include <vtkTexture.h>
#include <vtkTriangleFilter.h>
#include <vtkPolyDataTangents.h>
#include <vtkPNGReader.h>
#include <vtkCleanPolyData.h>
#include <vtkImageData.h>

#include <vtkNew.h>

namespace Graphics
{

VTKTetMeshGraphicsObject::VTKTetMeshGraphicsObject(const std::string& name, const Geometry::TetMesh* mesh, const Config::ObjectRenderConfig& render_config)
    : TetMeshGraphicsObject(name, mesh)
{
    _latest_topology_version = mesh->topologyVersion()-1; // hack to trigger an immediate topology update

    _front_poly_data = vtkSmartPointer<vtkPolyData>::New();

    // create points
    vtkNew<vtkPoints> vtk_points;
    for (int i = 0; i < _mesh->vertices().totalSize(); i++)
    {
        // even if the vertex is invalid we still need to insert it so that the faces have proper indexing
        /** TODO: is there a better way? */
        const Vec3r& vert = _mesh->vertices()[i];
        vtk_points->InsertNextPoint(vert[0], vert[1], vert[2]);
    }

    // create faces
    vtkNew<vtkCellArray> vtk_faces;
    for (const auto& face : _mesh->faces())
    {
        vtkNew<vtkTriangle> tri;
        tri->GetPointIds()->SetId(0, face[0]);
        tri->GetPointIds()->SetId(1, face[1]);
        tri->GetPointIds()->SetId(2, face[2]);

        vtk_faces->InsertNextCell(tri);
    }

    _front_poly_data->SetPoints(vtk_points);
    _front_poly_data->SetPolys(vtk_faces);

    _smooth_normals = render_config.smoothNormals();

    if (render_config.drawEdges())
    {
        
        _edge_extractor = vtkSmartPointer<vtkExtractEdges>::New();
        _edge_extractor->SetInputData(_front_poly_data);
        _edge_extractor->Update();

        _edge_mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
        _edge_mapper->SetInputConnection(_edge_extractor->GetOutputPort());

        _edges_vtk_actor = vtkSmartPointer<vtkActor>::New();
        _edges_vtk_actor->SetMapper(_edge_mapper);

        _edges_vtk_actor->GetProperty()->SetColor(0.0, 0.0, 0.0);
        _edges_vtk_actor->GetProperty()->LightingOff();
    }

    if (render_config.drawFaces())
    {
        _face_mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
        _face_mapper->SetInputData(_front_poly_data);
        _face_mapper->ScalarVisibilityOn();
        _face_mapper->SetScalarModeToUseCellData();
        
        _faces_vtk_actor = vtkSmartPointer<vtkActor>::New();
        _faces_vtk_actor->SetMapper(_face_mapper);
        

        VTKUtils::setupActorFromRenderConfig(_faces_vtk_actor.Get(), render_config);

        // extract the colors from the render config, we need to remember them
        _colors = render_config.colors();
        if (render_config.color().has_value())
            _bulk_color = render_config.color().value();
        else if (render_config.colors().has_value())
            _bulk_color = render_config.colors().value()[0];
        else
            _bulk_color = Vec3r(1, 0, 0);

        if (render_config.cutColor().has_value())
            _cut_color = render_config.cutColor().value();
        else
            _cut_color = _bulk_color;


        // if the config file specifies multiple colors, and the mesh has the "class" vertex attribute
        // then we can assign different colors to vertices based on their class
        if (render_config.colors().has_value() && mesh->hasFaceProperty<int>("class"))
        {
            // set colors for each section of the mesh
            vtkNew<vtkUnsignedCharArray> colors;
            colors->SetNumberOfComponents(3);
            colors->SetName("Colors");

            std::vector<Vec3r> colors_f = render_config.colors().value();
            const Geometry::MeshProperty<int>& vert_class_prop = mesh->getVertexProperty<int>("class");
            const Geometry::MeshProperty<int>& face_class_prop = mesh->getFaceProperty<int>("class");
            for (int i = 0; i < mesh->numFaces(); i++)
            {
                int face_class = face_class_prop.get(i);

                // make sure the config file specifies enough colors
                if (static_cast<unsigned>(face_class) >= colors_f.size())
                {
                    std::cout << KYEL << BOLD << "WARNING" << RST << KYEL << ": Only " << colors_f.size() << " colors were specified, but face " << i <<
                    " has class " << face_class << ". (Specify more colors in the config file)" << RST << std::endl;
                }

                Vec3r color_f = colors_f[face_class];
                unsigned char color[3];
                color[0] = static_cast<unsigned char>(color_f[0] * 255);
                color[1] = static_cast<unsigned char>(color_f[1] * 255);
                color[2] = static_cast<unsigned char>(color_f[2] * 255);

                colors->InsertNextTypedTuple(color);
            }

            _front_poly_data->GetCellData()->SetScalars(colors);
        }
    }
    
}

void VTKTetMeshGraphicsObject::_setFaces(const RenderInfo* rmesh)
{
    vtkCellArray* faces = _front_poly_data->GetPolys();

    int total_num_faces = rmesh->faces.size();
    for (const auto& interior : rmesh->interior_faces)
        total_num_faces += interior.size();
        
    faces->Reset();
    faces->AllocateExact(total_num_faces, total_num_faces * 3);

    vtkIdType vtk_face[3];
    for (const auto& face : rmesh->faces)
    {
        vtk_face[0] = static_cast<vtkIdType>(face[0]);
        vtk_face[1] = static_cast<vtkIdType>(face[1]);
        vtk_face[2] = static_cast<vtkIdType>(face[2]);
        faces->InsertNextCell(3, vtk_face);
    }

    for (const auto& interior : rmesh->interior_faces)
    {
        for (const auto& face : interior)
        {
            vtk_face[0] = static_cast<vtkIdType>(face[0]);
            vtk_face[1] = static_cast<vtkIdType>(face[1]);
            vtk_face[2] = static_cast<vtkIdType>(face[2]);
            faces->InsertNextCell(3, vtk_face);
        }
    }

    faces->Modified();
}

void VTKTetMeshGraphicsObject::_setVertices(const RenderInfo* rmesh)
{
    // update points
    vtkPoints* points = _front_poly_data->GetPoints();
    points->SetNumberOfPoints(rmesh->vertices.totalSize());

    for (size_t i = 0; i < rmesh->vertices.totalSize(); i++)
    {
        // p[3*i]     = rmesh->vertices[i][0];
        // p[3*i + 1] = rmesh->vertices[i][1];
        // p[3*i + 2] = rmesh->vertices[i][2];
        points->SetPoint(i, rmesh->vertices[i][0], rmesh->vertices[i][1], rmesh->vertices[i][2]);
    }
    points->Modified();
}

void VTKTetMeshGraphicsObject::_setNormals(const RenderInfo* rmesh)
{
    // copy normals from rmesh buffer into VTK buffer
    vtkNew<vtkFloatArray> normals;
    normals->SetNumberOfComponents(3);
    normals->SetNumberOfTuples(rmesh->vertices.totalSize());
    normals->SetName("Normals");

    float* n = normals->GetPointer(0);

    for (unsigned i = 0; i < rmesh->vertices.totalSize(); i++)
    {
        float* ni = n + 3*i;
        const Vec3r& normal = rmesh->vertex_normals[i];
        ni[0] = static_cast<float>(normal[0]);
        ni[1] = static_cast<float>(normal[1]);
        ni[2] = static_cast<float>(normal[2]);
    }
    _front_poly_data->GetPointData()->SetNormals(normals);
}

void VTKTetMeshGraphicsObject::updateGraphicsBuffers() 
{

    RenderInfo* rmesh = _latest_rmesh.load(std::memory_order_acquire);
    
    bool topology_changed = (rmesh->topology_version != _latest_topology_version);

    _setVertices(rmesh);

    if (_smooth_normals)
        _setNormals(rmesh);

    if (topology_changed)
    {
        _setFaces(rmesh);

        _front_poly_data->BuildCells();
        _front_poly_data->BuildLinks();

        _setColorsForCutSurface(rmesh);

        if (_edge_extractor)
        {
            _edge_extractor->Update();
        }

        _latest_topology_version = rmesh->topology_version;
    }
    
    _front_poly_data->Modified();
}

void VTKTetMeshGraphicsObject::_setColorsForCutSurface(const RenderInfo* rmesh)
{
    if (!rmesh->hasFaceProperty<bool>("on-cut-surface"))
        return;
    
    // set colors for each section of the mesh
    vtkNew<vtkUnsignedCharArray> colors;
    colors->SetNumberOfComponents(3);
    colors->SetName("Colors");

    const Geometry::MeshProperty<bool>& on_cut_surface_prop = rmesh->getFaceProperty<bool>("on-cut-surface");
    for (const auto& face_index : rmesh->faces.validIndices())
    {
        unsigned char color[3];
        bool on_cut_surface = on_cut_surface_prop.get(face_index);

        if (on_cut_surface)
        {
            color[0] = static_cast<unsigned char>(_cut_color[0] * 255);
            color[1] = static_cast<unsigned char>(_cut_color[1] * 255);
            color[2] = static_cast<unsigned char>(_cut_color[2] * 255);
        }
        else
        {
            color[0] = static_cast<unsigned char>(_bulk_color[0] * 255);
            color[1] = static_cast<unsigned char>(_bulk_color[1] * 255);
            color[2] = static_cast<unsigned char>(_bulk_color[2] * 255);
        }

        colors->InsertNextTypedTuple(color);
    }

    for (const auto& interior : rmesh->interior_faces)
    {
        for (const auto& face : interior)
        {
            unsigned char color[3];
            color[0] = static_cast<unsigned char>(_bulk_color[1] * 255);
            color[1] = static_cast<unsigned char>(_bulk_color[0] * 255);
            color[2] = static_cast<unsigned char>(_bulk_color[2] * 255);
            colors->InsertNextTypedTuple(color);
        }
    }

    _front_poly_data->GetCellData()->SetScalars(colors);
}

void VTKTetMeshGraphicsObject::_setColorsForTemperature(const RenderInfo* rmesh)
{
    if (!rmesh->hasVertexProperty<Real>("temperature"))
        return;

    // set colors for each section of the mesh
    vtkNew<vtkUnsignedCharArray> colors;
    colors->SetNumberOfComponents(3);
    colors->SetName("Colors");

    const Geometry::MeshProperty<Real>& temp_prop = rmesh->getVertexProperty<Real>("temperature");
    for (unsigned vert_index = 0; vert_index < _mesh->vertices().totalSize(); vert_index++)
    {
        unsigned char color[3];
        Real temp = temp_prop.get(vert_index);

        // for now, 0 = blue and 100 = red
        // std::cout << temp << std::endl;
        if (temp <= 0)
        {
            color[0] = 0u;
            color[1] = 0u;
            color[2] = 255u;
        }
        else if (temp >= 100)
        {
            color[0] = 255u;
            color[1] = 0u;
            color[2] = 0u;
        }
        else
        {
            Real t = temp / 100.0;
            color[0] = static_cast<unsigned char>(t * 255);
            color[1] = 0u;
            color[2] = static_cast<unsigned char>((1-t) * 255);
        }

        colors->InsertNextTypedTuple(color);
    }
    // std::cout << "\n\n\n\n\n\n" << std::endl;

    _front_poly_data->GetPointData()->SetScalars(colors);
        
}


} // namespace Graphics

