#include "graphics/vtk/VTKMeshGraphicsObject.hpp"
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

VTKMeshGraphicsObject::VTKMeshGraphicsObject(const std::string& name, const Geometry::Mesh* mesh, const Config::ObjectRenderConfig& render_config)
    : MeshGraphicsObject(name, mesh)
{
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
    }

    if (render_config.drawFaces())
    {
        _face_mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
        if (render_config.smoothNormals())
        {
            // smooth normals
            _normals_generator = vtkSmartPointer<vtkPolyDataNormals>::New();
            _normals_generator->SetInputData(_front_poly_data);
            _normals_generator->SetFeatureAngle(30.0);
            _normals_generator->SplittingOff();
            // normal_generator->ConsistencyOn();
            _normals_generator->ComputePointNormalsOn();
            _normals_generator->ComputeCellNormalsOff();
            _normals_generator->Update();

            // vtkNew<vtkPolyDataTangents> tangents;
            // tangents->SetInputConnection(normal_generator->GetOutputPort());
            // tangents->Update();

            _face_mapper->SetInputConnection(_normals_generator->GetOutputPort());
        }
        else
        {
            _face_mapper->SetInputData(_front_poly_data);
        }
        
        _faces_vtk_actor = vtkSmartPointer<vtkActor>::New();
        _faces_vtk_actor->SetMapper(_face_mapper);

        VTKUtils::setupActorFromRenderConfig(_faces_vtk_actor.Get(), render_config);

        // if the config file specifies multiple colors, and the mesh has the "class" vertex attribute
        // then we can assign different colors to vertices based on their class
        if (render_config.colors().has_value() && mesh->hasVertexProperty<int>("class"))
        {
            // set colors for each section of the mesh
            vtkNew<vtkUnsignedCharArray> colors;
            colors->SetNumberOfComponents(3);
            colors->SetName("Colors");

            std::vector<Vec3r> colors_f = render_config.colors().value();
            const Geometry::MeshProperty<int>& vert_class_prop = mesh->getVertexProperty<int>("class");
            for (int i = 0; i < mesh->numVertices(); i++)
            {
                int vert_class = vert_class_prop.get(i);

                // make sure the config file specifies enough colors
                if (static_cast<unsigned>(vert_class) >= colors_f.size())
                {
                    std::cout << KYEL << BOLD << "WARNING" << RST << KYEL << ": Only " << colors_f.size() << " colors were specified, but vertex " << i <<
                    " has class " << vert_class << ". (Specify more colors in the config file)" << RST << std::endl;
                }

                Vec3r color_f = colors_f[vert_class];
                unsigned char color[3];
                color[0] = static_cast<unsigned char>(color_f[0] * 255);
                color[1] = static_cast<unsigned char>(color_f[1] * 255);
                color[2] = static_cast<unsigned char>(color_f[2] * 255);

                colors->InsertNextTypedTuple(color);
            }

            _front_poly_data->GetPointData()->SetScalars(colors);
        }
    }
    
}

void VTKMeshGraphicsObject::_setFaces(const RenderInfo* rmesh)
{
    vtkCellArray* faces = _front_poly_data->GetPolys();
    faces->Reset();
    faces->AllocateExact(rmesh->faces.size(), rmesh->faces.size() * 3);

    vtkIdType vtk_face[3];
    for (const auto& face : rmesh->faces)
    {
        vtk_face[0] = static_cast<vtkIdType>(face[0]);
        vtk_face[1] = static_cast<vtkIdType>(face[1]);
        vtk_face[2] = static_cast<vtkIdType>(face[2]);
        faces->InsertNextCell(3, vtk_face);
    }

    faces->Modified();
}

void VTKMeshGraphicsObject::_setVertices(const RenderInfo* rmesh)
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


void VTKMeshGraphicsObject::updateGraphicsBuffers() 
{

    RenderInfo* rmesh = _latest_rmesh.load(std::memory_order_acquire);

    int old_num_points = _front_poly_data->GetNumberOfPoints();
    int old_num_cells = _front_poly_data->GetNumberOfCells();
    
    bool topology_changed = (rmesh->vertices.totalSize() != old_num_points) ||
                           (rmesh->faces.size() != old_num_cells);

    _setVertices(rmesh);
    
    _setColors(rmesh);

    if (topology_changed)
    {
        _setFaces(rmesh);

        _front_poly_data->BuildCells();
        _front_poly_data->BuildLinks();
    }

    
    _front_poly_data->Modified();

    if (_edge_extractor)
    {
        _edge_extractor->Update();
    }
    
    if (_normals_generator)
    {
        _normals_generator->Update();
    }
    
    
    // if (_normals_generator)
    // {
    //     _normals_generator->Modified();
    //     _normals_generator->Update();
    // }
}

void VTKMeshGraphicsObject::_setColors(const RenderInfo* rmesh)
{
    if (!rmesh->hasVertexProperty<Real>("voltage"))
        return;

    // set colors for each section of the mesh
    vtkNew<vtkUnsignedCharArray> colors;
    colors->SetNumberOfComponents(3);
    colors->SetName("Colors");

    const Geometry::MeshProperty<Real>& temp_prop = rmesh->getVertexProperty<Real>("voltage");
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

