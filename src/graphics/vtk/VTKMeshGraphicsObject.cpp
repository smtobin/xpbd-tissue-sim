#include "graphics/vtk/VTKMeshGraphicsObject.hpp"
#include "graphics/vtk/VTKUtils.hpp"

#include "common/colors.hpp"

#include <vtkPolyDataMapper.h>
#include <vtkPointData.h>
#include <vtkExtractEdges.h>

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
    _vtk_poly_data = vtkSmartPointer<vtkPolyData>::New();

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

    _vtk_poly_data->SetPoints(vtk_points);
    _vtk_poly_data->SetPolys(vtk_faces);

    
    // if (render_config.drawEdges())
    // {
        
    //     vtkNew<vtkExtractEdges> extract_edges;
    //     extract_edges->SetInputData(_vtk_poly_data);
    //     extract_edges->Update();

    //     vtkNew<vtkPolyDataMapper> mapper;
    //     mapper->SetInputConnection(extract_edges->GetOutputPort());

    //     _edges_vtk_actor = vtkSmartPointer<vtkActor>::New();
    //     _edges_vtk_actor->SetMapper(mapper);

    //     _edges_vtk_actor->GetProperty()->SetColor(0.0, 0.0, 0.0);
    // }

    // if (render_config.drawFaces())
    // {
    //     _face_mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    //     if (render_config.smoothNormals())
    //     {
    //         // smooth normals
    //         _normals_generator = vtkSmartPointer<vtkPolyDataNormals>::New();
    //         _normals_generator->SetInputData(_vtk_poly_data);
    //         _normals_generator->SetFeatureAngle(30.0);
    //         _normals_generator->SplittingOff();
    //         // normal_generator->ConsistencyOn();
    //         _normals_generator->ComputePointNormalsOn();
    //         _normals_generator->ComputeCellNormalsOff();
    //         _normals_generator->Update();

    //         // vtkNew<vtkPolyDataTangents> tangents;
    //         // tangents->SetInputConnection(normal_generator->GetOutputPort());
    //         // tangents->Update();

    //         _face_mapper->SetInputConnection(_normals_generator->GetOutputPort());
    //     }
    //     else
    //     {
    //         _face_mapper->SetInputData(_vtk_poly_data);
    //     }
        
    //     _faces_vtk_actor = vtkSmartPointer<vtkActor>::New();
    //     _faces_vtk_actor->SetMapper(_face_mapper);

    //     VTKUtils::setupActorFromRenderConfig(_faces_vtk_actor.Get(), render_config);

    //     // if the config file specifies multiple colors, and the mesh has the "class" vertex attribute
    //     // then we can assign different colors to vertices based on their class
    //     if (render_config.colors().has_value() && mesh->hasVertexProperty<int>("class"))
    //     {
    //         // set colors for each section of the mesh
    //         vtkNew<vtkUnsignedCharArray> colors;
    //         colors->SetNumberOfComponents(3);
    //         colors->SetName("Colors");

    //         std::vector<Vec3r> colors_f = render_config.colors().value();
    //         const Geometry::MeshProperty<int>& vert_class_prop = mesh->getVertexProperty<int>("class");
    //         for (int i = 0; i < mesh->numVertices(); i++)
    //         {
    //             int vert_class = vert_class_prop.get(i);

    //             // make sure the config file specifies enough colors
    //             if (static_cast<unsigned>(vert_class) >= colors_f.size())
    //             {
    //                 std::cout << KYEL << BOLD << "WARNING" << RST << KYEL << ": Only " << colors_f.size() << " colors were specified, but vertex " << i <<
    //                 " has class " << vert_class << ". (Specify more colors in the config file)" << RST << std::endl;
    //             }

    //             Vec3r color_f = colors_f[vert_class];
    //             unsigned char color[3];
    //             color[0] = static_cast<unsigned char>(color_f[0] * 255);
    //             color[1] = static_cast<unsigned char>(color_f[1] * 255);
    //             color[2] = static_cast<unsigned char>(color_f[2] * 255);

    //             colors->InsertNextTypedTuple(color);
    //         }

    //         _vtk_poly_data->GetPointData()->SetScalars(colors);
    //     }
    // }
    
}

void VTKMeshGraphicsObject::_rebuildPolyData()
{
    std::cout << "REBUILDING POLY DATA!" << std::endl;
    // Create fresh polydata
    vtkNew<vtkPoints> points;
    points->SetNumberOfPoints(_mesh->vertices().totalSize());
    
    for (int i = 0; i < _mesh->vertices().totalSize(); i++)
    {
        const Vec3r& vert = _mesh->vertices()[i];
        points->SetPoint(i, vert[0], vert[1], vert[2]);
    }
    
    vtkNew<vtkCellArray> faces;
    faces->Allocate(_mesh->numFaces(), _mesh->numFaces() * 3);
    
    for (const auto& face : _mesh->faces())
    {
        vtkIdType triangle[3] = {face[0], face[1], face[2]};
        faces->InsertNextCell(3, triangle);
    }
    
    // Clear and rebuild polydata
    vtkNew<vtkPolyData> new_poly_data;
    new_poly_data->SetPoints(points);
    new_poly_data->SetPolys(faces);
    new_poly_data->BuildCells();
    new_poly_data->BuildLinks();

    _vtk_poly_data->ShallowCopy(new_poly_data);
    _vtk_poly_data->Modified();
    
    // _setColors();
}


void VTKMeshGraphicsObject::_setFaces()
{
    vtkNew<vtkCellArray> new_faces;
    new_faces->Allocate(_mesh->numFaces(), _mesh->numFaces() * 3);
    
    int max_vertex_index = -1;

    for (const auto& face : _mesh->faces())
    {
        vtkIdType triangle[3] = {face[0], face[1], face[2]};

        // Track the maximum vertex index
        max_vertex_index = std::max(max_vertex_index, 
                                   std::max({(int)face[0], (int)face[1], (int)face[2]}));

        new_faces->InsertNextCell(3, triangle);
    }

    // VALIDATION: Check if faces reference vertices that don't exist
    int num_points = _vtk_poly_data->GetNumberOfPoints();
    // if (max_vertex_index >= num_points)
    // {
        std::cerr << "Face references vertex " << max_vertex_index 
                  << " and " << num_points << " points exist!" << std::endl;
        std::cerr << "Mesh reports " << _mesh->vertices().totalSize() 
                  << " total vertices" << std::endl;
        // abort();
    // }

    _vtk_poly_data->DeleteCells();
    _vtk_poly_data->DeleteLinks();
    
    _vtk_poly_data->SetPolys(new_faces);

    _vtk_poly_data->Modified();
    
}

void VTKMeshGraphicsObject::_setVertices()
{
    // create points
    vtkPoints* points = _vtk_poly_data->GetPoints();

    // _vtk_poly_data->GetPointData()->Initialize();
    // points->SetNumberOfPoints(_mesh->vertices().totalSize());

    int vtk_index = 0;
    // for (const auto& vert : _mesh->vertices())
    for (int i = 0; i < _mesh->vertices().totalSize(); i++)
    {
        const Vec3r& vert = _mesh->vertices()[i];
        points->SetPoint(i, vert.data());

        if (!_mesh->vertexValid(i))
            std::cout << "vertex " << i << " not valid!" << std::endl;
    }
    points->Modified();
}

void VTKMeshGraphicsObject::_setColors()
{
    if (!_mesh->hasVertexProperty<Real>("voltage"))
        return;

    // set colors for each section of the mesh
    vtkNew<vtkUnsignedCharArray> colors;
    colors->SetNumberOfComponents(3);
    colors->SetName("Colors");

    const Geometry::MeshProperty<Real>& temp_prop = _mesh->getVertexProperty<Real>("voltage");
    for (const auto& vert_index : _mesh->vertices().validIndices())
    {
        Real temp = temp_prop.get(vert_index);

        // for now, 0 = blue and 100 = red
        unsigned char color[3];
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

    _vtk_poly_data->GetPointData()->SetScalars(colors);
        
}

void VTKMeshGraphicsObject::update() 
{
    
    int old_num_points = _vtk_poly_data->GetNumberOfPoints();
    int old_num_cells = _vtk_poly_data->GetNumberOfCells();
    
    bool topology_changed = (_mesh->vertices().totalSize() != old_num_points) ||
                           (_mesh->numFaces() != old_num_cells);

    
    // _setColors();

    if (topology_changed)
    {
        _rebuildPolyData();
    }
    else
    {
        _setVertices();
    }
    
}

} // namespace Graphics

