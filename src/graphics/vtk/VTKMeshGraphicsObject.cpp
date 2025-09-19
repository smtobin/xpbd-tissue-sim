#include "graphics/vtk/VTKMeshGraphicsObject.hpp"
#include "graphics/vtk/VTKUtils.hpp"

#include "common/colors.hpp"

#include <vtkPolyDataMapper.h>
#include <vtkPolyDataNormals.h>
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
#include <vtkPolyDataMapper.h>
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
    vtkNew<vtkPoints> vtk_points;
    vtkNew<vtkCellArray> vtk_faces;
    _vtk_poly_data->SetPoints(vtk_points);
    _vtk_poly_data->SetPolys(vtk_faces);

    _setVertices();
    _setFaces();

    vtkNew<vtkPolyDataMapper> mapper;
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

        // vtkNew<vtkPolyDataTangents> tangents;
        // tangents->SetInputConnection(normal_generator->GetOutputPort());
        // tangents->Update();

        mapper->SetInputConnection(normal_generator->GetOutputPort());
    }
    else
    {
        mapper->SetInputData(_vtk_poly_data);
    }
    
    _vtk_actor = vtkSmartPointer<vtkActor>::New();
    _vtk_actor->SetMapper(mapper);

    VTKUtils::setupActorFromRenderConfig(_vtk_actor.Get(), render_config);

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

        _vtk_poly_data->GetPointData()->SetScalars(colors);
    }
}

void VTKMeshGraphicsObject::_setFaces()
{
    vtkNew<vtkCellArray> new_faces;
    new_faces->Allocate(_mesh->numFaces(), _mesh->numFaces() * 3);
    
    for (const auto& face : _mesh->faces())
    {
        vtkIdType triangle[3] = {face[0], face[1], face[2]};
        new_faces->InsertNextCell(3, triangle);
    }
    
    _vtk_poly_data->SetPolys(new_faces);
}

void VTKMeshGraphicsObject::_setVertices()
{
    // create points
    vtkPoints* points = _vtk_poly_data->GetPoints();
    int cur_num_vtk_points = points->GetNumberOfPoints();

    points->Resize(_mesh->numVertices());
    points->SetNumberOfPoints(_mesh->numVertices());

    int vtk_index = 0;
    for (const auto& vert : _mesh->vertices())
    {
        points->SetPoint(vtk_index++, vert.data());
    }
    points->Modified();
}

void VTKMeshGraphicsObject::update() 
{
    
    _setVertices();

    if (_mesh->numFaces() != _vtk_poly_data->GetNumberOfCells())
    {
        _setFaces();
    }
    
}

} // namespace Graphics

