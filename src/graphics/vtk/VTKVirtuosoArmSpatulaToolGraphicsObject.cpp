#include "graphics/vtk/VTKVirtuosoArmSpatulaToolGraphicsObject.hpp"
#include "graphics/vtk/VTKUtils.hpp"

#include <vtkCylinderSource.h>
#include <vtkPolyDataMapper.h>
#include <vtkLinearExtrusionFilter.h>
#include <vtkTransformPolyDataFilter.h>

namespace Graphics
{

std::vector<Vec3r> GenerateArc(
    Real cx, Real cy,
    Real radius,
    Real startAngle,
    Real endAngle,
    int segments)
{
    std::vector<Vec3r> pts;
    Real step = (endAngle - startAngle) / segments;

    for (int i = 0; i <= segments; ++i)
    {
        Real a = startAngle + i * step;
        pts.push_back(Vec3r(
            cx + radius * cos(a),
            0.0,
            cy + radius * sin(a)
        ));
    }
    return pts;
}

VTKVirtuosoArmSpatulaToolGraphicsObject::VTKVirtuosoArmSpatulaToolGraphicsObject(const std::string& name, const Sim::VirtuosoArmSpatulaTool* spatula, const Config::ObjectRenderConfig& render_config)
    : VirtuosoArmSpatulaToolGraphicsObject(name, spatula)
{

    Real width = Sim::VirtuosoArmSpatulaTool::WIDTH;
    Real length = Sim::VirtuosoArmSpatulaTool::LENGTH;
    Real radius = Sim::VirtuosoArmSpatulaTool::RADIUS;
    Real thickness = Sim::VirtuosoArmSpatulaTool::THICKNESS;
    int arc_resolution = 10;

    vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
    vtkSmartPointer<vtkPolygon> polygon = vtkSmartPointer<vtkPolygon>::New();

    std::vector<Vec3r> outline;

    Real hw = width / 2.0;
    Real hl = length / 2.0;

    // ---- Build rounded rectangle (CCW) ----

    // Top-right corner
    auto arcTR = GenerateArc(hw - radius, hl - radius, radius, 0.0, M_PI / 2.0, arc_resolution);

    // Top-left
    auto arcTL = GenerateArc(-hw + radius, hl - radius, radius, M_PI / 2.0, M_PI, arc_resolution);

    // Bottom-left
    auto arcBL = GenerateArc(-hw + radius, -hl + radius, radius, M_PI, 3.0 * M_PI / 2.0, arc_resolution);

    // Bottom-right
    auto arcBR = GenerateArc(hw - radius, -hl + radius, radius, 3.0 * M_PI / 2.0, 2.0 * M_PI, arc_resolution);

    // Combine arcs
    outline.insert(outline.end(), arcTR.begin(), arcTR.end());
    outline.insert(outline.end(), arcTL.begin(), arcTL.end());
    outline.insert(outline.end(), arcBL.begin(), arcBL.end());
    outline.insert(outline.end(), arcBR.begin(), arcBR.end());

    // Add points to VTK
    polygon->GetPointIds()->SetNumberOfIds(outline.size());

    for (size_t i = 0; i < outline.size(); ++i)
    {
        vtkIdType id = points->InsertNextPoint(outline[i].data());
        polygon->GetPointIds()->SetId(i, id);
    }

    vtkSmartPointer<vtkCellArray> polygons = vtkSmartPointer<vtkCellArray>::New();
    polygons->InsertNextCell(polygon);

    vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();
    polyData->SetPoints(points);
    polyData->SetPolys(polygons);

    // Triangulate (important for extrusion correctness)
    vtkSmartPointer<vtkTriangleFilter> triangulate =
        vtkSmartPointer<vtkTriangleFilter>::New();
    triangulate->SetInputData(polyData);

    vtkNew<vtkTransform> preTransform;
    preTransform->Translate(0.0, -thickness/2, 0.0);

    vtkNew<vtkTransformPolyDataFilter> transformFilter;
    transformFilter->SetInputConnection(triangulate->GetOutputPort());
    transformFilter->SetTransform(preTransform);

    // Extrude
    vtkSmartPointer<vtkLinearExtrusionFilter> extrude =
        vtkSmartPointer<vtkLinearExtrusionFilter>::New();
    extrude->SetInputConnection(transformFilter->GetOutputPort());
    extrude->SetExtrusionTypeToNormalExtrusion();
    extrude->SetVector(0, thickness, 0);
    extrude->CappingOn();

    

    // smooth normals
    vtkNew<vtkPolyDataNormals> normal_generator;
    normal_generator->SetInputConnection(extrude->GetOutputPort());
    normal_generator->SetFeatureAngle(30.0);
    normal_generator->SplittingOff();
    normal_generator->ConsistencyOn();
    normal_generator->ComputePointNormalsOn();
    normal_generator->ComputeCellNormalsOff();
    normal_generator->Update();

    // Mapper + Actor
    vtkSmartPointer<vtkPolyDataMapper> mapper =
        vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputConnection(normal_generator->GetOutputPort());

    _vtk_actor = vtkSmartPointer<vtkActor>::New();
    _vtk_actor->SetMapper(mapper);

    VTKUtils::setupActorFromRenderConfig(_vtk_actor.Get(), render_config);

    _vtk_transform = vtkSmartPointer<vtkTransform>::New();

    Geometry::TransformationMatrix spatula_transform = _spatula->spatulaFrame().transform();
     // IMPORTANT: use row-major ordering since that is what VTKTransform expects (default for Eigen is col-major)
    Eigen::Matrix<Real, 4, 4, Eigen::RowMajor> transform_mat = spatula_transform.asMatrix();
    _vtk_transform->SetMatrix(transform_mat.data());

    _vtk_actor->SetUserTransform(_vtk_transform);
}

void VTKVirtuosoArmSpatulaToolGraphicsObject::updateGraphicsBuffers()
{
    RenderInfo* rinfo = _latest_rinfo.load(std::memory_order_acquire);

    Geometry::TransformationMatrix spatula_transform = rinfo->spatula_frame.transform();
     // IMPORTANT: use row-major ordering since that is what VTKTransform expects (default for Eigen is col-major)
    Eigen::Matrix<Real, 4, 4, Eigen::RowMajor> transform_mat = spatula_transform.asMatrix();
    _vtk_transform->SetMatrix(transform_mat.data());

    _vtk_actor->SetUserTransform(_vtk_transform);
}

} // namespace Graphics