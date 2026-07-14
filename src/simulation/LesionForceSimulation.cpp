#include "simulation/LesionForceSimulation.hpp"

namespace Sim
{

LesionForceSimulation::LesionForceSimulation(const Config::VirtuosoCTAnatomySimulationConfig* config)
    : VirtuosoCTAnatomySimulation(config), _lesion_body_force(0,0,0), _lesion_class_index(0)
{

}

void LesionForceSimulation::setup()
{
    VirtuosoCTAnatomySimulation::setup();

    // get the element class the corresponds to the lesion
    const auto& material_classes = _tissue_obj.materialClasses();
    std::string lesion_material = "Lesion";
    for (unsigned i = 0; i < material_classes.size(); i++)
    {
        if (material_classes[i]->name() == lesion_material)
        {
            _lesion_class_index = i;
            break;
        }
    }

    // get the lesion elements
    const auto [lesion_vertices, lesion_faces, lesion_elements] = _tissue_obj.tetMesh()->submeshForElementClass(_lesion_class_index);
    _lesion_elements = lesion_elements;
}

void LesionForceSimulation::setLesionBodyForce(const Vec3r& force)
{
    Geometry::TetMesh* tet_mesh = _tissue_obj.tetMesh();
    // clear applied forces in tissue obj
    for (auto v : tet_mesh->vertices().validIndices())
    {
        _tissue_obj.setVertexAppliedForce(v, Vec3r::Zero());
    }

    // for each of the lesion elements, distribute forces to vertices
    _lesion_body_force = force;
    for (auto e : _lesion_elements)
    {
        const Vec4i& indices = tet_mesh->element(e);
        Real V = tet_mesh->elementRestVolume(e);
        Vec3r total_force = V * _lesion_body_force;

        // distribute to vertices
        for (int k = 0 ; k < 4; k++)
        {
            Vec3r cur_force = _tissue_obj.vertexAppliedForce(indices[k]);
            _tissue_obj.setVertexAppliedForce(indices[k], cur_force + 0.25*total_force);
        }
    }
    
}

} // namespace Sim