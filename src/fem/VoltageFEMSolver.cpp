#include "fem/VoltageFEMSolver.hpp"

namespace FEM
{

VoltageFEMSolver::VoltageFEMSolver(Geometry::TetMesh* mesh, Real k)
    : _mesh(mesh), _fem_mesh(mesh), _k(k)
{
    _V.resize(_mesh->numVertices(), 0);
    _V_prev = _V;

    // compute electrical "masses" for each vertex
    _M.resize(_mesh->numVertices(), 0);
    for (const auto& element_index : _mesh->elements().validIndices())
    {
        const Vec4i& elem = _mesh->element(element_index);
        Real volume = _mesh->elementVolume(element_index);
        for (int i = 0; i < 4; i++)
        {
            _M[elem[i]] += 0.25 * volume; // some number
        }
    }

    _on_essential_boundary.resize(_mesh->numVertices(), false);

    // create voltage property for the mesh
    _mesh->addVertexProperty<Real>("voltage", 0);
}

void VoltageFEMSolver::setVoltageAtBoundary(int vertex_index, Real value)
{
    _essential_boundary[vertex_index] = value;

    _on_essential_boundary[vertex_index] = true;
}

void VoltageFEMSolver::clearVoltageBoundary()
{
    _essential_boundary.clear();

    _on_essential_boundary.assign(_mesh->numVertices(), false);
}

void VoltageFEMSolver::step(Real dt)
{

    // enforce essential boundary conditions
    for (const auto& [vertex_index, temp] : _essential_boundary)
    {
        _V_prev[vertex_index] = temp;
        _V[vertex_index] = temp;
    }

    // loop through elements, calculate contribution to next temperatures
    // for now, we assume heat generation is 0
    for (const auto& element_index : _mesh->elements().validIndices())
    {
        const typename FEMTetMesh::ElementShapeFunctionGradientsMat delN = _fem_mesh.elementShapeFunctionGradients(element_index);
        Real detJ = _fem_mesh.elementJacobian(element_index).determinant();

        // single point Gauss quadrature
        Mat4r K_e = 0.25*0.25*0.25 * std::abs(detJ) * _k * delN.transpose() * delN;

        // compute K*T
        const Vec4i& elem = _mesh->element(element_index);
        Vec4r V_e(_V_prev[elem[0]], _V_prev[elem[1]], _V_prev[elem[2]], _V_prev[elem[3]]);
        Vec4r K_e_V_e = K_e * V_e;

        // scatter back to next voltage
        for (int i = 0; i < 4; i++)
        {
            if (_on_essential_boundary[elem[i]])
                continue;

            _V[elem[i]] += dt * 1.0/_M[elem[i]] * (-K_e_V_e[i]);
        }
        
    }

    // update T_prev
    _V_prev = _V;

    // copy into mesh property
    Geometry::MeshProperty<Real>& voltage_prop = _mesh->getVertexProperty<Real>("voltage");
    for (unsigned i = 0; i < _V.size(); i++)
    {
        voltage_prop.set(i, _V[i]);
    }

    std::cout << "V: [" << std::endl;
    for (const auto& t : _V)
    {
        std::cout << t << std::endl;
    }
    std::cout << "]\n" << std::endl;
}

} // namespace FEM