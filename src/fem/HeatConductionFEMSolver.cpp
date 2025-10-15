#include "fem/HeatConductionFEMSolver.hpp"

namespace FEM
{

HeatConductionFEMSolver::HeatConductionFEMSolver(Geometry::TetMesh* mesh, const ElasticMaterial& material, Real h, Real T_a)
    : _mesh(mesh), _fem_mesh(mesh), _laplace_solver(mesh, material.electricalConductivity()),
     _material(material), _h(h), _T_a(T_a)
{
    _T.resize(_mesh->numVertices(), T_a);
    _T_prev = _T;

    // compute thermal masses for each vertex
    _M.resize(_mesh->numVertices(), 0);
    for (const auto& element_index : _mesh->elements().validIndices())
    {
        const Vec4i& elem = _mesh->element(element_index);
        Real volume = _mesh->elementVolume(element_index);
        for (int i = 0; i < 4; i++)
        {
            _M[elem[i]] += 0.25 * volume * _material.density() * _material.specificHeat();
        }
    }

    _on_essential_boundary.resize(_mesh->numVertices(), false);

    // create temperature property for the mesh
    _mesh->addVertexProperty<Real>("temperature", 0);

}

void HeatConductionFEMSolver::setTemperatureAtBoundary(int vertex_index, Real value)
{
    _essential_boundary[vertex_index] = value;

    _on_essential_boundary[vertex_index] = true;
}

void HeatConductionFEMSolver::clearTemperatureBoundary()
{
    _essential_boundary.clear();

    _on_essential_boundary.assign(_mesh->numVertices(), false);
}

void HeatConductionFEMSolver::setVoltageAtBoundary(int vertex_index, Real voltage)
{
    _laplace_solver.setEssentialBoundary(vertex_index, voltage);
}

void HeatConductionFEMSolver::clearVoltageBoundary()
{
    _laplace_solver.clearEssentialBoundary();
}

VecXr HeatConductionFEMSolver::solve()
{
    // solve for the voltage
    _V = _laplace_solver.solve();

    // assemble global system
    _assembly();

    // solve the linear system
    VecXr x = _system_matrix.llt().solve(_RHS_vec);
    return x;
}

void HeatConductionFEMSolver::step(Real dt)
{

    // enforce essential boundary conditions
    for (const auto& [vertex_index, temp] : _essential_boundary)
    {
        _T_prev[vertex_index] = temp;
        _T[vertex_index] = temp;
    }

    // loop through elements, calculate contribution to next temperatures
    // for now, we assume heat generation is 0
    for (const auto& element_index : _mesh->elements().validIndices())
    {
        const typename FEMTetMesh::ElementShapeFunctionGradientsMat delN = _fem_mesh.elementShapeFunctionGradients(element_index);
        Real detJ = _fem_mesh.elementJacobian(element_index).determinant();

        // single point Gauss quadrature
        Mat4r K_e = 0.25*0.25*0.25 * std::abs(detJ) * _material.thermalConductivity() * delN.transpose() * delN;

        // compute K*T
        const Vec4i& elem = _mesh->element(element_index);
        Vec4r T_e(_T_prev[elem[0]], _T_prev[elem[1]], _T_prev[elem[2]], _T_prev[elem[3]]);
        Vec4r K_e_T_e = K_e * T_e;

        // scatter back to next temperature
        // heat generation term will replace the zero
        for (int i = 0; i < 4; i++)
        {
            if (_on_essential_boundary[elem[i]])
                continue;

            _T[elem[i]] += dt * 1.0/_M[elem[i]] * (0 - K_e_T_e[i]);
        }
        
    }

    // loop through surface faces, calculate contribution to next temperatures
    // assuming a convective boundary
    for (const auto& face_index : _mesh->faces().validIndices())
    {
        const typename FEMTetMesh::FaceJacobianMat J_e = _fem_mesh.faceJacobian(face_index);
        Real detJ = J_e.row(0).cross(J_e.row(1)).norm();

        // 3-point Gauss quadrature
        Mat3r K_e = Mat3r::Zero();
        for (int i = 0; i < FEMTetMesh::NUM_FACE_QUADRATURE_PTS; i++)
        {
            Real weight = FEMTetMesh::FACE_QUADRATURE_weights[i];
            Real e1 = FEMTetMesh::FACE_QUADRATURE_e1[i];
            Real e2 = FEMTetMesh::FACE_QUADRATURE_e2[i];

            Vec3r shape_funcs = _fem_mesh.faceShapeFunctions(e1, e2);
            K_e += weight * std::abs(detJ) * _h * shape_funcs * shape_funcs.transpose();
        }

        // compute K*T
        const Vec3i& face = _mesh->face(face_index);
        Vec3r T_e(_T_prev[face[0]], _T_prev[face[1]], _T_prev[face[2]]);
        Vec3r K_e_T_e = K_e * T_e;

        // scatter back to next temperature
        for (int i = 0; i < 3; i++)
        {
            if (_on_essential_boundary[face[i]])
                continue;

            _T[face[i]] += dt * 1.0/_M[face[i]] * (-K_e_T_e[i]);
        }
    }

    // update T_prev
    _T_prev = _T;

    // copy into mesh property
    Geometry::MeshProperty<Real>& temperature_prop = _mesh->getVertexProperty<Real>("temperature");
    for (unsigned i = 0; i < _T.size(); i++)
    {
        temperature_prop.set(i, _T[i]);
    }

    std::cout << "T: [" << std::endl;
    for (const auto& t : _T)
    {
        std::cout << t << std::endl;
    }
    std::cout << "]\n" << std::endl;
}

Mat4r HeatConductionFEMSolver::_elementStiffnessMatrix(int element_index) const
{
    const typename FEMTetMesh::ElementShapeFunctionGradientsMat delN = _fem_mesh.elementShapeFunctionGradients(element_index);
    Real detJ = _fem_mesh.elementJacobian(element_index).determinant();

    // single point Gauss quadrature
    Mat4r K_e = 0.25*0.25*0.25 * std::abs(detJ) * _material.thermalConductivity() * delN.transpose() * delN;

    return K_e;
}

Vec4r HeatConductionFEMSolver::_elementRHSVector(int element_index) const
{
    // get the gradient of voltage
    const typename FEMTetMesh::ElementShapeFunctionGradientsMat delN = _fem_mesh.elementShapeFunctionGradients(element_index);
    const Vec4i& elem = _mesh->element(element_index);
    const Vec4r V_e(_V[elem[0]], _V[elem[1]], _V[elem[2]], _V[elem[3]]);
    const Vec3r delV = delN * V_e;
    // compute the heat source term using voltage gradient
    Real q_g = _material.electricalConductivity() * delV.dot(delV);

    Vec4r shape_funcs = _fem_mesh.elementShapeFunctions(0.25, 0.25, 0.25);
    Real detJ = _fem_mesh.elementJacobian(element_index).determinant();

    // single point Gauss quadrature
    Vec4r RHS = 0.25*0.25*0.25 * std::abs(detJ) * q_g * shape_funcs;

    return RHS;
}

Mat3r HeatConductionFEMSolver::_faceStiffnessMatrix(int face_index) const
{
    const typename FEMTetMesh::FaceJacobianMat J_e = _fem_mesh.faceJacobian(face_index);
    Real detJ = J_e.row(0).cross(J_e.row(1)).norm();

    // 3-point Gauss quadrature
    Mat3r K_e = Mat3r::Zero();
    for (int i = 0; i < FEMTetMesh::NUM_FACE_QUADRATURE_PTS; i++)
    {
        Real weight = FEMTetMesh::FACE_QUADRATURE_weights[i];
        Real e1 = FEMTetMesh::FACE_QUADRATURE_e1[i];
        Real e2 = FEMTetMesh::FACE_QUADRATURE_e2[i];

        Vec3r shape_funcs = _fem_mesh.faceShapeFunctions(e1, e2);
        K_e += weight * std::abs(detJ) * _h * shape_funcs * shape_funcs.transpose();
    }

    return K_e;
}

void HeatConductionFEMSolver::_assembly()
{
    /** TODO: adapt for changes in number of vertices */
    _system_matrix = MatXr::Zero(_mesh->numVertices(), _mesh->numVertices());
    _RHS_vec = VecXr::Zero(_mesh->numVertices());

    // assemble stiffness matrix
    for (const auto& element_index : _mesh->elements().validIndices())
    {
        Mat4r K_e = _elementStiffnessMatrix(element_index);

        const Vec4i& elem = _mesh->element(element_index);
        for (int i = 0; i < 4; i++)
        {
            for (int j = 0; j < 4; j++)
            {
                _system_matrix(elem[i], elem[j]) += K_e(i,j);
            }
        }   
    }
    for (const auto& face_index : _mesh->faces().validIndices())
    {
        Mat3r K_e = _faceStiffnessMatrix(face_index);

        const Vec3i& face = _mesh->face(face_index);
        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                _system_matrix(face[i], face[j]) += K_e(i,j);
            }
        }
    }

    // assemble RHS vector
    for (const auto& element_index : _mesh->elements().validIndices())
    {
        Vec4r RHS_e = _elementRHSVector(element_index);

        const Vec4i& elem = _mesh->element(element_index);
        for (int i = 0; i < 4; i++)
        {
            _RHS_vec[elem[i]] += RHS_e[i];
        }
    }

    // then, perform elimination
    for (const auto& [index, value] : _essential_boundary)
    {
        _RHS_vec[index] = value;

        // perform elimination
        for (int row = 0; row < _mesh->numVertices(); row++)
        {
            if (row == index)
                continue;
            
            _RHS_vec[row] -= _system_matrix(row, index) * _RHS_vec[index];
        }
        _system_matrix.row(index) = VecXr::Zero(_mesh->numVertices());
        _system_matrix.col(index) = VecXr::Zero(_mesh->numVertices());
        _system_matrix(index,index) = 1;
    }
}

} // namespace FEM