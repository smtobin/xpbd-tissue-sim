#include "fem/HeatConductionFEMSolver.hpp"

namespace FEM
{

HeatConductionFEMSolver::HeatConductionFEMSolver(const Geometry::TetMesh* mesh, Real k, Real sigma, Real h, Real T_a)
    : _mesh(mesh), _fem_mesh(mesh), _laplace_solver(mesh, sigma),
     _k(k), _sigma(sigma), _h(h), _T_a(T_a)
{

}

void HeatConductionFEMSolver::setTemperatureAtBoundary(int vertex_index, Real value)
{
    _essential_boundary[vertex_index] = value;
}

void HeatConductionFEMSolver::clearTemperatureBoundary()
{
    _essential_boundary.clear();
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

Mat4r HeatConductionFEMSolver::_elementStiffnessMatrix(int element_index) const
{
    const typename FEMTetMesh::ElementShapeFunctionGradientsMat delN = _fem_mesh.elementShapeFunctionGradients(element_index);
    Real detJ = _fem_mesh.elementJacobian(element_index).determinant();

    // single point Gauss quadrature
    Mat4r K_e = 0.25*0.25*0.25 * std::abs(detJ) * _k * delN.transpose() * delN;

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
    Real q_g = _sigma * delV.dot(delV);

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