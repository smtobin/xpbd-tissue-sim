#include "fem/LaplacianFEMSolver.hpp"

namespace FEM
{

LaplacianFEMSolver::LaplacianFEMSolver(Geometry::TetMesh* mesh, Real k)
    : _mesh(mesh), _fem_mesh(mesh), _k(k) 
{

}

void LaplacianFEMSolver::setEssentialBoundary(int vertex_index, Real value)
{
    _essential_boundary[vertex_index] = value;
}

void LaplacianFEMSolver::clearEssentialBoundary()
{
    _essential_boundary.clear();
}

VecXr LaplacianFEMSolver::solve()
{
    // assemble global system
    _assembly();

    // solve the linear system
    VecXr x = _system_matrix.llt().solve(_RHS_vec);
    return x;
}

Mat4r LaplacianFEMSolver::_elementStiffnessMatrix(int element_index) const
{
    const typename FEMTetMesh::ElementShapeFunctionGradientsMat delN = _fem_mesh.elementShapeFunctionGradients(element_index);
    Real detJ = _fem_mesh.elementJacobian(element_index).determinant();

    // single point Gauss quadrature
    Mat4r K_e = 0.25*0.25*0.25 * std::abs(detJ) * _k * delN.transpose() * delN;

    return K_e;
}

void LaplacianFEMSolver::_assembly()
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

    // assemble RHS vector
    // since we're assuming insulated natural boundary (boundary flux = 0) this is just the essential boundary
    // also, perform elimination
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