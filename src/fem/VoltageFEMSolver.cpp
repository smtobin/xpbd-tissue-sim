#include "fem/VoltageFEMSolver.hpp"

namespace FEM
{

VoltageFEMSolver::VoltageFEMSolver(Geometry::RefinedTetMesh* mesh, Real k)
    : _mesh(mesh), _fem_mesh(mesh), _k(k)
{
    // create voltage property for the mesh
    _mesh->addVertexProperty<Real>("voltage", 0);

    PetscInitialize(NULL, NULL, NULL, NULL);

    _allocateMemory();

    _prev_num_vertices = _mesh->numVertices();
    _prev_num_elements = _mesh->numElements();

    // create PETSc linear system solver
    KSPCreate(PETSC_COMM_WORLD, &_ksp);
    KSPSetInitialGuessNonzero(_ksp, true);  // we have Dirichlet boundary conditions, so we know part of the solution already!
    KSPSetOperators(_ksp, _A, _A);
    KSPSetFromOptions(_ksp);

    
}

void VoltageFEMSolver::_allocateMemory()
{
    // allocate memory for voltage mesh property
    _mesh->getVertexProperty<Real>("voltage").resize(_mesh->numVertices());

    // allocate memory for the voltage std::vector
    _V.resize(_mesh->numVertices(), 0);
    _V_prev = _V;

    // allocate memory for the essential boundary
    _on_essential_boundary.resize(_mesh->numVertices(), false);

    // if A, b, x already exist, delete them
    if (_A) MatDestroy(&_A);
    if (_b) VecDestroy(&_b);
    if (_x) VecDestroy(&_x);

    // allocate memory per row based on mesh connectivity
    std::vector<int> nnz;
    nnz.reserve(_mesh->numVertices());
    for (const auto& vertex_index : _mesh->vertices().validIndices())
    {
        const std::unordered_set<int>& adj_verts = _mesh->vertexAdjacentVertices(vertex_index);
        nnz.push_back(adj_verts.size() + 1);
    }
    // create PETSc global stiffness matrix
    MatCreateSeqAIJ(PETSC_COMM_WORLD, _mesh->numVertices(), _mesh->numVertices(), 0, nnz.data(), &_A);

    // create PETSc global RHS vector
    VecCreate(PETSC_COMM_WORLD, &_b);
    VecSetSizes(_b, _mesh->numVertices(), _mesh->numVertices());
    VecSetFromOptions(_b);

    VecDuplicate(_b, &_x);

    if (_ksp)
    {
        KSPSetOperators(_ksp, _A, _A);
    }

    MatSetOption(_A, MAT_NEW_NONZERO_ALLOCATION_ERR, PETSC_TRUE);

}


void VoltageFEMSolver::setVoltageAtBoundary(int vertex_index, Real value, bool permanent)
{
    if (permanent)
        _permanent_essential_boundary[vertex_index] = value;
    else
        _temporary_essential_boundary[vertex_index] = value;

    _on_essential_boundary[vertex_index] = true;
}

void VoltageFEMSolver::clearVoltageBoundary()
{
    for (const auto& [vertex_index, temp] : _temporary_essential_boundary)
        _on_essential_boundary[vertex_index] = false;
    
    _temporary_essential_boundary.clear();
}

void VoltageFEMSolver::step(Real /* dt */)
{
    // check if the number of elements or vertices has changed since we last solved
    if (_mesh->numVertices() != _prev_num_vertices || _mesh->numElements() != _prev_num_elements)
    {
        // if it has, reallocate the stiffness matrix, RHS vector, and solution vector
        _allocateMemory();
    }

    // assemble global system
    _assembly();

    // solve the linear system
    KSPSolve(_ksp, _b, _x);

    // copy into mesh property
    Geometry::MeshProperty<Real>& voltage_prop = _mesh->getVertexProperty<Real>("voltage");
    std::vector<Real>& prop_data = voltage_prop.properties();
    Real* data;
    VecGetArray(_x, &data);
    // copy memory
    memcpy(_V.data(), data, sizeof(Real)*_mesh->numVertices());
    memcpy(prop_data.data(), data, sizeof(Real)*_mesh->numVertices());
    VecRestoreArray(_x, &data);
}

VoltageFEMSolver::ElementStiffnessMatrixType VoltageFEMSolver::_elementStiffnessMatrix(int element_index) const
{
    const typename FEMTetMesh::ElementShapeFunctionGradientsMat delN = _fem_mesh.elementShapeFunctionGradients(element_index);
    Real detJ = _fem_mesh.elementJacobian(element_index).determinant();

    // single point Gauss quadrature
    ElementStiffnessMatrixType K_e = 0.25*0.25*0.25 * std::abs(detJ) * _k * delN.transpose() * delN;

    return K_e;
}

void VoltageFEMSolver::_assembly()
{
    MatZeroEntries(_A);
    VecZeroEntries(_b);

    // assemble stiffness matrix
    for (const auto& element_index : _mesh->elements().validIndices())
    {
        ElementStiffnessMatrixType K_e = _elementStiffnessMatrix(element_index);

        const Vec4i& elem = _mesh->element(element_index);
        PetscErrorCode ierr = MatSetValues(_A, 4, elem.data(), 4, elem.data(), K_e.data(), ADD_VALUES);
        assert(ierr == 0); 
    }

    MatAssemblyBegin(_A, MAT_FINAL_ASSEMBLY);
    MatAssemblyEnd(_A, MAT_FINAL_ASSEMBLY);

    // assemble RHS vector
    // since we're assuming insulated natural boundary (boundary flux = 0) this is just the essential boundary
    // also, perform elimination
    std::vector<int> boundary_row_indices;
    boundary_row_indices.reserve(_temporary_essential_boundary.size() + _permanent_essential_boundary.size());
    for (const auto& [index, value] : _temporary_essential_boundary)
    {
        boundary_row_indices.push_back(index);
        PetscErrorCode ierr = VecSetValue(_x, index, value, INSERT_VALUES);
        assert(ierr == 0);
    }

    for (const auto& [index, value] : _permanent_essential_boundary)
    {
        boundary_row_indices.push_back(index);
        PetscErrorCode ierr = VecSetValue(_x, index, value, INSERT_VALUES);
        assert(ierr == 0);
    }

    PetscErrorCode ierr = MatZeroRowsColumns(_A, boundary_row_indices.size(), boundary_row_indices.data(), 1.0, _x, _b);
    assert(ierr == 0);

    // apply hanging node constraints
    const std::unordered_map<int, Geometry::Edge>& hanging_vertices = _mesh->hangingVertices();
    for (const auto [v, edge] : hanging_vertices)
    {
        // row v is now: V_v - 0.5*V_edge1 - 0.5*V_edge2 = 0
        int cols[] = {v, edge.index1, edge.index2};
        Real vals[] = {1.0, -0.5, -0.5};
        MatSetValues(_A, 1, &v, 3, cols, vals, INSERT_VALUES);
        VecSetValue(_b, v, 0.0, INSERT_VALUES); 
    }

    MatAssemblyBegin(_A, MAT_FINAL_ASSEMBLY);
    MatAssemblyEnd(_A, MAT_FINAL_ASSEMBLY);

    VecAssemblyBegin(_b);
    VecAssemblyEnd(_b);
}

} // namespace FEM