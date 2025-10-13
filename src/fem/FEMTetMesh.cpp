#include "fem/FEMTetMesh.hpp"

namespace FEM
{

FEMTetMesh::FEMTetMesh(const Geometry::TetMesh* mesh)
    : _mesh(mesh)
{

}

Mat3r FEMTetMesh::elementJacobian(int element_index) const
{
    const Vec4i& elem = _mesh->element(element_index);
    const Vec3r& v1 = _mesh->vertex(elem[0]);
    const Vec3r& v2 = _mesh->vertex(elem[1]);
    const Vec3r& v3 = _mesh->vertex(elem[2]);
    const Vec3r& v4 = _mesh->vertex(elem[3]);

    Mat3r J_e;
    J_e.row(0) = v1 - v4;
    J_e.row(1) = v2 - v4;
    J_e.row(2) = v3 - v4;

    return J_e;
}

Vec4r FEMTetMesh::elementShapeFunctions(Real e1, Real e2, Real e3) const
{
    return Vec4r(e1, e2, e3, 1-e1-e2-e3);
}

FEMTetMesh::ElementShapeFunctionGradientsMat FEMTetMesh::elementShapeFunctionGradients(int element_index) const
{
    ElementShapeFunctionGradientsMat nat_coord_grads;
    nat_coord_grads << 
        1, 0, 0, -1,
        0, 1, 0, -1,
        0, 0, 1, -1;
    
    return elementJacobian(element_index).inverse() * nat_coord_grads;
}

///////////////////

FEMTetMesh::FaceJacobianMat FEMTetMesh::faceJacobian(int face_index) const
{
    const Vec3i& face = _mesh->face(face_index);
    const Vec3r& v1 = _mesh->vertex(face[0]);
    const Vec3r& v2 = _mesh->vertex(face[1]);
    const Vec3r& v3 = _mesh->vertex(face[2]);

    FaceJacobianMat J_e;
    J_e.row(0) = v1 - v2;
    J_e.row(1) = v1 - v3;

    return J_e;
}

Vec3r FEMTetMesh::faceShapeFunctions(Real e1, Real e2) const
{
    return Vec3r(e1, e2, 1-e1-e2);
}


} // namespace FEM