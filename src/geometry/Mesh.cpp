#include "geometry/Mesh.hpp"

#include <set>
#include <iostream>
#include <fstream>

#ifdef HAVE_CUDA
#include "gpu/resource/MeshGPUResource.hpp"
#endif

namespace Geometry
{

Mesh::Mesh(const VerticesMat& vertices, const FacesMat& faces)
    : _vertices(vertices), _faces(faces)
{
    // create surface vertex property
    addVertexProperty<bool>("surface");
    auto& surface_property = getVertexProperty<bool>("surface");
    for (int i = 0; i < numFaces(); i++)
    {
        const Eigen::Vector3i& cur_face = face(i);
        surface_property.set(cur_face[0], true);
        surface_property.set(cur_face[1], true);
        surface_property.set(cur_face[2], true);
    }

    _mesh_origin = Vec3r::Zero();

    setCurrentStateAsUndeformedState();
}

Mesh::Mesh(const Mesh& other)
{
    _vertices = other._vertices;
    _faces = other._faces;
    _unrotated_size_xyz = other._unrotated_size_xyz;
    _mesh_origin = other._mesh_origin;
    _vertex_properties = other._vertex_properties;
    _face_properties = other._face_properties;
    _vertex_adjacent_vertices = other._vertex_adjacent_vertices;

    // NOTE: we do NOT do anything with the GPU resource - if we are copying this mesh, we don't want to just automatically create a new GPU resource if we don't need to
    // (we can't copy the GPU resource since it's a unique_ptr)
}

Mesh::Mesh(Mesh&& other)
{
    _vertices = std::move(other._vertices);
    _faces = std::move(other._faces);
    _unrotated_size_xyz = std::move(other._unrotated_size_xyz);
    _mesh_origin = std::move(other._mesh_origin);
    _vertex_properties = std::move(other._vertex_properties);
    _face_properties = std::move(other._face_properties);
    _vertex_adjacent_vertices = std::move(other._vertex_adjacent_vertices);

 #ifdef HAVE_CUDA
    _gpu_resource = std::move(other._gpu_resource);
 #endif
}

void Mesh::_computeAdjacentVertices()
{
    _vertex_adjacent_vertices.resize(numVertices());
    
    // clear all the adjacency lists
    for (int i = 0; i < numVertices(); i++)
    {
        _vertex_adjacent_vertices[i].clear();
    }

    // go through each of the faces and add adjacent vertices for each vertex in the face
    for (int i = 0; i < numFaces(); i++)
    {
        const Eigen::Vector3i& cur_face = face(i);

        std::vector<int>& adj_verts0 = _vertex_adjacent_vertices[cur_face[0]];
        std::vector<int>& adj_verts1 = _vertex_adjacent_vertices[cur_face[1]];
        std::vector<int>& adj_verts2 = _vertex_adjacent_vertices[cur_face[2]];

        // for v0
        if (std::find(adj_verts0.begin(), adj_verts0.end(), cur_face[1]) == adj_verts0.end())   adj_verts0.push_back(cur_face[1]);
        if (std::find(adj_verts0.begin(), adj_verts0.end(), cur_face[2]) == adj_verts0.end())   adj_verts0.push_back(cur_face[2]);

        // for v1
        if (std::find(adj_verts1.begin(), adj_verts1.end(), cur_face[0]) == adj_verts1.end())   adj_verts1.push_back(cur_face[0]);
        if (std::find(adj_verts1.begin(), adj_verts1.end(), cur_face[2]) == adj_verts1.end())   adj_verts1.push_back(cur_face[2]);

        // for v2
        if (std::find(adj_verts2.begin(), adj_verts2.end(), cur_face[0]) == adj_verts2.end())   adj_verts2.push_back(cur_face[0]);
        if (std::find(adj_verts2.begin(), adj_verts2.end(), cur_face[1]) == adj_verts2.end())   adj_verts2.push_back(cur_face[1]);
    }
}

void Mesh::setCurrentStateAsUndeformedState()
{
    AABB bbox = boundingBox();
    _unrotated_size_xyz = bbox.size();

    _computeAdjacentVertices();
}

void Mesh::updateVertexNormals()
{
    _vertex_normals = VerticesMat::Zero(3, numVertices());
    for (int i = 0; i < numFaces(); i++)
    {
        const Vec3i& f = face(i);
        // edge 0->1
        const Vec3r e01 = vertex(f[1]) - vertex(f[0]);
        // edge 1->2
        const Vec3r e12 = vertex(f[2]) - vertex(f[1]);
        // edge 2->0
        const Vec3r e20 = vertex(f[2]) - vertex(f[0]);

        _vertex_normals.col(f[0]) += e20.cross(e01);
        _vertex_normals.col(f[1]) += e01.cross(e12);
        _vertex_normals.col(f[2]) += e12.cross(e20);
    }
}

Real* Mesh::vertexPointer(const int index) const
{
    assert(index < numVertices());
    Real* p = const_cast<Real*>(_vertices.col(index).data());
    return p;
}

AABB Mesh::boundingBox() const
{
    Vec3r min = _vertices.rowwise().minCoeff();
    Vec3r max = _vertices.rowwise().maxCoeff();
    return AABB(min, max);
}

std::pair<int,Real> Mesh::averageFaceEdgeLength() const
{
    // keep track of unique edges, stored in ascending-index order
    std::set<std::pair<int, int>> edges;

    // add all the unique edges to the set
    for (const auto& face : _faces.colwise())
    {
        // 3 edges per face
        // make sure to insert them into the set in ascending-index order
        if (face[0] > face[1])  edges.insert(std::pair<int,int>(face[1], face[0]));
        else                    edges.insert(std::pair<int,int>(face[0], face[1]));

        if (face[0] > face[2])  edges.insert(std::pair<int,int>(face[2], face[0]));
        else                    edges.insert(std::pair<int,int>(face[0], face[2]));

        if (face[1] > face[2])  edges.insert(std::pair<int,int>(face[2], face[1]));
        else                    edges.insert(std::pair<int,int>(face[1], face[2]));
    }

    // go through set of edges and add up total edge length
    Real total_edge_length = 0;
    for (const auto& edge : edges)
    {
        total_edge_length += (vertex(edge.first) - vertex(edge.second)).norm();
    }

    // return the number of unqiue edges as well as the average edge length in the mesh
    return std::pair<int,Real>(edges.size(), total_edge_length / edges.size());
}

int Mesh::getClosestVertex(const Vec3r& p) const
{
    unsigned closest_index = 0;
    Real closest_dist = (p - vertex(0)).squaredNorm();
    // I'm sure there is a better way to do this with std
    for (int i = 0; i < _vertices.cols(); i++)
    {
        Real dist = (p - vertex(i)).squaredNorm();
        if (dist < closest_dist)
        {
            closest_index = i;
            closest_dist = dist;
        }
    }

    return closest_index;

}

std::vector<int> Mesh::getVerticesWithX(const Real x) const
{
    std::vector<int> verts;
    for (int i = 0; i < numVertices(); i++)
    {
        if (_vertices(0,i) == x)
        {
            verts.push_back(i);
        }
    }

    return verts;
}

std::vector<int> Mesh::getVerticesWithY(const Real y) const
{
    std::vector<int> verts;
    for (int i = 0; i < numVertices(); i++)
    {
        if (_vertices(1,i) == y)
        {
            verts.push_back(i);
        }
    }

    return verts;
}

std::vector<int> Mesh::getVerticesWithZ(const Real z) const
{
    std::vector<int> verts;
    for (int i = 0; i < numVertices(); i++)
    {
        if (_vertices(2,i) == z)
        {
            verts.push_back(i);
        }
    }

    return verts;
}

void Mesh::resize(const Real size_of_max_dim)
{
    // compute the AABB
    const AABB aabb = boundingBox();
    // find the scaling factor such that when the mesh is scaled the largest dimension has the specified size
    Real scaling_factor = size_of_max_dim / (aabb.max - aabb.min).maxCoeff();

    // move all vertices to be centered around (0,0,0), apply the scaling, and then move them back
    // moveTogether(-aabb.center());
    _vertices *= scaling_factor;

    _mesh_origin *= scaling_factor;
    // moveTogether(aabb.center());

    // scale the unrotated size
    _unrotated_size_xyz *= scaling_factor;
}

void Mesh::resize(const Vec3r& new_size)
{
    // compute the AABB
    const AABB aabb = boundingBox();
    const Vec3r size = aabb.size();
    // compute the scaling factors for each dimension
    // compute the scaling factors for each dimension
    Real scaling_factor_x = (size(0) != 0) ? new_size(0) / size(0) : 1;
    Real scaling_factor_y = (size(1) != 0) ? new_size(1) / size(1) : 1;
    Real scaling_factor_z = (size(2) != 0) ? new_size(2) / size(2) : 1;

    // move all vertices to be centered around (0,0,0), apply the scaling, then move them back
    // moveTogether(-aabb.center());
    _vertices.row(0) *= scaling_factor_x;
    _vertices.row(1) *= scaling_factor_y;
    _vertices.row(2) *= scaling_factor_z;

    _mesh_origin[0] *= scaling_factor_x;
    _mesh_origin[1] *= scaling_factor_y;
    _mesh_origin[2] *= scaling_factor_z;
    // moveTogether(aabb.center());

    // scale the unrotated size
    _unrotated_size_xyz[0] *= scaling_factor_x;
    _unrotated_size_xyz[1] *= scaling_factor_y;
    _unrotated_size_xyz[2] *= scaling_factor_z;
}

void Mesh::moveTogether(const Vec3r& delta)
{
    _vertices.colwise() += delta;
    _mesh_origin += delta;
}

void Mesh::moveSeparate(const VerticesMat& delta)
{
    _vertices.noalias() += delta;
}

void Mesh::moveTo(const Vec3r& position)
{

    // calculate the required position offset based on the current center of the AABB
    const AABB aabb = boundingBox();
    const Vec3r offset = position - aabb.center();

    // apply the position offset
    moveTogether(offset);
}

void Mesh::rotateAbout(const Vec3r& p, const Vec3r& xyz_angles)
{
    const Real x = xyz_angles(0) * M_PI / 180.0;
    const Real y = xyz_angles(1) * M_PI / 180.0;
    const Real z = xyz_angles(2) * M_PI / 180.0;
    // using the "123" convention: rotate first about x axis, then about y, then about z
    Mat3r rot_mat;
    rot_mat(0,0) = std::cos(y) * std::cos(z);
    rot_mat(0,1) = std::sin(x)*std::sin(y)*std::cos(z) - std::cos(x)*std::sin(z);
    rot_mat(0,2) = std::cos(x)*std::sin(y)*std::cos(z) + std::sin(x)*std::sin(z);

    rot_mat(1,0) = std::cos(y)*std::sin(z);
    rot_mat(1,1) = std::sin(x)*std::sin(y)*std::sin(z) + std::cos(x)*std::cos(z);
    rot_mat(1,2) = std::cos(x)*std::sin(y)*std::sin(z) - std::sin(x)*std::cos(z);

    rot_mat(2,0) = -std::sin(y);
    rot_mat(2,1) = std::sin(x)*std::cos(y);
    rot_mat(2,2) = std::cos(x)*std::cos(y);
    
    rotateAbout(p, rot_mat);
}

void Mesh::rotateAbout(const Vec3r& p, const Mat3r& rot_mat)
{
    moveTogether(-p);
    _vertices = rot_mat * _vertices;
    _mesh_origin = rot_mat * _mesh_origin;
    moveTogether(p);
}

std::tuple<Real, Vec3r, Mat3r> Mesh::massProperties(Real density) const
{
    // uses the algorithm described here: http://number-none.com/blow/inertia/index.html
    Real total_volume = 0;
    Vec3r weighted_volume(0,0,0);
    Mat3r covariance = Mat3r::Zero();

    // covariance of "canonical" tetrahedron which is (0,0,0), (1,0,0), (0,1,0), (0,0,1)
    Mat3r C_canonical;
    C_canonical <<  1.0/60.0, 1.0/120.0, 1.0/120.0,
                    1.0/120.0, 1.0/60.0, 1.0/120.0,
                    1.0/120.0, 1.0/120.0, 1.0/60.0;
    for (const auto& f : _faces.colwise())
    {
        // each triangle in the mesh + origin forms a tetrahedron
        // v0=origin, v1=f[0], v2=f[1], v3=f[2]
        const Vec3r v0(0,0,0);
        const Vec3r v1 = _vertices.col(f[0]);
        const Vec3r v2 = _vertices.col(f[1]);
        const Vec3r v3 = _vertices.col(f[2]);

        // tet basis matrix
        Mat3r A;
        A.col(0) = (v1 - v0);
        A.col(1) = (v2 - v0);
        A.col(2) = (v3 - v0);

        // find signed volume of tet
        const Real volume = A.determinant() / 6.0;

        // calculate the center of mass of this tetrahedron - just average of 4 vertices
        const Vec3r tet_cm = 0.25*(v0 + v1 + v2 + v3);
        // update overall center of mass using a weighted average
        weighted_volume += tet_cm*volume;

        // add covariance matrix from this tet
        covariance += A.determinant() * A * C_canonical * A.transpose();
        // update overall volume
        total_volume += volume;
    }

    Vec3r center_of_mass = weighted_volume / total_volume;

    // move covariance matrix to center of mass
    covariance = covariance + total_volume * ( 2*(-center_of_mass) * (center_of_mass).transpose() + (-center_of_mass)*(-center_of_mass).transpose());

    // compute moment of inertia tensor from covariance mat
    const Mat3r I = Mat3r::Identity() * covariance.trace() - covariance;

    return std::tuple<Real, Vec3r, Mat3r>(density*total_volume, center_of_mass, density*I);
}

Vec3r Mesh::massCenter() const
{
    Real total_volume = 0;
    Vec3r weighted_volume(0,0,0);
    for (const auto& f : _faces.colwise())
    {
        // each triangle in the mesh + origin forms a tetrahedron
        // v0=origin, v1=f[0], v2=f[1], v3=f[2]
        const Vec3r v0(0,0,0);
        const Vec3r v1 = _vertices.col(f[0]);
        const Vec3r v2 = _vertices.col(f[1]);
        const Vec3r v3 = _vertices.col(f[2]);

        // tet basis matrix
        Mat3r A;
        A.col(0) = (v1 - v0);
        A.col(1) = (v2 - v0);
        A.col(2) = (v3 - v0);

        // find signed volume of tet
        const Real volume = A.determinant() / 6.0;

        // calculate the center of mass of this tetrahedron - just average of 4 vertices
        const Vec3r tet_cm = 0.25*(v0 + v1 + v2 + v3);
        // update overall center of mass using a weighted average
        // if (total_volume + volume > 0)
        //     center_of_mass = (center_of_mass*total_volume + tet_cm*volume) / (total_volume + volume);
        weighted_volume += tet_cm * volume;

        // update overall volume
        total_volume += volume;
    }

    return weighted_volume / total_volume;
}

void Mesh::writeMeshToObjFile(const std::string& filename) const
{
    std::ofstream obj_file(filename);
    if (obj_file.is_open())
    {
        for (const auto& v : _vertices.colwise())
        {
            obj_file << "v " << v[0] << " " << v[1] << " " << v[2] << std::endl;
        }
        
        for (const auto& f : _faces.colwise())
        {
            obj_file << "f " << f[0]+1 << " " << f[1]+1 << " " << f[2]+1 << std::endl;
        }
    }
}

#ifdef HAVE_CUDA
void Mesh::createGPUResource()
{
    _gpu_resource = std::make_unique<Sim::MeshGPUResource>(this);
    _gpu_resource->allocate();
}
#endif

} // namespace Geometry