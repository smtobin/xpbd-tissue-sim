#include "geometry/Mesh.hpp"

#include <set>
#include <iostream>
#include <fstream>

#ifdef HAVE_CUDA
#include "gpu/resource/MeshGPUResource.hpp"
#endif

namespace Geometry
{

std::ostream& operator<<(std::ostream& os, const Edge& edge) {
    os << "(" << edge.index1 << ", " << edge.index2 << ")";
    return os;
}

std::ostream& operator<<(std::ostream& os, const Face& face) {
    os << "(" << face.index1 << ", " << face.index2 << ", " << face.index3 << ")";
    return os;
}

Mesh::Mesh(const std::vector<Vec3r>& vertices, const std::vector<Vec3i>& faces)
    : _vertices(vertices), _faces(faces)
{
    // create surface vertex property
    addVertexProperty<bool>("surface");
    auto& surface_property = getVertexProperty<bool>("surface");
    for (const auto& f : _faces)
    {
        surface_property.set(f[0], true);
        surface_property.set(f[1], true);
        surface_property.set(f[2], true);
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
    for (const auto& cur_face : _faces)
    {
        std::unordered_set<int>& adj_verts0 = _vertex_adjacent_vertices[cur_face[0]];
        std::unordered_set<int>& adj_verts1 = _vertex_adjacent_vertices[cur_face[1]];
        std::unordered_set<int>& adj_verts2 = _vertex_adjacent_vertices[cur_face[2]];

        // for v0
        adj_verts0.insert(cur_face[1]);
        adj_verts0.insert(cur_face[2]);

        // for v1
        adj_verts1.insert(cur_face[0]);
        adj_verts1.insert(cur_face[2]);

        // for v2
        adj_verts2.insert(cur_face[0]);
        adj_verts2.insert(cur_face[1]);
    }
}

void Mesh::setCurrentStateAsUndeformedState()
{
    AABB bbox = boundingBox();
    _unrotated_size_xyz = bbox.size();

    _computeAdjacentVertices();
    updateVertexNormals();

    // set the initial vertices
    _initial_vertices.resize(_vertices.totalSize());
    for (unsigned i = 0; i < _vertices.totalSize(); i++)
    {
        _initial_vertices[i] = _vertices[i];
    }
}

void Mesh::updateVertexNormals()
{
    // make sure we have enough space
    _vertex_normals.resize(_vertices.totalSize());

    // zero out all normals
    for (const auto& vert_index : _vertices.validIndices())
    {
        _vertex_normals[vert_index] = Vec3r::Zero();
    }
        
    // iterate through faces and add normal contributions to vertices
    for (const auto& f : _faces)
    {
        const Vec3r& v0 = vertex(f[0]);
        const Vec3r& v1 = vertex(f[1]);
        const Vec3r& v2 = vertex(f[2]);

        // edge 0->1
        const Vec3r e01 = v1 - v0;
        // edge 1->2
        const Vec3r e12 = v2 - v1;
        // edge 2->0
        const Vec3r e20 = v0 - v2;

        // edge magnitudes
        Real e01_mag = e01.norm();
        Real e12_mag = e12.norm();
        Real e20_mag = e20.norm();

        // approximate angle at each vertex
        Real w0 = 1.0 / (e01_mag * e20_mag + 1e-12);
        Real w1 = 1.0 / (e12_mag * e01_mag + 1e-12);
        Real w2 = 1.0 / (e20_mag * e12_mag + 1e-12);

        // face normal
        const Vec3r n = -e01.cross(e20);    // negative because using e20 here

        _vertex_normals[f[0]] += w0 * n;
        _vertex_normals[f[1]] += w1 * n;
        _vertex_normals[f[2]] += w2 * n;
    }

    for (const auto& vert_index : _vertices.validIndices())
    {
        _vertex_normals[vert_index] = _vertex_normals[vert_index].normalized();
    }
}

Real* Mesh::vertexPointer(int index) const
{
    Real* p = const_cast<Real*>(_vertices.at(index).data());
    return p;
}

AABB Mesh::boundingBox() const
{
    Vec3r min = Vec3r::Constant(std::numeric_limits<Real>::max());
    Vec3r max = Vec3r::Constant(std::numeric_limits<Real>::lowest());
    for (const auto& v : _vertices)
    {
        min[0] = std::min(v[0], min[0]);
        min[1] = std::min(v[1], min[1]);
        min[2] = std::min(v[2], min[2]);

        max[0] = std::max(v[0], max[0]);
        max[1] = std::max(v[1], max[1]);
        max[2] = std::max(v[2], max[2]);
    }
    return AABB(min, max);
}

std::pair<int,Real> Mesh::averageFaceEdgeLength() const
{
    // keep track of unique edges, stored in ascending-index order
    std::set<std::pair<int, int>> edges;

    // add all the unique edges to the set
    for (const auto& face : _faces)
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
    for (const auto& index : _vertices.validIndices())
    {
        Real dist = (p - vertex(index)).squaredNorm();
        if (dist < closest_dist)
        {
            closest_index = index;
            closest_dist = dist;
        }
    }

    return closest_index;

}

std::vector<int> Mesh::getVerticesWithX(Real x) const
{
    std::vector<int> verts;
    for (const auto& index : _vertices.validIndices())
    {
        if (vertex(index)[0] == x)
        {
            verts.push_back(index);
        }
    }

    return verts;
}

std::vector<int> Mesh::getVerticesWithY(Real y) const
{
    std::vector<int> verts;
    for (const auto& index : _vertices.validIndices())
    {
        if (vertex(index)[1] == y)
        {
            verts.push_back(index);
        }
    }

    return verts;
}

std::vector<int> Mesh::getVerticesWithZ(Real z) const
{
    std::vector<int> verts;
    for (const auto& index : _vertices.validIndices())
    {
        if (vertex(index)[2] == z)
        {
            verts.push_back(index);
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
    // moveTogether(-aabb.center());    /** TODO: why is this commented out? */
    for (auto& v : _vertices)
        v *= scaling_factor;

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
    for (auto& v : _vertices)
    {
        v[0] *= scaling_factor_x;
        v[1] *= scaling_factor_y;
        v[2] *= scaling_factor_z;
    }

    for (auto& v : _initial_vertices)
    {
        v[0] *= scaling_factor_x;
        v[1] *= scaling_factor_y;
        v[2] *= scaling_factor_z;
    }

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
    for (auto& v : _vertices)
        v += delta;

    for (auto& v : _initial_vertices)
        v += delta;
        
    _mesh_origin += delta;
}

// void Mesh::moveSeparate(const VerticesMat& delta)
// {
//     _vertices.noalias() += delta;
// }

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
    for (auto& v : _vertices)
        v = rot_mat * v;

    for (auto& v : _initial_vertices)
        v = rot_mat * v;

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
    for (const auto& f : _faces)
    {
        // each triangle in the mesh + origin forms a tetrahedron
        // v0=origin, v1=f[0], v2=f[1], v3=f[2]
        const Vec3r v0(0,0,0);
        const Vec3r v1 = vertex(f[0]);
        const Vec3r v2 = vertex(f[1]);
        const Vec3r v3 = vertex(f[2]);

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
    for (const auto& f : _faces)
    {
        // each triangle in the mesh + origin forms a tetrahedron
        // v0=origin, v1=f[0], v2=f[1], v3=f[2]
        const Vec3r v0(0,0,0);
        const Vec3r v1 = vertex(f[0]);
        const Vec3r v2 = vertex(f[1]);
        const Vec3r v3 = vertex(f[2]);

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
        for (const auto& v : _vertices)
        {
            obj_file << "v " << v[0] << " " << v[1] << " " << v[2] << std::endl;
        }
        
        for (const auto& f : _faces)
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

void Mesh::serialize(std::vector<std::byte>& buf) const
{
    pack(buf, _vertices);
    pack(buf, _faces);
    pack(buf, _vertex_normals);
    pack(buf, _initial_vertices);
    pack(buf, _vertex_adjacent_vertices);
    pack(buf, _unrotated_size_xyz);
    pack(buf, _mesh_origin);
    pack(buf, _vertex_properties);
    pack(buf, _face_properties);
    pack(buf, _topology_version);
}

void Mesh::deserialize(const std::byte*& buf)
{
    unpack(buf, _vertices);
    unpack(buf, _faces);
    unpack(buf, _vertex_normals);
    unpack(buf, _initial_vertices);
    unpack(buf, _vertex_adjacent_vertices);
    unpack(buf, _unrotated_size_xyz);
    unpack(buf, _mesh_origin);
    unpack(buf, _vertex_properties);
    unpack(buf, _face_properties);
    unpack(buf, _topology_version);
}

} // namespace Geometry