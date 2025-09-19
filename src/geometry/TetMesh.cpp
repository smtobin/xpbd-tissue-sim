#include "geometry/TetMesh.hpp"

#include "common/colors.hpp"

#include <set>

#ifdef HAVE_CUDA
#include "gpu/resource/TetMeshGPUResource.hpp"
#endif

namespace Geometry
{

TetMesh::TetMesh(const std::vector<Vec3r>& vertices, const std::vector<Vec3i>& faces, const std::vector<Vec4i>& elements)
    : Mesh(vertices, faces), _elements(elements)
{
    setCurrentStateAsUndeformedState();
}

TetMesh::TetMesh(const TetMesh& other)
    : Mesh(other)
{
    _elements = other._elements;
    _attached_elements_to_vertex = other._attached_elements_to_vertex;
    _element_inv_undeformed_basis = other._element_inv_undeformed_basis;
    _element_rest_volumes = other._element_rest_volumes;
    _surface_elements = other._surface_elements;
}

TetMesh::TetMesh(TetMesh&& other)
    : Mesh(other)
{
    _elements = std::move(other._elements);
    _attached_elements_to_vertex = std::move(other._attached_elements_to_vertex);
    _element_inv_undeformed_basis = std::move(other._element_inv_undeformed_basis);
    _element_rest_volumes = std::move(other._element_rest_volumes);
    _surface_elements = std::move(other._surface_elements);
}

void TetMesh::_computeAdjacentVertices()
{
    _vertex_adjacent_vertices.resize(numVertices());
    
    // clear all the adjacency lists
    for (int i = 0; i < numVertices(); i++)
    {
        _vertex_adjacent_vertices[i].clear();
    }

    // go through each of the faces and add adjacent vertices for each vertex in the face
    // even though std::vector is slow for this, we only do this once
    for (int i = 0; i < numElements(); i++)
    {
        const Eigen::Vector4i& cur_element = element(i);

        std::vector<int>& adj_verts0 = _vertex_adjacent_vertices[cur_element[0]];
        std::vector<int>& adj_verts1 = _vertex_adjacent_vertices[cur_element[1]];
        std::vector<int>& adj_verts2 = _vertex_adjacent_vertices[cur_element[2]];
        std::vector<int>& adj_verts3 = _vertex_adjacent_vertices[cur_element[3]];

        // for v0
        if (std::find(adj_verts0.begin(), adj_verts0.end(), cur_element[1]) == adj_verts0.end())    adj_verts0.push_back(cur_element[1]);
        if (std::find(adj_verts0.begin(), adj_verts0.end(), cur_element[2]) == adj_verts0.end())    adj_verts0.push_back(cur_element[2]);
        if (std::find(adj_verts0.begin(), adj_verts0.end(), cur_element[3]) == adj_verts0.end())    adj_verts0.push_back(cur_element[3]);

        // for v1
        if (std::find(adj_verts1.begin(), adj_verts1.end(), cur_element[0]) == adj_verts1.end())   adj_verts1.push_back(cur_element[0]);
        if (std::find(adj_verts1.begin(), adj_verts1.end(), cur_element[2]) == adj_verts1.end())   adj_verts1.push_back(cur_element[2]);
        if (std::find(adj_verts1.begin(), adj_verts1.end(), cur_element[3]) == adj_verts1.end())   adj_verts1.push_back(cur_element[3]);

        // for v2
        if (std::find(adj_verts2.begin(), adj_verts2.end(), cur_element[0]) == adj_verts2.end())   adj_verts2.push_back(cur_element[0]);
        if (std::find(adj_verts2.begin(), adj_verts2.end(), cur_element[1]) == adj_verts2.end())   adj_verts2.push_back(cur_element[1]);
        if (std::find(adj_verts2.begin(), adj_verts2.end(), cur_element[3]) == adj_verts2.end())   adj_verts2.push_back(cur_element[3]);

        // for v3
        if (std::find(adj_verts3.begin(), adj_verts3.end(), cur_element[0]) == adj_verts3.end())   adj_verts3.push_back(cur_element[0]);
        if (std::find(adj_verts3.begin(), adj_verts3.end(), cur_element[1]) == adj_verts3.end())   adj_verts3.push_back(cur_element[1]);
        if (std::find(adj_verts3.begin(), adj_verts3.end(), cur_element[2]) == adj_verts3.end())   adj_verts3.push_back(cur_element[2]);
    }
}

void TetMesh::setCurrentStateAsUndeformedState()
{
    Mesh::setCurrentStateAsUndeformedState();

    // compute mesh properties
    _attached_elements_to_vertex.resize(numVertices());
    for (int i = 0; i < numElements(); i++)
    {
        const Eigen::Vector4i& elem = element(i);
        _attached_elements_to_vertex[elem[0]].push_back(i);
        _attached_elements_to_vertex[elem[1]].push_back(i);
        _attached_elements_to_vertex[elem[2]].push_back(i);
        _attached_elements_to_vertex[elem[3]].push_back(i);
    }

    // inverse undeformed basis for each element
    _element_inv_undeformed_basis.resize(numElements());
    for (int i = 0; i < numElements(); i++)
    {
        const Eigen::Vector4i& elem = element(i);
        const Vec3r& v1 = vertex(elem[0]);
        const Vec3r& v2 = vertex(elem[1]);
        const Vec3r& v3 = vertex(elem[2]);
        const Vec3r& v4 = vertex(elem[3]);

        Mat3r X;
        X.col(0) = (v1 - v4);
        X.col(1) = (v2 - v4);
        X.col(2) = (v3 - v4);

        _element_inv_undeformed_basis[i] = X.inverse();
    }

    // element rest volumes
    _element_rest_volumes.resize(numElements());
    for (int i = 0; i < numElements(); i++)
    {
        _element_rest_volumes[i] = elementVolume(i);
    }

    // find elements that share faces
    // for now, just do a dumb O(n^2) search
    _face_adjacent_elements.resize(numElements());
    for (int i = 0; i < numElements(); i++)
    {
        const Eigen::Vector4i& elem_i = element(i);
        for (int j = i+1; j < numElements(); j++)
        {
            
            const Eigen::Vector4i& elem_j = element(j);
            int num_shared_vertices = 0;
            for (int k = 0; k < 4; k++)
            {
                if (elem_j[k] == elem_i[0] || elem_j[k] == elem_i[1] || elem_j[k] == elem_i[2] || elem_j[k] == elem_i[3])
                    num_shared_vertices++;
            }

            if (num_shared_vertices == 3)   // the two elements share a face
            {
                _face_adjacent_elements[i].push_back(j);
                _face_adjacent_elements[j].push_back(i);
            }
                
        }
    }

    // find surface elements
    // for now, just do a dumb O(n^2) search
    _surface_elements.clear();
    for (int i = 0; i < numFaces(); i++)
    {
        const Vec3i& f = face(i);
        // find the element that has this face
        for (int j = 0; j < numElements(); j++)
        {
            const Eigen::Vector4i& elem = element(j);
            if (    (f[0] == elem[0] || f[0] == elem[1] || f[0] == elem[2] || f[0] == elem[3]) &&
                    (f[1] == elem[0] || f[1] == elem[1] || f[1] == elem[2] || f[1] == elem[3]) &&
                    (f[2] == elem[0] || f[2] == elem[1] || f[2] == elem[2] || f[2] == elem[3]) )
            {
                _surface_elements.push_back(j);
                break;
            }
        }
    }

    // make sure that we found an element that corresponds to each surface face
    if (_surface_elements.size() != static_cast<unsigned>(numFaces()))
    {
        std::cerr << KRED << BOLD << "FATAL" << RST << KRED << ": Some surface faces do not have elements associated with them!" << RST << std::endl;
        std::cerr << "Double check your .msh file for floating faces." << std::endl;
        assert(0);
    }
    
}

Real TetMesh::elementVolume(int index) const
{
    const Eigen::Vector4i& elem = element(index);
    const Vec3r& v1 = vertex(elem[0]);
    const Vec3r& v2 = vertex(elem[1]);
    const Vec3r& v3 = vertex(elem[2]);
    const Vec3r& v4 = vertex(elem[3]);

    Mat3r X;
    X.col(0) = (v1 - v4);
    X.col(1) = (v2 - v4);
    X.col(2) = (v3 - v4);

    return std::abs(X.determinant() / 6.0);
}

Mat3r TetMesh::elementDeformationGradient(int index) const
{
    const Eigen::Vector4i& elem = element(index);
    const Vec3r& v1 = vertex(elem[0]);
    const Vec3r& v2 = vertex(elem[1]);
    const Vec3r& v3 = vertex(elem[2]);
    const Vec3r& v4 = vertex(elem[3]);

    Mat3r deformed_basis;
    deformed_basis.col(0) = (v1 - v4);
    deformed_basis.col(1) = (v2 - v4);
    deformed_basis.col(2) = (v3 - v4);

    return deformed_basis * _element_inv_undeformed_basis[index];
}

void TetMesh::removeElementWithFace(int face_index)
{
    // get the element corresponding to the surface face
    int elem_index = elementWithFace(face_index);
    removeElement(elem_index);
}

void TetMesh::removeElement(int elem_index)
{
    // get adjacent elements
    const std::vector<int>& adjacent_elements = _face_adjacent_elements[elem_index];

    // get surface faces associated with the element we're removing
    // for now, just brute force search through the surface faces
    std::vector<int> surface_faces_on_removed_element;
    for (const auto& index : _faces.validIndices())
    {
        if (elementWithFace(index) == elem_index)
        {
            surface_faces_on_removed_element.push_back(index);
        }
    }

    // remove surface faces
    for (const auto& index : surface_faces_on_removed_element)
    {
        _faces.erase(index);
    }

    // add new surface faces
    // these are the faces that are shared with the adjacent element(s)
    const Vec4i& elem_to_remove = element(elem_index);
    for (const auto& adj_elem_index : adjacent_elements)
    {
        const Vec4i& adj_elem = element(adj_elem_index);
        int index_in_face = 0;
        Vec3i new_face;
        int elem_vertex_not_in_new_face = -1;
        for (const auto& adj_vert_index : adj_elem)
        {
            bool found = false;
            for (const auto& vert_index : elem_to_remove)
            {
                if (adj_vert_index == vert_index)
                {
                    new_face[index_in_face++] = vert_index;
                    found = true;
                    break;
                }
            }
            if (!found) elem_vertex_not_in_new_face = adj_vert_index;
        }
        assert(index_in_face == 3);
        // make sure normal is correct
        // edge 0->1
        const Vec3r e01 = vertex(new_face[1]) - vertex(new_face[0]);
        // edge 0->2
        const Vec3r e02 = vertex(new_face[2]) - vertex(new_face[0]);
        const Vec3r n = e01.cross(e02);

        // the dot product of the new face normal and the vertex of the element that is not in this new face should be positive
        // (assuming the element is not inverted)
        // if it's not, simply flip vertices 1 and 2 in the new face
        if (n.dot(vertex(elem_vertex_not_in_new_face)) < 0)
        {
            int tmp = new_face[1];
            new_face[1] = new_face[2];
            new_face[2] = tmp;
        }


        _faces.push_back(std::move(new_face));
    }

    // remove element
    _elements.erase(elem_index);
}

std::pair<int, Real> TetMesh::averageTetEdgeLength() const
{
    std::set<std::pair<int, int>> edges;

    auto make_edge = [] (int v1, int v2)
    {
        if (v1 > v2)
            return std::pair<int, int>(v2, v1);
        else 
            return std::pair<int, int>(v1, v2);
    };

    Real total_length = 0;
    for (const auto& elem : _elements)
    {
        const Vec3r& v1 = vertex(elem(0));
        const Vec3r& v2 = vertex(elem(1));
        const Vec3r& v3 = vertex(elem(2));
        const Vec3r& v4 = vertex(elem(3));

        std::pair<int, int> e1 = make_edge(elem(0), elem(1));
        std::pair<int, int> e2 = make_edge(elem(0), elem(2));
        std::pair<int, int> e3 = make_edge(elem(0), elem(3));
        std::pair<int, int> e4 = make_edge(elem(1), elem(2));
        std::pair<int, int> e5 = make_edge(elem(1), elem(3));
        std::pair<int, int> e6 = make_edge(elem(2), elem(3));
        

        if (edges.count(e1) == 0)
        {
            total_length += (v1-v2).norm();
            edges.insert(e1);
        }
        if (edges.count(e2) == 0)
        {
            total_length += (v1-v3).norm();
            edges.insert(e2);
        }
        if (edges.count(e3) == 0)
        {
            total_length += (v1-v4).norm();
            edges.insert(e3);
        }
        if (edges.count(e4) == 0)
        {
            total_length += (v2-v3).norm();
            edges.insert(e4);
        }
        if (edges.count(e5) == 0)
        {
            total_length += (v2-v4).norm();
            edges.insert(e5);
        }
        if (edges.count(e6) == 0)
        {
            total_length += (v3-v4).norm();
            edges.insert(e6);
        }
    }

    return std::pair<int,Real>(edges.size(), total_length/edges.size());
}

#ifdef HAVE_CUDA
void TetMesh::createGPUResource()
{
    _gpu_resource = std::make_unique<Sim::TetMeshGPUResource>(this);
    _gpu_resource->allocate();
}
#endif

} // namespace Geometry