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

    _vertex_to_elements_map = other._vertex_to_elements_map;
    _edge_to_elements_map = other._edge_to_elements_map;
    _face_to_elements_map = other._face_to_elements_map;

    _surface_face_to_element_map = other._surface_face_to_element_map;
    _element_to_surface_faces_map = other._element_to_surface_faces_map;

    _element_inv_undeformed_basis = other._element_inv_undeformed_basis;
    _element_rest_volumes = other._element_rest_volumes;
    
}

TetMesh::TetMesh(TetMesh&& other)
    : Mesh(other)
{
    _elements = std::move(other._elements);
    _vertex_to_elements_map = std::move(other._vertex_to_elements_map);
    _edge_to_elements_map = std::move(other._edge_to_elements_map);
    _face_to_elements_map = std::move(other._face_to_elements_map);

    _surface_face_to_element_map = std::move(other._surface_face_to_element_map);
    _element_to_surface_faces_map = other._element_to_surface_faces_map;

    _element_inv_undeformed_basis = std::move(other._element_inv_undeformed_basis);
    _element_rest_volumes = std::move(other._element_rest_volumes);
    
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

    // update maps for vertices -> elements, edges -> elements, faces -> elements
    _vertex_to_elements_map.resize(numVertices());
    for (int i = 0; i < numElements(); i++)
    {
        const Eigen::Vector4i& elem = element(i);
        // vertex -> elements map
        for (int k = 0; k < 4; k++)
            _vertex_to_elements_map[elem[k]].push_back(i);

        // edge -> elements map
        for (int k1 = 0; k1 < 4; k1++)
            for (int k2 = k1+1; k2 < 4; k2++)
                _edge_to_elements_map.insert({Edge(elem[k1], elem[k2]), i});

        // face -> elements map
        _face_to_elements_map.insert({Face(elem[0], elem[1], elem[2]), i});
        _face_to_elements_map.insert({Face(elem[0], elem[1], elem[3]), i});
        _face_to_elements_map.insert({Face(elem[0], elem[2], elem[3]), i});
        _face_to_elements_map.insert({Face(elem[1], elem[2], elem[3]), i});
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
    
    // find surface elements
    // for now, just do a dumb O(n^2) search
    _surface_face_to_element_map.clear();
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
                _surface_face_to_element_map.push_back(j);
                _element_to_surface_faces_map.insert({j, i});
                break;
            }
        }
    }

    // make sure that we found an element that corresponds to each surface face
    if (_surface_face_to_element_map.size() != static_cast<unsigned>(numFaces()))
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

std::vector<int> TetMesh::faceAdjacentElements(int element_index)
{
    const Vec4i& elem = element(element_index);

    std::vector<int> adjacent_elements;
    adjacent_elements.reserve(4);

    // F012
    {
        auto range = _face_to_elements_map.equal_range(Face(elem[0], elem[1], elem[2]));
        for (auto it = range.first; it != range.second; it++)
        {
            if (it->second != element_index)
            {
                adjacent_elements.push_back(it->second);
                break;  // only 1 other possible element - if we've found it, break
            }
        }
    }

    // F013
    {
        auto range = _face_to_elements_map.equal_range(Face(elem[0], elem[1], elem[3]));
        for (auto it = range.first; it != range.second; it++)
        {
            if (it->second != element_index)
            {
                adjacent_elements.push_back(it->second);
                break;  // only 1 other possible element - if we've found it, break
            }
        }
    }

    // F023
    {
        auto range = _face_to_elements_map.equal_range(Face(elem[0], elem[2], elem[3]));
        for (auto it = range.first; it != range.second; it++)
        {
            if (it->second != element_index)
            {
                adjacent_elements.push_back(it->second);
                break;  // only 1 other possible element - if we've found it, break
            }
        }
    }

    // F123
    {
        auto range = _face_to_elements_map.equal_range(Face(elem[1], elem[2], elem[3]));
        for (auto it = range.first; it != range.second; it++)
        {
            if (it->second != element_index)
            {
                adjacent_elements.push_back(it->second);
                break;  // only 1 other possible element - if we've found it, break
            }
        }
    }

    return adjacent_elements;
}

void TetMesh::_updateElementMapsForNewElement(int element_index)
{
    const Vec4i& elem = element(element_index);
    // update vertex -> element map
    // first make sure it has enough space
    _vertex_to_elements_map.resize(_vertices.totalSize());
    for (int k = 0; k < 4; k++)
    {
        _vertex_to_elements_map[elem[k]].push_back(element_index);
    }

    // update edge -> element map
    for (int k1 = 0; k1 < 4; k1++)
    {
        for (int k2 = 0; k2 < 4; k2++)
        {
            _edge_to_elements_map.insert({Edge(elem[k1], elem[k2]), element_index});
        }
    }

    // update face -> element map
    _face_to_elements_map.insert({Face(elem[0], elem[1], elem[2]), element_index});
    _face_to_elements_map.insert({Face(elem[0], elem[1], elem[3]), element_index});
    _face_to_elements_map.insert({Face(elem[0], elem[2], elem[3]), element_index});
    _face_to_elements_map.insert({Face(elem[1], elem[2], elem[3]), element_index});
}

void TetMesh::_updateElementMapsForRemovedElement(int element_index)
{
    const Vec4i& elem_to_remove = element(element_index);

    // vertex -> element mappings
    for (int k = 0; k < 4; k++)
    {
        std::vector<int>& vk_map = _vertex_to_elements_map[elem_to_remove[k]];
        vk_map.erase(
            std::remove(vk_map.begin(), vk_map.end(), element_index), vk_map.end()
        );
    }

    // edge -> element mappings
    for (int k1 = 0; k1 < 4; k1++)
    {
        for (int k2 = 0; k2 < 4; k2++)
        {
            auto range = _edge_to_elements_map.equal_range(Edge(elem_to_remove[k1], elem_to_remove[k2]));
            for (auto it = range.first; it != range.second; it++)
            {
                if (it->second == element_index) {
                    _edge_to_elements_map.erase(it);
                    break;
                }
            }
        }
    }

    // face -> element mappings
    // F012
    {
        auto range = _face_to_elements_map.equal_range(Face(elem_to_remove[0], elem_to_remove[1], elem_to_remove[2]));
        for (auto it = range.first; it != range.second; it++)
        {
            if (it->second == element_index) {
                _face_to_elements_map.erase(it);
                break;
            }
        }
    }
    // F013
    {
        auto range = _face_to_elements_map.equal_range(Face(elem_to_remove[0], elem_to_remove[1], elem_to_remove[3]));
        for (auto it = range.first; it != range.second; it++)
        {
            if (it->second == element_index) {
                _face_to_elements_map.erase(it);
                break;
            }
        }
    }
    // F023
    {
        auto range = _face_to_elements_map.equal_range(Face(elem_to_remove[0], elem_to_remove[2], elem_to_remove[3]));
        for (auto it = range.first; it != range.second; it++)
        {
            if (it->second == element_index) {
                _face_to_elements_map.erase(it);
                break;
            }
        }
    }
    // F123
    {
        auto range = _face_to_elements_map.equal_range(Face(elem_to_remove[1], elem_to_remove[2], elem_to_remove[3]));
        for (auto it = range.first; it != range.second; it++)
        {
            if (it->second == element_index) {
                _face_to_elements_map.erase(it);
                break;
            }
        }
    }
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
    const std::vector<int>& adjacent_elements = faceAdjacentElements(elem_index);

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


        size_t new_face_index = _faces.push_back(std::move(new_face));

        // update surface elements vector
        _surface_face_to_element_map.resize(_faces.totalSize());
        _surface_face_to_element_map[new_face_index] = adj_elem_index;

        _element_to_surface_faces_map.insert({adj_elem_index, new_face_index});

        // update vertex -> element, edge -> element, and face -> element maps
        _updateElementMapsForRemovedElement(elem_index);
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