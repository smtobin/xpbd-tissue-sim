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

void TetMesh::_computeAdjacentVertices()
{
    _vertex_adjacent_vertices.resize(_vertices.totalSize());
    
    // clear all the adjacency lists
    for (unsigned i = 0; i < _vertices.totalSize(); i++)
    {
        _vertex_adjacent_vertices[i].clear();
    }

    // go through each of the faces and add adjacent vertices for each vertex in the face
    // even though std::vector is slow for this, we only do this once
    for (const auto& elem_ind : _elements.validIndices())
    {
        const Eigen::Vector4i& cur_element = element(elem_ind);

        std::unordered_set<int>& adj_verts0 = _vertex_adjacent_vertices[cur_element[0]];
        std::unordered_set<int>& adj_verts1 = _vertex_adjacent_vertices[cur_element[1]];
        std::unordered_set<int>& adj_verts2 = _vertex_adjacent_vertices[cur_element[2]];
        std::unordered_set<int>& adj_verts3 = _vertex_adjacent_vertices[cur_element[3]];

        // for v0
        adj_verts0.insert({cur_element[1], cur_element[2], cur_element[3]});

        // for v1
        adj_verts1.insert({cur_element[0], cur_element[2], cur_element[3]});

        // for v2
        adj_verts2.insert({cur_element[0], cur_element[1], cur_element[3]});

        // for v3
        adj_verts3.insert({cur_element[0], cur_element[1], cur_element[2]});
    }
}

void TetMesh::setCurrentStateAsUndeformedState()
{
    Mesh::setCurrentStateAsUndeformedState();

    // update maps for vertices -> elements, edges -> elements, faces -> elements
    _vertex_to_elements_map.clear();
    _vertex_to_elements_map.resize(_vertices.totalSize());
    _edge_to_elements_map.clear();
    _face_to_elements_map.clear();
    for (const auto& elem_ind : _elements.validIndices())
    {
        const Eigen::Vector4i& elem = element(elem_ind);
        // vertex -> elements map
        for (int k = 0; k < 4; k++)
            _vertex_to_elements_map[elem[k]].push_back(elem_ind);

        // edge -> elements map
        for (int k1 = 0; k1 < 4; k1++)
            for (int k2 = k1+1; k2 < 4; k2++)
                _edge_to_elements_map.insert({Edge(elem[k1], elem[k2]), elem_ind});

        // face -> elements map
        _face_to_elements_map.insert({Face(elem[0], elem[1], elem[2]), elem_ind});
        _face_to_elements_map.insert({Face(elem[0], elem[1], elem[3]), elem_ind});
        _face_to_elements_map.insert({Face(elem[0], elem[2], elem[3]), elem_ind});
        _face_to_elements_map.insert({Face(elem[1], elem[2], elem[3]), elem_ind});
    }

    // inverse undeformed basis for each element
    _element_inv_undeformed_basis.resize(_elements.totalSize());
    for (const auto& elem_ind : _elements.validIndices())
    {
        const Eigen::Vector4i& elem = element(elem_ind);
        const Vec3r& v1 = vertex(elem[0]);
        const Vec3r& v2 = vertex(elem[1]);
        const Vec3r& v3 = vertex(elem[2]);
        const Vec3r& v4 = vertex(elem[3]);

        Mat3r X;
        X.col(0) = (v1 - v4);
        X.col(1) = (v2 - v4);
        X.col(2) = (v3 - v4);

        _element_inv_undeformed_basis[elem_ind] = X.inverse();
    }

    // element rest volumes
    _element_rest_volumes.resize(_elements.totalSize());
    for (const auto& elem_ind : _elements.validIndices())
    {
        _element_rest_volumes[elem_ind] = elementVolume(elem_ind);
    }

    // vertex rest volumes
    _vertex_rest_volumes.clear();
    _vertex_rest_volumes.resize(_vertices.totalSize(), 0);
    for (const auto& elem_ind : _elements.validIndices())
    {
        for (const auto& v : _elements[elem_ind])
            _vertex_rest_volumes[v] += 0.25 * _element_rest_volumes[elem_ind];
    }
    
    // find surface elements
    // for now, just do a dumb O(n^2) search
    _surface_face_to_element_map.clear();
    _surface_face_to_element_map.resize(_faces.totalSize(), -1);
    _element_to_surface_faces_map.clear();
    for (const auto& face_ind : _faces.validIndices())
    {
        const Vec3i& f = face(face_ind);
        // find the element that has this face
        for (const auto& elem_ind : _elements.validIndices())
        {
            const Eigen::Vector4i& elem = element(elem_ind);
            if (    (f[0] == elem[0] || f[0] == elem[1] || f[0] == elem[2] || f[0] == elem[3]) &&
                    (f[1] == elem[0] || f[1] == elem[1] || f[1] == elem[2] || f[1] == elem[3]) &&
                    (f[2] == elem[0] || f[2] == elem[1] || f[2] == elem[2] || f[2] == elem[3]) )
            {
                _surface_face_to_element_map[face_ind] = elem_ind;
                _element_to_surface_faces_map.insert({elem_ind, face_ind});
                break;
            }
        }
    }

    bool face_with_no_element = false;
    for (const auto& face_index : _faces.validIndices())
    {
        if (_surface_face_to_element_map[face_index] == -1)
        {
            std::cerr << "  Face " << face_index << " has no associated element." << std::endl;
            face_with_no_element = true;
        }
    }

    // make sure that we found an element that corresponds to each surface face
    if (face_with_no_element)
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

    return elementVolume(v1, v2, v3, v4);
}

Real TetMesh::elementVolume(const Vec3r& v1, const Vec3r& v2, const Vec3r& v3, const Vec3r& v4) const
{
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

std::vector<int> TetMesh::elementSurfaceFaces(int element_index) const
{
    std::vector<int> surface_faces;
    auto range = _element_to_surface_faces_map.equal_range(element_index);
    for (auto it = range.first; it != range.second; it++)
    {
        surface_faces.push_back(it->second);
    }

    return surface_faces;
}

Vec3r TetMesh::elementCentroid(int element_index) const
{
    const Vec4i& elem_verts = element(element_index);
    Vec3r sum = vertex(elem_verts[0]) + vertex(elem_verts[1]) + vertex(elem_verts[2]) + vertex(elem_verts[3]);
    return sum/4.0;
}

Vec3r TetMesh::elementInitialCentroid(int element_index) const
{
    const Vec4i& elem_verts = element(element_index);
    Vec3r sum = initialVertex(elem_verts[0]) + initialVertex(elem_verts[1]) + initialVertex(elem_verts[2]) + initialVertex(elem_verts[3]);
    return sum/4.0;
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
        for (int k2 = k1+1; k2 < 4; k2++)
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
    // vertex -> element mappings
    _updateVertexElementMapForRemovedElement(element_index);

    // edge -> element mappings
    _updateEdgeElementMapForRemovedElement(element_index);

    // face -> element mappings
    _updateFaceElementMapForRemovedElement(element_index);

    // element -> surface faces map
    _updateElementSurfaceFaceMapForRemovedElement(element_index);
}

void TetMesh::_updateVertexElementMapForRemovedElement(int element_index)
{
    const Vec4i& elem_to_remove = element(element_index);
    for (int k = 0; k < 4; k++)
    {
        std::vector<int>& vk_map = _vertex_to_elements_map[elem_to_remove[k]];
        vk_map.erase(
            std::remove(vk_map.begin(), vk_map.end(), element_index), vk_map.end()
        );

        // if there are no other elements associated with this vertex, remove it
        if (vk_map.size() == 0)
        {
            _vertices.erase(elem_to_remove[k]);

            // vertex no longer in mesh, so clear its adjacent vertices list
            _vertex_adjacent_vertices[elem_to_remove[k]].clear();
        }
    }
}

void TetMesh::_updateEdgeElementMapForRemovedElement(int element_index)
{
    const Vec4i& elem_to_remove = element(element_index);
    for (int k1 = 0; k1 < 4; k1++)
    {
        for (int k2 = k1+1; k2 < 4; k2++)
        {
            // get all elements that currently share the edge (this will include the element we are removing)
            Edge edge(elem_to_remove[k1], elem_to_remove[k2]);
            auto range = _edge_to_elements_map.equal_range(edge);
            int num_elements_with_edge = std::distance(range.first, range.second);
            
            // if only one element (i.e. the one we are removing) shares this edge, we need to update the adjacent vertices list
            if (num_elements_with_edge <= 1)
            {
                _vertex_adjacent_vertices[edge.index1].erase(edge.index2);
                _vertex_adjacent_vertices[edge.index2].erase(edge.index1);
            }

            // remove the element from being associated with the edge
            for (auto it = range.first; it != range.second; it++)
            {
                if (it->second == element_index) {
                    _edge_to_elements_map.erase(it);
                    break;
                }
            }
        }
    }
}

void TetMesh::_updateFaceElementMapForRemovedElement(int element_index)
{
    const Vec4i& elem_to_remove = element(element_index);
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

void TetMesh::_updateElementSurfaceFaceMapForRemovedElement(int element_index)
{
    _element_to_surface_faces_map.erase(element_index);
}

int TetMesh::_addFace(const Vec3i& new_face, int elem_with_face)
{
    // the new face (nominally) is the same as the element face
    // but we likely have to flip the normal
    Vec3i new_face_copy = new_face;

    const Vec4i& elem_verts = element(elem_with_face);

    // get the vertex of this adjacent element that is not in the face
    int elem_4th_vertex = -1;
    for (const auto& v : elem_verts)
    {
        if (v == new_face[0] || v == new_face[1] || v == new_face[2])
            continue;
        
        elem_4th_vertex = v;
        break;
    }
    assert(elem_4th_vertex != -1);

    // make sure normal is correct
    const Vec3r& v0 = vertex(new_face[0]);
    const Vec3r& v1 = vertex(new_face[1]);
    const Vec3r& v2 = vertex(new_face[2]);
    
    // edge 0->1
    const Vec3r e01 = v1 - v0;
    // edge 0->2
    const Vec3r e02 = v2 - v0;
    const Vec3r n = e01.cross(e02);

    const Vec3r c = (v0 + v1 + v2) / 3.0;

    // the dot product of the new face normal and the vertex of the element that is not in this new face should be positive
    // (assuming the element is not inverted)
    // if it's not, simply flip vertices 1 and 2 in the new face
    if (n.dot(c - vertex(elem_4th_vertex)) < 0)
    {
        new_face_copy[1] = new_face[2];
        new_face_copy[2] = new_face[1];
    }

    // finally add the new face
    int new_face_index = _faces.push_back(new_face_copy);

    // update surface elements vector
    _surface_face_to_element_map.resize(_faces.totalSize());
    _surface_face_to_element_map[new_face_index] = elem_with_face;

    _element_to_surface_faces_map.insert({elem_with_face, new_face_index});

    // resize face properties after adding the face
    _face_properties.for_each_element([&](auto& prop) {
        prop.resize(_faces.totalSize());
    });

    return new_face_index;
}

void TetMesh::removeElementWithFace(int face_index)
{
    // get the element corresponding to the surface face
    int elem_index = elementWithFace(face_index);
    removeElement(elem_index);
}

TetMesh::RemovedElement TetMesh::removeElement(int elem_index)
{
    // increment topology version since the topology is changing
    _topology_version++;

    // update vertex volumes
    _updateVertexVolumesForRemovedElement(elem_index);
    
    // get adjacent elements
    // std::vector<int> adjacent_elements = faceAdjacentElements(elem_index);

    // remove any surface faces associated with the element we're removing
    auto surface_faces_range = _element_to_surface_faces_map.equal_range(elem_index);
    for (auto it = surface_faces_range.first; it != surface_faces_range.second; it++)
    {
        _faces.erase(it->second);
    }

    // add new surface faces
    // these are the faces that are shared with the adjacent element(s)
    auto add_surface_face = [&](const Vec3i& elem_face)
    {
        // create a Face object and query the face -> element map
        Face query_face(elem_face[0], elem_face[1], elem_face[2]);
        auto faces_range = _face_to_elements_map.equal_range(query_face);
        // see if there are any adjacent elements (i.e element indices that are not the element we are removing)
        int adj_elem_index = -1;
        for (auto it = faces_range.first; it != faces_range.second; it++)
        {
            if (it->second != elem_index)
            {
                adj_elem_index = it->second;
                break;
            }
        }

        // we didn't find any adjacent elements, so we're done here
        if (adj_elem_index == -1)
            return;

        // we have found an element that shares the face!
        const Vec4i& adj_elem = element(adj_elem_index);

        // finally add the face
        _addFace(elem_face, adj_elem_index);
    };


    // check if we need to add new faces for the 4 faces of the removed element
    const Vec4i& elem_to_remove = element(elem_index);
    add_surface_face(Vec3i(elem_to_remove[0], elem_to_remove[1], elem_to_remove[2]));
    add_surface_face(Vec3i(elem_to_remove[0], elem_to_remove[1], elem_to_remove[3]));
    add_surface_face(Vec3i(elem_to_remove[0], elem_to_remove[2], elem_to_remove[3]));
    add_surface_face(Vec3i(elem_to_remove[1], elem_to_remove[2], elem_to_remove[3]));

    // update vertex -> element, edge -> element, face -> element maps, element -> surface face maps
    _updateElementMapsForRemovedElement(elem_index);

    RemovedElement removed_element(
        elem_index, elem_to_remove, 
        elementCentroid(elem_index), elementInitialCentroid(elem_index), 
        elementRestVolume(elem_index)
    );

    _recently_removed_elements.push_back(removed_element);

    // remove element
    _elements.erase(elem_index);

    return removed_element;
}

void TetMesh::_updateVertexVolumesForRemovedElement(int element_index)
{
    const Vec4i& elem = element(element_index);
    Real rest_volume = elementRestVolume(element_index);
    for (const auto& v : elem)
    {
        _vertex_rest_volumes[v] -= 0.25*rest_volume;
    }
}

std::vector<TetMesh::RemovedElement> TetMesh::recentlyRemovedElements(bool clear_cache)
{
    if (clear_cache)
    {
        auto tmp = _recently_removed_elements;
        _recently_removed_elements.clear();
        return tmp;
    }
    else
    {
        return _recently_removed_elements;
    }
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

std::tuple<std::vector<int>, std::vector<Vec3i>, std::vector<int>> TetMesh::submeshForElementClass(int element_class)
{
    std::vector<int> class_vertices_vec;
    std::vector<Vec3i> class_faces_vec;
    std::vector<int> class_elements;
    if (!hasElementProperty<int>("class"))
        return {class_vertices_vec, class_faces_vec, class_elements};
    
    const MeshProperty<int>& class_eprop = getElementProperty<int>("class");

    std::unordered_set<int> class_vertices;
    std::unordered_set<Face, FaceHash> class_faces;
    

    // go through all elements and find the elements that have the specified class
    for (const auto& elem_index : _elements.validIndices())
    {
        if (class_eprop.get(elem_index) == element_class)
        {
            // add to list of elements of the desired class
            class_elements.push_back(elem_index);

            const Vec4i& elem = element(elem_index);

            // go through vertices of this element and add to the list of vertices
            for (int k = 0; k < 4; k++)
                class_vertices.insert(elem[k]);

            // check each face of the element to see if it is on the "surface" of the submesh
            // this is the case when (1) there is no other element that shares the faces or (2) the element that shares the face has a different class
            // (note that this assumes there is no refinement boundary within the submesh)
            auto check_face = [&] (const Face& query_face)
            {
                auto faces_range = _face_to_elements_map.equal_range(query_face);
                // see if there are any adjacent elements (i.e element indices that are not the element we are removing)
                int adj_elem_index = -1;
                for (auto it = faces_range.first; it != faces_range.second; it++)
                {
                    if (it->second != elem_index)
                    {
                        adj_elem_index = it->second;
                        break;
                    }
                }

                // we didn't find any adjacent elements, so the face is on the outer surface of the submesh
                if (adj_elem_index == -1)
                    class_faces.insert(query_face);
                // the adjacent element has a different class ==> face is on the outer surface of the submesh
                else if (class_eprop.get(adj_elem_index) != element_class)
                    class_faces.insert(query_face);
            };

            check_face( Face(elem[0], elem[1], elem[2]) );
            check_face( Face(elem[0], elem[2], elem[3]) );
            check_face( Face(elem[0], elem[1], elem[3]) );
            check_face( Face(elem[1], elem[2], elem[3]) );
        }
    } 

    // copy information from sets to vectors
    // vertices
    for (const auto& v : class_vertices)
    {
        class_vertices_vec.push_back(v);
    }

    // faces
    for (const auto& f : class_faces)
    {
        class_faces_vec.push_back(Vec3i(f.index1, f.index2, f.index3));
    }
    
    return {class_vertices_vec, class_faces_vec, class_elements};
}

#ifdef HAVE_CUDA
void TetMesh::createGPUResource()
{
    _gpu_resource = std::make_unique<Sim::TetMeshGPUResource>(this);
    _gpu_resource->allocate();
}
#endif

void TetMesh::serialize(std::vector<std::byte>& buf) const
{
    Mesh::serialize(buf);
    pack(buf, _elements);
    pack(buf, _element_properties);
    pack(buf, _element_rest_volumes);
    pack(buf, _element_inv_undeformed_basis);
    pack(buf, _vertex_rest_volumes);
    pack(buf, _surface_face_to_element_map);
    pack(buf, _element_to_surface_faces_map);
    pack(buf, _vertex_to_elements_map);
    pack(buf, _edge_to_elements_map);
    pack(buf, _face_to_elements_map);
    pack(buf, _recently_removed_elements);
}

void TetMesh::deserialize(const std::byte*& buf)
{
    Mesh::deserialize(buf);
    unpack(buf, _elements);
    unpack(buf, _element_properties);
    unpack(buf, _element_rest_volumes);
    unpack(buf, _element_inv_undeformed_basis);
    unpack(buf, _vertex_rest_volumes);
    unpack(buf, _surface_face_to_element_map);
    unpack(buf, _element_to_surface_faces_map);
    unpack(buf, _vertex_to_elements_map);
    unpack(buf, _edge_to_elements_map);
    unpack(buf, _face_to_elements_map);
    unpack(buf, _recently_removed_elements);
}

} // namespace Geometry