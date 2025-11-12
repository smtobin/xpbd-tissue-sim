#include "geometry/RefinedTetMesh.hpp"

namespace Geometry
{

RefinedTetMesh::RefinedTetMesh(const std::vector<Vec3r>& vertices, const std::vector<Vec3i>& faces, const std::vector<Vec4i>& elements)
    : TetMesh(vertices, faces, elements)
{

}

RefinedTetMesh::RefinedTetMesh(const TetMesh& tet_mesh)
    : TetMesh(tet_mesh)
{

}

void RefinedTetMesh::removeElement(int elem_index)
{
    // before we remove the element, we need to update the tree structure (when applicable)
    if (auto search = _element_to_tree_node_map.find(elem_index); search != _element_to_tree_node_map.end())
    {
        int node_index = search->second;
        const ElementTreeNode& node_to_remove = _element_tree_nodes[node_index];
        ElementTreeNode& parent_tree_node = _element_tree_nodes[node_to_remove.parent];
        // remove the leaf tree node from its parent's list of children
        parent_tree_node.children.erase(
            std::remove(parent_tree_node.children.begin(), parent_tree_node.children.end(), node_index),
            parent_tree_node.children.end()
        );

        // remove the leaf tree node
        _element_tree_nodes.erase(node_index);

        // remove the element from the element -> tree node map
        _element_to_tree_node_map.erase(elem_index);
    }

    TetMesh::removeElement(elem_index);
}

void RefinedTetMesh::_updateVertexElementMapForRemovedElement(int element_index)
{
    // this is the same code as for TetMesh, but with extra logic to update the parent edge -> child vertex map when a vertex is removed
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

            // since this vertex is being removed, we must update the parent edge -> child vertex map
            // but this only applies if the element being removed is a child element (i.e. created as a result of refinement)
            if (auto search = _element_to_tree_node_map.find(element_index); search != _element_to_tree_node_map.end())
            {
                int node_index = search->second;
                const ElementTreeNode& child_tree_node = _element_tree_nodes[node_index];
                const ElementTreeNode& parent_tree_node = _element_tree_nodes[child_tree_node.parent];

                // go through parent element edges - MMM SPAGHETTI!
                const Vec4i& parent_elem = parent_tree_node.vertices;
                if (auto it = _parent_edge_to_child_vertex_map.find(Edge(parent_elem[0], parent_elem[1])); it != _parent_edge_to_child_vertex_map.end())
                {
                    if (it->second == elem_to_remove[k])
                        _parent_edge_to_child_vertex_map.erase(it);
                }
                else if (auto it = _parent_edge_to_child_vertex_map.find(Edge(parent_elem[0], parent_elem[2])); it != _parent_edge_to_child_vertex_map.end())
                {
                    if (it->second == elem_to_remove[k])
                        _parent_edge_to_child_vertex_map.erase(it);
                }
                else if (auto it = _parent_edge_to_child_vertex_map.find(Edge(parent_elem[0], parent_elem[3])); it != _parent_edge_to_child_vertex_map.end())
                {
                    if (it->second == elem_to_remove[k])
                        _parent_edge_to_child_vertex_map.erase(it);
                }
                else if (auto it = _parent_edge_to_child_vertex_map.find(Edge(parent_elem[1], parent_elem[2])); it != _parent_edge_to_child_vertex_map.end())
                {
                    if (it->second == elem_to_remove[k])
                        _parent_edge_to_child_vertex_map.erase(it);
                }
                else if (auto it = _parent_edge_to_child_vertex_map.find(Edge(parent_elem[1], parent_elem[3])); it != _parent_edge_to_child_vertex_map.end())
                {
                    if (it->second == elem_to_remove[k])
                        _parent_edge_to_child_vertex_map.erase(it);
                }
                else if (auto it = _parent_edge_to_child_vertex_map.find(Edge(parent_elem[2], parent_elem[3])); it != _parent_edge_to_child_vertex_map.end())
                {
                    if (it->second == elem_to_remove[k])
                        _parent_edge_to_child_vertex_map.erase(it);
                }
                
            }
        }
    }
}

/** Refinement */

int RefinedTetMesh::_addRefinedVertex(int parent_index1, int parent_index2)
{
    Edge parent_edge(parent_index1, parent_index2);

    // check if the parent edge already has a vertex at the midpoint
    if (auto search = _parent_edge_to_child_vertex_map.find(parent_edge); search != _parent_edge_to_child_vertex_map.end())
    {
        // update if this node is hanging or not
        // if the parent edge is not in the mesh, then the child is not hanging
        if (_edge_to_elements_map.count(parent_edge) == 0)
        {
            _hanging_vertices.erase(search->second);
        }
        
        return search->second;
    }

    // vertex doesn't exist yet, so create it!
    int new_index = _vertices.push_back( (_vertices.at(parent_index1) + _vertices.at(parent_index2)) / 2.0 );

    // now we need to check if it is hanging or not
    // if either of the parents are hanging, the child must be hanging too
    if (_hanging_vertices.count(parent_index1) > 0 || _hanging_vertices.count(parent_index2) > 0)
    {
        _hanging_vertices.insert(new_index);
    }
    // if neither of the parents are hanging, we need to check if the parent edge is in the mesh
    // if the parent edge is not in the mesh, then the child is not hanging
    else
    {
        if (_edge_to_elements_map.count(parent_edge) > 0)
        {
            _hanging_vertices.insert(new_index);
        }
    }

    // add the new vertex to the parent edge -> child vertex map
    _parent_edge_to_child_vertex_map.insert({parent_edge, new_index});

    return new_index;
}

int RefinedTetMesh::_addNewElementFromElementTreeNode(int tree_node_index)
{
    const ElementTreeNode& node = _element_tree_nodes[tree_node_index];
    int elem_index = _addNewElement(node.vertices, node.f123_on_surface, node.f124_on_surface, node.f134_on_surface, node.f234_on_surface);
    _element_to_tree_node_map.insert({elem_index, tree_node_index});
    return elem_index;
}

int RefinedTetMesh::_addNewElement(const Vec4i& new_element, bool f123_on_surface, bool f124_on_surface, bool f134_on_surface, bool f234_on_surface)
{
    // add the new element to the elements vector
    int new_elem_index = _elements.push_back(new_element);

    // update vertex -> element, edge -> element, face -> element maps
    _updateElementMapsForNewElement(new_elem_index);

    auto add_new_face = [&](const Vec3i& new_face) -> void
    {
        int new_face_index = _faces.push_back(new_face);

        _element_to_surface_faces_map.insert({new_elem_index, new_face_index});

        // ensure that the surface face -> element vector has enough space for the new face
        if (new_face_index >= static_cast<int>(_surface_face_to_element_map.size()))     _surface_face_to_element_map.resize(new_face_index+1);
        _surface_face_to_element_map[new_face_index] = new_elem_index;
    };

    // add surface faces and update element -> surface face and surface face -> element maps
    if (f123_on_surface)
    {
        Vec3i new_face(new_element[0], new_element[1], new_element[2]);
        add_new_face(new_face);
    }

    if (f124_on_surface)
    {
        Vec3i new_face(new_element[0], new_element[1], new_element[3]);
        add_new_face(new_face);
    }

    if (f134_on_surface)
    {
        Vec3i new_face(new_element[0], new_element[2], new_element[3]);
        add_new_face(new_face);
    }

    if (f234_on_surface)
    {
        Vec3i new_face(new_element[1], new_element[2], new_element[3]);
        add_new_face(new_face);
    }
    
    return new_elem_index;
}

void RefinedTetMesh::refineElement(int element_index, int refinement_level)
{
    
    assert(elementValid(element_index));


    /** === Step 1: Prepare for the refinement algorithm. === */

    const Vec4i base_element = element(element_index);  // don't use a ref here since the element will soon be removed

    // create the initial ElementTreeNode struct for the base element that we are subdividing
    ElementTreeNode base_node(element(element_index), ElementTreeNode::INVALID_INDEX, 0);
        
    // find which faces of the base element (if any) are on the outer surface of the mesh
    auto surface_faces_range = _element_to_surface_faces_map.equal_range(element_index);
    for (auto it = surface_faces_range.first; it != surface_faces_range.second; it++)
    {
        const Vec3i& face_vec = face(it->second);
        const Vec4i& elem = base_element;
        Face surface_face(face_vec[0], face_vec[1], face_vec[2]);
        
        if (Face(elem[0], elem[1], elem[2]) == surface_face)        base_node.f123_on_surface = true;
        else if (Face(elem[0], elem[1], elem[3]) == surface_face)   base_node.f124_on_surface = true;
        else if (Face(elem[0], elem[2], elem[3]) == surface_face)   base_node.f134_on_surface = true;
        else if (Face(elem[1], elem[2], elem[3]) == surface_face)   base_node.f234_on_surface = true;
    }

    // add the base element tree node to the tree nodes vector
    int base_node_index = _element_tree_nodes.push_back(std::move(base_node));
    
    // recursively keep track of "parent" elements that we want to subdivide
    std::vector<int> parent_nodes = {base_node_index};
    int num_new_tets = 1;   // calculate the number of new tets to be added at each refinement level



    /** === Step 3: Remove the element from the mesh === */

    // first remove surface faces associated with the element
    for (auto it = surface_faces_range.first; it != surface_faces_range.second; it++)
    {
        _faces.erase(it->second);

        // note: we do not need to update the surface face -> element map since that will just be overwritten by whatever new faces are added
    }

    // update the edge -> element, face -> element, and element -> surface face maps
    // we need to wait to update the vertex -> element map, because if we do it now, we might accidentally remove some of the original tet's vertices!
    _updateEdgeElementMapForRemovedElement(element_index);
    _updateFaceElementMapForRemovedElement(element_index);
    _updateElementSurfaceFaceMapForRemovedElement(element_index);

    /** === Step 4: Refine the element. === */

    
    for (int level = 0; level < refinement_level; level++)
    {
        num_new_tets *= 8;

        // create a vector to store the next level of parent elements
        // note that this only applies when we are not at the deepest refinement level
        std::vector<int> next_parent_nodes;
        if (level < refinement_level-1)
            next_parent_nodes.reserve(num_new_tets);

        // add newest level of midpoint vertices for each parent element at this level
        std::array<int,6> mid_verts;
        for (const auto& parent_node_index : parent_nodes)
        {
            // std::cout << "\n\n=== Parent Element: " << parent_node.transpose() << std::endl;
            ElementTreeNode& parent_node = _element_tree_nodes[parent_node_index];

            int mid_vert_cnt = 0;
            for (int vi = 0; vi < 4; vi++)
            {
                for (int vj = vi+1; vj < 4; vj++)
                {
                    mid_verts[mid_vert_cnt++] = _addRefinedVertex(parent_node.vertices[vi], parent_node.vertices[vj]);
                }
            }

            // the 4 "corner" new tets
            const Vec4i elem1(parent_node.vertices[0], mid_verts[0], mid_verts[1], mid_verts[2]);   // (0, 4, 5, 6)
            const Vec4i elem2(parent_node.vertices[1], mid_verts[0], mid_verts[3], mid_verts[4]);   // (1, 4, 7, 8)
            const Vec4i elem3(parent_node.vertices[2], mid_verts[1], mid_verts[3], mid_verts[5]);   // (2, 5, 7, 9)
            const Vec4i elem4(mid_verts[2], mid_verts[4], mid_verts[5], parent_node.vertices[3]);   // (6, 8, 9, 3)

            // the 4 center tets from the octahedron
            const Vec4i elem5(mid_verts[0], mid_verts[1], mid_verts[3], mid_verts[2]);    // (4, 5, 7, 6)
            const Vec4i elem6(mid_verts[0], mid_verts[2], mid_verts[3], mid_verts[4]);    // (4, 6, 7, 8)
            const Vec4i elem7(mid_verts[2], mid_verts[4], mid_verts[5], mid_verts[3]);    // (6, 8, 9, 7)
            const Vec4i elem8(mid_verts[1], mid_verts[5], mid_verts[2], mid_verts[3]);    // (5, 9, 6, 7)

            // create tree nodes for each element
            int enode1 = _element_tree_nodes.emplace_back(elem1, parent_node_index, parent_node.level+1, parent_node.f123_on_surface, parent_node.f124_on_surface, parent_node.f134_on_surface, false);
            int enode2 = _element_tree_nodes.emplace_back(elem2, parent_node_index, parent_node.level+1, parent_node.f123_on_surface, parent_node.f124_on_surface, parent_node.f234_on_surface, false);
            int enode3 = _element_tree_nodes.emplace_back(elem3, parent_node_index, parent_node.level+1, parent_node.f123_on_surface, parent_node.f134_on_surface, parent_node.f234_on_surface, false);
            int enode4 = _element_tree_nodes.emplace_back(elem4, parent_node_index, parent_node.level+1, false, parent_node.f124_on_surface, parent_node.f134_on_surface, parent_node.f234_on_surface);
            int enode5 = _element_tree_nodes.emplace_back(elem5, parent_node_index, parent_node.level+1, parent_node.f123_on_surface, false, false, false);
            int enode6 = _element_tree_nodes.emplace_back(elem6, parent_node_index, parent_node.level+1, false, parent_node.f124_on_surface, false, false);
            int enode7 = _element_tree_nodes.emplace_back(elem7, parent_node_index, parent_node.level+1, false, false, false, parent_node.f234_on_surface);
            int enode8 = _element_tree_nodes.emplace_back(elem8, parent_node_index, parent_node.level+1, parent_node.f134_on_surface, false, false, false);

            // add each child node to the parent
            parent_node.children.insert(parent_node.children.end(), {enode1, enode2, enode3, enode4, enode5, enode6, enode7, enode8});

            if (level == refinement_level-1)
            {
                // we are at the lowest level
                // so add the new elements to the global elements list
                // we also update the element -> tree node map
                _addNewElementFromElementTreeNode(enode1);
                _addNewElementFromElementTreeNode(enode2);
                _addNewElementFromElementTreeNode(enode3);
                _addNewElementFromElementTreeNode(enode4);
                _addNewElementFromElementTreeNode(enode5);
                _addNewElementFromElementTreeNode(enode6);
                _addNewElementFromElementTreeNode(enode7);
                _addNewElementFromElementTreeNode(enode8);
            }
            else
            {
                // we are not at the lowest level
                // so add the elements to next_parent_nodes for the next iteration
                next_parent_nodes.insert(next_parent_nodes.end(), {enode1, enode2, enode3, enode4, enode5, enode6, enode7, enode8});
            }

        }
        
        parent_nodes = std::move(next_parent_nodes);
        
    }

    // remove the element
    _updateVertexElementMapForRemovedElement(element_index);
    _elements.erase(element_index);


}

} // namespace Geometry