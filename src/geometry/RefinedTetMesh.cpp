#include "geometry/RefinedTetMesh.hpp"

#include <stack>

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

        // if the parent has no more children, it is defunct, so remove it also
        if (parent_tree_node.children.size() == 0)
        {
            _element_tree_nodes.erase(node_to_remove.parent);
        }

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
            _hanging_vertices.erase(elem_to_remove[k]);

            // since this vertex is being removed, we must update the parent edge -> child vertex map
            // but this only applies if the vertex being removed is a child vertex (i.e. created as a result of refinement)
            if (auto search = _child_vertex_to_parent_edge_map.find(elem_to_remove[k]); search != _child_vertex_to_parent_edge_map.end())
            {
                _parent_edge_to_child_vertex_map.erase(search->second);
                _child_vertex_to_parent_edge_map.erase(search->first);
            }
        }
    }
}

/** Refinement */

// int RefinedTetMesh::_addRefinedVertex(int parent_index1, int parent_index2)
int RefinedTetMesh::_addRefinedVertex(const ElementTreeNode& parent_node, int vi, int vj, int base_tree_node_index)
{
    int parent_index1 = parent_node.vertices[vi];
    int parent_index2 = parent_node.vertices[vj];

    Edge parent_edge(parent_index1, parent_index2);

    // check if the parent edge already has a vertex at the midpoint
    if (auto search = _parent_edge_to_child_vertex_map.find(parent_edge); search != _parent_edge_to_child_vertex_map.end())
    {
        // update if this node is hanging or not

        // if the parent edge is not in the mesh, then the child is not hanging
        if (_edge_to_elements_map.count(parent_edge) == 0)// && _hanging_vertices.count(parent_index1) == 0 && _hanging_vertices.count(parent_index2) == 0)
        {
            
        }
        // if the child node is hanging, and it is a "face" hanging node
        if (auto hang_search = _hanging_vertices.find(search->second); hang_search != _hanging_vertices.end() )
        {
            bool is_face = hang_search->second.second;
            int hanging_base_tree_node_index = hang_search->second.first;
            std::cout << "hanging vertex: " << search->second << "  is_face: " << is_face << "  hanging_btni: " << hanging_base_tree_node_index << "  btni: " << base_tree_node_index << std::endl;
            if (_edge_to_elements_map.count(parent_edge) == 0 && is_face)
            {
                if (hanging_base_tree_node_index != base_tree_node_index)
                {
                    std::cout << "Face vertex " << search->second << " no longer hanging!" << std::endl;
                    _hanging_vertices.erase(search->second);
                }
            }
            else if (_edge_to_elements_map.count(parent_edge) == 0 && !is_face)
            {
                bool is_hanging = false;
                if (_hanging_vertices.count(parent_index1))
                {
                    std::cout << "  parent_index1: " << parent_index1 << " is hanging..." << std::endl;
                    Edge p1_edge = _child_vertex_to_parent_edge_map.at(parent_index1);
                    bool parent_on_p1_edge = (p1_edge.index1 == parent_index2 || p1_edge.index2 == parent_index2);
                    if (parent_on_p1_edge)
                    {
                        is_hanging = true;
                    }
                }
                if (_hanging_vertices.count(parent_index2))
                {
                    std::cout << "  parent_index2: " << parent_index2 << " is hanging..." << std::endl;
                    Edge p2_edge = _child_vertex_to_parent_edge_map.at(parent_index2);
                    bool parent_on_p2_edge = (p2_edge.index1 == parent_index1 || p2_edge.index2 == parent_index1);
                    if (parent_on_p2_edge)
                    {
                        is_hanging = true;
                    }
                }
                if (!is_hanging)
                {
                    std::cout << "Vertex " << search->second << " no longer hanging!" << std::endl;
                    _hanging_vertices.erase(search->second);
                }
            }
        }
        
        return search->second;
    }

    // vertex doesn't exist yet, so create it!
    int new_index = _vertices.push_back( (_vertices.at(parent_index1) + _vertices.at(parent_index2)) / 2.0 );
    std::cout << "=== Created new vertex " << new_index << " ===" << std::endl;
    std::cout << "  parent_index1: " << parent_index1 << "  parent_index2: " << parent_index2 << std::endl;

    // now we need to check if it is hanging or not
    
    auto search_p1 = _hanging_vertices.find(parent_index1);
    auto search_p2 = _hanging_vertices.find(parent_index2);
    bool p1_hanging = search_p1 != _hanging_vertices.end();
    bool p2_hanging = search_p2 != _hanging_vertices.end();

    // regardless of whether or not the parents are hanging, if the parent edge is present in the mesh, then the new child vertex is hanging!
    if (_edge_to_elements_map.count(parent_edge) > 0)
    {
        std::cout << "  New vertex " << new_index << " is hanging with no hanging parents" << std::endl;
        _hanging_vertices.insert({new_index, std::make_pair(base_tree_node_index, false)});
    }
    // at least one parent is hanging and the parent edge is not in the mesh
    // there are a few different cases to check to determine whether the new node is hanging or not, mainly:
    //   - if the parent edges of the parent vertices are co-linear (child may be hanging)
    //   - if the parent edges of the parent vertices share a vertex (child may be hanging)
    //   - if the parent edges of the parent vertices do not share any vertices (child is not hanging)
    else if (p1_hanging || p2_hanging)
    {
        std::cout << "  At least one parent is hanging..." << std::endl;
        Edge p1_edge = _child_vertex_to_parent_edge_map.at(parent_index1);  // TODO: is this always safe? i.e. for the original tet verts that aren't created from refinement?
        Edge p2_edge = _child_vertex_to_parent_edge_map.at(parent_index2);

        // Case 1: parents are on edges that are collinear
        //  In this scenario, one of the parents' parent edges is composed of a vertex that is the other parent vertex.
        //  i.e. parent1's parent edge has parent2 on it 
        //
        //  if the "middle" parent vertex (parent1 in the above example) is hanging, then the child vertex is also hanging 
        bool parent_on_p1_edge = (p1_edge.index1 == parent_index2 || p1_edge.index2 == parent_index2);
        bool parent_on_p2_edge = (p2_edge.index1 == parent_index1 || p2_edge.index2 == parent_index1);
        // Case 1a: parent1's parent edge has parent2 on it
        if (parent_on_p1_edge)
        {
            // check if parent1 is hanging -> if so, the new child vertex is hanging
            if (p1_hanging)
            {
                std::cout << "  Both parents are on collinear edges = vertex is hanging" << std::endl;

                // determine if the child vertex is on a face of the original base element being subdivided
                // which is true if either parent is a face of the original base element being subdivided
                bool is_face = search_p1->second.second;
                if (p2_hanging) is_face = is_face || search_p2->second.second;

                _hanging_vertices.insert({new_index, std::make_pair(base_tree_node_index, is_face)});
            }
        }
        // Case 1b: parent2's parent edge has parent1 on it
        else if (parent_on_p2_edge)
        {
            // check if parent2 is hanging -> if so, the new child vertex is hanging
            if (p2_hanging)
            {
                // determine if the child vertex is on a face of the original base element being subdivided
                // which is true if either parent is on a face of the original base element being subdivided
                bool is_face = search_p2->second.second;
                if (p1_hanging) is_face = is_face || search_p1->second.second;

                std::cout << "  Both parents are on collinear edges = vertex is hanging" << std::endl;
                _hanging_vertices.insert({new_index, std::make_pair(base_tree_node_index, is_face)});
            }
        }
        // Case 2: parents are on different edges that share a vertex
        //  In this scenario, the parents' parent edges form a face that the child vertex is on.
        //  If this face is on the border of the original base element being subdivided (and this face is not on the outer surface of the mesh),
        //  then this vertex is hanging.
        else if (p1_edge.index1 == p2_edge.index1 || p1_edge.index1 == p2_edge.index2 || p1_edge.index2 == p2_edge.index1 || p1_edge.index2 == p2_edge.index2)
        {
            std::cout << "  Parents are on different parent edges that share a vertex!" << std::endl;

            // form the face that the parents' parent edges make
            Face face;
            if (p1_edge.index1 == p2_edge.index1 || p1_edge.index2 == p2_edge.index1)   face = Face(p1_edge.index1, p1_edge.index2, p2_edge.index2);
            if (p1_edge.index1 == p2_edge.index2 || p1_edge.index2 == p2_edge.index2)   face = Face(p1_edge.index1, p1_edge.index2, p2_edge.index1);

            // get the "grandparent" node, i.e. the parent node's parent
            // we will use the face properties of the grandparent node to determine if the face formed by the parents' parent edges is on the border of the original tet
            // or the outer surface of the mesh
            const ElementTreeNode& grandparent_node = _element_tree_nodes.at(parent_node.parent);

            std::cout << "  Face: " << face.index1 << ", " << face.index2 << ", " << face.index3 << std::endl;
            // Case 2a: The face formed from the parents' parent edge is the same as F123 on the grandparent element.
            // and F123 is on the border of the original tet and not on the mesh surface
            if ( (grandparent_node.f123_on_border && !grandparent_node.f123_on_surface) )
            {
                Face face123(grandparent_node.vertices[0], grandparent_node.vertices[1], grandparent_node.vertices[2]);
                std::cout << "  Face123: " << face123.index1 << ", " << face123.index2 << ", " << face123.index3 << std::endl;
                if (face123 == face)
                {
                    std::cout << "  On a border face not on a surface face = vertex is hanging" << std::endl;
                    _hanging_vertices.insert({new_index, std::make_pair(base_tree_node_index, true)});
                }
            }
            // Case 2b: The face formed from the parents' parent edge is the same as F124 on the grandparent element.
            // and F124 is on the border of the original tet and not on the mesh surface
            if ( (grandparent_node.f124_on_border && !grandparent_node.f124_on_surface) )
            {
                Face face124(grandparent_node.vertices[0], grandparent_node.vertices[1], grandparent_node.vertices[3]);
                std::cout << "  Face124: " << face124.index1 << ", " << face124.index2 << ", " << face124.index3 << std::endl;
                if (face124 == face)
                {
                    std::cout << "  On a border face not on a surface face = vertex is hanging" << std::endl;
                    _hanging_vertices.insert({new_index, std::make_pair(base_tree_node_index, true)});
                }
            }
            // Case 2c: The face formed from the parents' parent edge is the same as F134 on the grandparent element.
            // and F134 is on the border of the original tet and not on the mesh surface
            if ( (grandparent_node.f134_on_border && !grandparent_node.f134_on_surface))
            {
                Face face134(grandparent_node.vertices[0], grandparent_node.vertices[2], grandparent_node.vertices[3]);
                std::cout << "  Face134: " << face134.index1 << ", " << face134.index2 << ", " << face134.index3 << std::endl;
                if (face134 == face)
                {
                    std::cout << "  On a border face not on a surface face = vertex is hanging" << std::endl;
                    _hanging_vertices.insert({new_index, std::make_pair(base_tree_node_index, true)});
                }
            }
            // Case 2d: The face formed from the parents' parent edge is the same as F234 on the grandparent element.
            // and F234 is on the border of the original tet and not on the mesh surface
            if ( (grandparent_node.f234_on_border && !grandparent_node.f234_on_surface) )
            {
                Face face234(grandparent_node.vertices[1], grandparent_node.vertices[2], grandparent_node.vertices[3]);
                std::cout << "  Face234: " << face234.index1 << ", " << face234.index2 << ", " << face234.index3 << std::endl;
                if (face234 == face)
                {
                    std::cout << "  On a border face not on a surface face = vertex is hanging" << std::endl;
                    _hanging_vertices.insert({new_index, std::make_pair(base_tree_node_index, true)});
                }
            }
        }
        // Case 3: parents are on different edges that do not share a vertex
        else
        {
            std::cout << "  Parents are on different edges that do not share a vertex = not hanging" << std::endl;
            // do nothing
        }
    }
    

    // add the new vertex to the parent edge -> child vertex map
    _parent_edge_to_child_vertex_map.insert({parent_edge, new_index});
    _child_vertex_to_parent_edge_map.insert({new_index, parent_edge});

    return new_index;
}

int RefinedTetMesh::_addNewElementFromElementTreeNode(int tree_node_index)
{
    ElementTreeNode& node = _element_tree_nodes[tree_node_index];
    int elem_index = _addNewElement(node.vertices, node.f123_on_surface, node.f124_on_surface, node.f134_on_surface, node.f234_on_surface);
    node.element_index = elem_index;

    // update the element -> tree node map
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

    int base_node_index;
    auto surface_faces_range = _element_to_surface_faces_map.equal_range(element_index);

    if (auto search = _element_to_tree_node_map.find(element_index); search != _element_to_tree_node_map.end())
    {
        base_node_index = search->second;
        _element_tree_nodes[base_node_index].element_index = ElementTreeNode::INVALID_INDEX;
        _element_tree_nodes[base_node_index].f123_on_border = true;
        _element_tree_nodes[base_node_index].f124_on_border = true;
        _element_tree_nodes[base_node_index].f134_on_border = true;
        _element_tree_nodes[base_node_index].f234_on_border = true;

        _element_to_tree_node_map.erase(search);
    }
    else
    {
        // create the initial ElementTreeNode struct for the base element that we are subdividing
        ElementTreeNode base_node(element(element_index), ElementTreeNode::INVALID_INDEX, 0);
            
        // find which faces of the base element (if any) are on the outer surface of the mesh
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

        base_node.f123_on_border = true;
        base_node.f124_on_border = true;
        base_node.f134_on_border = true;
        base_node.f234_on_border = true;

        // add the base element tree node to the tree nodes vector
        base_node_index = _element_tree_nodes.push_back(std::move(base_node));
    }
    
    // recursively keep track of "parent" elements that we want to subdivide
    std::vector<int> parent_nodes = {base_node_index};
    int num_new_tets = 1;   // calculate the number of new tets to be added at each refinement level



    /** === Step 3: Remove the element from the mesh === */

    // remove surface faces associated with the element
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

    std::cout << "\n\n\n======================\nRefining element " << element_index << "\n======================" << std::endl;
    
    for (int level = 0; level < refinement_level; level++)
    {
        num_new_tets *= 8;

        std::cout << "Level " << level << "..." << std::endl;

        // create a vector to store the next level of parent elements
        // note that this only applies when we are not at the deepest refinement level
        std::vector<int> next_parent_nodes;
        if (level < refinement_level-1)
            next_parent_nodes.reserve(num_new_tets);

        // add newest level of midpoint vertices for each parent element at this level
        std::array<int,6> mid_verts;
        for (const auto& parent_node_index : parent_nodes)
        {            
            // reserve space for the new ElementTreeNodes ahead of time so our reference is not invalidated
            _element_tree_nodes.reserve(_element_tree_nodes.totalSize()+8);

            ElementTreeNode& parent_node = _element_tree_nodes[parent_node_index];
            // add vertices at midpoints
            int mid_vert_cnt = 0;
            for (int vi = 0; vi < 4; vi++)
            {
                for (int vj = vi+1; vj < 4; vj++)
                {
                    // mid_verts[mid_vert_cnt++] = _addRefinedVertex(parent_node.vertices[vi], parent_node.vertices[vj]);
                    mid_verts[mid_vert_cnt++] = _addRefinedVertex(parent_node, vi, vj, base_node_index);
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
            int enode1 = _element_tree_nodes.emplace_back(elem1, parent_node_index, parent_node.level+1, 
                parent_node.f123_on_surface, parent_node.f124_on_surface, parent_node.f134_on_surface, false,
                parent_node.f123_on_border, parent_node.f124_on_border, parent_node.f134_on_border, false);
            int enode2 = _element_tree_nodes.emplace_back(elem2, parent_node_index, parent_node.level+1, 
                parent_node.f123_on_surface, parent_node.f124_on_surface, parent_node.f234_on_surface, false,
                parent_node.f123_on_border, parent_node.f124_on_border, parent_node.f234_on_border, false);
            int enode3 = _element_tree_nodes.emplace_back(elem3, parent_node_index, parent_node.level+1, 
                parent_node.f123_on_surface, parent_node.f134_on_surface, parent_node.f234_on_surface, false,
                parent_node.f123_on_border, parent_node.f134_on_border, parent_node.f234_on_border, false);
            int enode4 = _element_tree_nodes.emplace_back(elem4, parent_node_index, parent_node.level+1, 
                false, parent_node.f124_on_surface, parent_node.f134_on_surface, parent_node.f234_on_surface,
                false, parent_node.f124_on_border, parent_node.f134_on_border, parent_node.f234_on_border);
            int enode5 = _element_tree_nodes.emplace_back(elem5, parent_node_index, parent_node.level+1,
                parent_node.f123_on_surface, false, false, false,
                parent_node.f123_on_border, false, false, false);
            int enode6 = _element_tree_nodes.emplace_back(elem6, parent_node_index, parent_node.level+1, 
                false, parent_node.f124_on_surface, false, false,
                false, parent_node.f124_on_border, false, false);
            int enode7 = _element_tree_nodes.emplace_back(elem7, parent_node_index, parent_node.level+1, 
                false, false, false, parent_node.f234_on_surface,
                false, false, false, parent_node.f234_on_border);
            int enode8 = _element_tree_nodes.emplace_back(elem8, parent_node_index, parent_node.level+1, 
                parent_node.f134_on_surface, false, false, false,
                parent_node.f134_on_border, false, false, false);

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

int RefinedTetMesh::coarsenElement(int element_index, int coarsening_level)
{
    // get the element tree node associated with this element
    auto search = _element_to_tree_node_map.find(element_index);

    // if the element is not a result of refinement, return
    if (search == _element_to_tree_node_map.end())
        return -1;

    const ElementTreeNode& leaf_node = _element_tree_nodes[search->second];

    // if the element doesn't have a parent (for some reason), remove it and do nothing
    if (leaf_node.parent == ElementTreeNode::INVALID_INDEX)
    {
        _element_tree_nodes.erase(search->second);
        _element_to_tree_node_map.erase(element_index);
        return -1;
    }

    // if the coarsening level was -1, use the level that the leaf node is at
    if (coarsening_level == -1)
        coarsening_level = leaf_node.level;

    // get the root of the tree branch that we are going to replace this element (and its relatives) with
    int root_index = leaf_node.parent;
    int cur_level = leaf_node.level - 1;
    while (cur_level > leaf_node.level - coarsening_level)
    {
        root_index = _element_tree_nodes[root_index].parent;
        cur_level--;
    }
    

    // add the element associated with root_node to the mesh
    // do this before we remove child elements so that the vertices associated with the root element don't get deleted
    ElementTreeNode& root_node = _element_tree_nodes[root_index];
    int new_node_index = _addNewElementFromElementTreeNode(root_index);

    // Stack holds pairs of (node index, processing_stage)
    // Stage 0: Push children
    // Stage 1: Delete node
    std::stack<std::pair<int, int>> stack;
    
    for (const auto& child_index : root_node.children)
    {
        stack.push({child_index, 0});
    }

    while (!stack.empty()) 
    {
        auto [node_index, stage] = stack.top();
        ElementTreeNode& node = _element_tree_nodes[node_index];
        stack.pop();
        
        if (stage == 0) 
        {
            // First visit: push this node back for deletion later
            stack.push({node_index, 1});
            
            // Then push all children (they'll be processed first)
            for (const auto& child_index : node.children) 
            {
                stack.push({child_index, 0});
            }
        } 
        else 
        {
            // Second visit: all children are deleted, safe to delete this node
            // if this is a leaf, delete the associated element in the mesh
            if (node.element_index != ElementTreeNode::INVALID_INDEX)
            {
                // remove surface faces associated with this element
                auto surface_faces_range = _element_to_surface_faces_map.equal_range(node.element_index);
                for (auto it = surface_faces_range.first; it != surface_faces_range.second; it++)
                {
                    _faces.erase(it->second);

                    // note: we do not need to update the surface face -> element map since that will just be overwritten by whatever new faces are added
                }

                _updateElementMapsForRemovedElement(node.element_index);
                _elements.erase(node.element_index);
                _element_to_tree_node_map.erase(node.element_index);
            }
            else
            {
                // if the node doesn't have an associated element index, it is a parent node
                // check it's edges for midpoint vertices - these vertices will now be hanging
                for (int k1 = 0; k1 < 4; k1++)
                {
                    for (int k2 = k1+1; k2 < 4; k2++)
                    {
                        if (auto search = _parent_edge_to_child_vertex_map.find(Edge(node.vertices[k1], node.vertices[k2])); search != _parent_edge_to_child_vertex_map.end())
                        {
                            if (vertexValid(search->second))
                            {
                                auto search_p1 = _hanging_vertices.find(node.vertices[k1]);
                                auto search_p2 = _hanging_vertices.find(node.vertices[k2]);
                                bool p1_hanging = search_p1 != _hanging_vertices.end();
                                bool p2_hanging = search_p2 != _hanging_vertices.end();

                                // this vertex is hanging, but what type is it? (edge or face?)
                                Edge p1_edge = _child_vertex_to_parent_edge_map.at(node.vertices[k1]);  // TODO: is this always safe? i.e. for the original tet verts that aren't created from refinement?
                                Edge p2_edge = _child_vertex_to_parent_edge_map.at(node.vertices[k2]);

                                //  if the "middle" parent vertex (parent1 in the above example) is hanging, then the child vertex is also hanging 
                                bool parent_on_p1_edge = (p1_edge.index1 == node.vertices[k2] || p1_edge.index2 == node.vertices[k2]);
                                bool parent_on_p2_edge = (p2_edge.index1 == node.vertices[k1] || p2_edge.index2 == node.vertices[k1]);
                                
                                bool is_edge = false;
                                // Case 1a: parent1's parent edge has parent2 on it
                                if (parent_on_p1_edge)
                                {
                                    // check if parent1 is hanging -> if so, the new child vertex is hanging
                                    if (p1_hanging)
                                    {
                                        // determine if the child vertex is on a face of the original base element being subdivided
                                        // which is true if either parent is a face of the original base element being subdivided
                                        bool is_face = search_p1->second.second;
                                        if (p2_hanging) is_face = is_face || search_p2->second.second;

                                        is_edge = !is_face;
                                    }
                                }
                                // Case 1b: parent2's parent edge has parent1 on it
                                else if (parent_on_p2_edge)
                                {
                                    // check if parent2 is hanging -> if so, the new child vertex is hanging
                                    if (p2_hanging)
                                    {
                                        // determine if the child vertex is on a face of the original base element being subdivided
                                        // which is true if either parent is on a face of the original base element being subdivided
                                        bool is_face = search_p2->second.second;
                                        if (p1_hanging) is_face = is_face || search_p1->second.second;

                                        is_edge = !is_face;
                                    }
                                }

                                std::cout << "  Vertex " << search->second << " is now a hanging vertex! is_edge: " << is_edge << std::endl;
                                _hanging_vertices.insert_or_assign(search->second, std::make_pair(-1, !is_edge));
                            }
                        }
                    }
                }
            }

            // remove the node
            _element_tree_nodes.erase(node_index);
        }   
    }

    // we have removed all the children so update the root node to reflect this
    root_node.children.clear();

    // update hanging vertices for the root node edges
    for (int k1 = 0; k1 < 4; k1++)
    {
        for (int k2 = k1+1; k2 < 4; k2++)
        {
            if (auto search = _parent_edge_to_child_vertex_map.find(Edge(root_node.vertices[k1], root_node.vertices[k2])); search != _parent_edge_to_child_vertex_map.end())
            {
                std::cout << "  Vertex " << search->second << " is now an edge hanging vertex!" << std::endl;
                _hanging_vertices.insert_or_assign(search->second, std::make_pair(root_index, false));
            }
        }
    }

    return new_node_index;
}

std::unordered_set<int> RefinedTetMesh::verifyHangingVertices() const
{
    std::unordered_set<int> hanging_verts;

    // iterate through all the edges in the mesh
    for (const auto& it : _edge_to_elements_map)
    {
        Edge edge = it.first;
        // std::cout << "Edge: " << edge.index1 << ", " << edge.index2 << std::endl;

        for (const auto& v_ind : _vertices.validIndices())
        {
            if (static_cast<int>(v_ind) == edge.index1 || static_cast<int>(v_ind) == edge.index2)
                continue;
            
            // check if the vertex is on the line
            const Vec3r& p = _vertices[v_ind];
            const Vec3r& A = _vertices[edge.index1];
            const Vec3r& B = _vertices[edge.index2];

            // if cross product != 0, vertex is not on the line
            const Vec3r AB = B - A;
            const Vec3r AP = p - A;
            const Vec3r cross = AB.cross(AP);
            if (cross.norm() > 1e-10)
                continue;

            // check if p is between A and B
            Real dot = AP.dot(AB);
            Real len_AB_sq = AB.squaredNorm();

            if (dot >= 0 && dot <= len_AB_sq)
            {
                // std::cout << "\tvertex " << v_ind << " on edge! " << std::endl;
                hanging_verts.insert(v_ind);
            }
        }
    }

    // iterate through all the faces in the mesh
    for (const auto& it : _face_to_elements_map)
    {
        Face face = it.first;
        // std::cout << "Face: " << face.index1 << ", " << face.index2 << ", " << face.index3 << std::endl;

        for (const auto& v_ind : _vertices.validIndices())
        {
            if (static_cast<int>(v_ind) == face.index1 || static_cast<int>(v_ind) == face.index2 || static_cast<int>(v_ind) == face.index3)
                continue;
            
            const Vec3r& p = _vertices[v_ind];
            const Vec3r& A = _vertices[face.index1];
            const Vec3r& B = _vertices[face.index2];
            const Vec3r& C = _vertices[face.index3];

            // check if vertex is on the face
            const Vec3r v0 = C - A;
            const Vec3r v1 = B - A;
            const Vec3r v2 = p - A;

            // Normal of triangle
            const Vec3r normal = (B - A).cross(C - A);
            Real normal_len = normal.norm();
            
            if (normal_len < 1e-10)
                continue;
            
            
            // Check coplanarity
            if (std::abs((p - A).dot(normal)) > 1e-10)
                continue;
            
            // Compute barycentric coordinates
            Real dot00 = v0.dot(v0);
            Real dot01 = v0.dot(v1);
            Real dot02 = v0.dot(v2);
            Real dot11 = v1.dot(v1);
            Real dot12 = v1.dot(v2);
            
            Real denom = dot00 * dot11 - dot01 * dot01;
            if (std::abs(denom) < 1e-10)
                continue;
            
            Real inv_denom = 1.0 / denom;
            Real u = (dot11 * dot02 - dot01 * dot12) * inv_denom;
            Real v = (dot00 * dot12 - dot01 * dot02) * inv_denom;
            
            // Check if point is in triangle (with small tolerance for numerical error)
            if ( (u >= 1e-10) && (v >= 1e-10) && (u + v <= 1 - 1e-10) )
            {
                // std::cout << "\tvertex " << v_ind << " on face! " << std::endl; 
                hanging_verts.insert(v_ind);
            }
        }
    }

    return hanging_verts;
}

} // namespace Geometry