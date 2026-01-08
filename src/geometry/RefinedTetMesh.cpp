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

void RefinedTetMesh::setCurrentStateAsUndeformedState()
{
    TetMesh::setCurrentStateAsUndeformedState();

    // set the initial vertices
    _initial_vertices.resize(_vertices.totalSize());
    for (int i = 0; i < _vertices.totalSize(); i++)
    {
        _initial_vertices[i] = _vertices[i];
    }

    // set the initial refinement levels
    _element_refinement_level.resize(_elements.totalSize(), 0);

    // add edge nodes and face nodes for the features in the mesh
    // _edge_nodes.clear();
    // _face_nodes.clear();
    // _edge_to_edge_node_map.clear();
    // _face_to_face_node_map.clear();

    // for (const auto& element_index : _elements.validIndices())
    // {
    //     const Vec4i& element = _elements[element_index];

    //     // create edge nodes
    //     for (int i = 0; i < 4; i++)
    //     {
    //         for (int j = i+1; j < 4; j++)
    //         {
    //             Edge edge(element[i], element[j]);
    //             EdgeNode edge_node(edge);
    //             edge_node.in_mesh = true;

    //             // if this edge isn't already accounted for, add it the appropriate maps
    //             if (_edge_to_edge_node_map.count(edge) == 0)
    //             {
    //                 int edge_node_index = _edge_nodes.push_back(std::move(edge_node));
    //                 _edge_to_edge_node_map.insert({edge, edge_node_index});
    //             }
    //         }
    //     }

    //     // create face nodes
    //     for (int i = 0; i < 4; i++)
    //     {
    //         for (int j = i+1; j < 4; j++)
    //         {
    //             for (int k = j+1; k < 4; k++)
    //             {
    //                 Face cur_face(element[i], element[j], element[k]);
    //                 FaceNode face_node(cur_face);
    //                 face_node.in_mesh = true;

    //                 auto surface_faces_range = _element_to_surface_faces_map.equal_range(element_index);    // iterators for the surface faces associated with the element - useful later
    //                 for (auto it = surface_faces_range.first; it != surface_faces_range.second; it++)
    //                 {
    //                     const Vec3i& face_vec = face(it->second);
    //                     Face surface_face(face_vec[0], face_vec[1], face_vec[2]);
    //                     if (cur_face == surface_face)
    //                     {
    //                         face_node.on_surface = true;
    //                     }
    //                 }
    //             }
    //         }
    //     }
        
    // }
}

int RefinedTetMesh::_addVertex(int parent1_index, int parent2_index)
{
    const Vec3r& parent1 = _vertices[parent1_index];
    const Vec3r& parent2 = _vertices[parent2_index];

    // new vertex is just the midpoint of the parents
    Vec3r new_vert = (parent1 + parent2) / 2.0;
    int new_index = _vertices.push_back(new_vert);

    // interpolate the parent vertices to find the initial vertex
    const Vec3r& initial_parent1 = _initial_vertices[parent1_index];
    const Vec3r& initial_parent2 = _initial_vertices[parent2_index];
    Vec3r initial_new_vert = (initial_parent1 + initial_parent2) / 2.0;
    _initial_vertices.resize(_vertices.totalSize());
    _initial_vertices[new_index] = initial_new_vert;


    // update vertex adjacency lists
    _vertex_adjacent_vertices.resize(_vertices.totalSize());
    _vertex_adjacent_vertices[new_index].clear();
    // when an edge is split, the parent nodes are no longer adjacent if the parent edge is not in the mesh
    Edge parent_edge(parent1_index, parent2_index);
    if (_edge_to_elements_map.count(parent_edge) == 0)
    {
        _vertex_adjacent_vertices[parent1_index].erase(parent2_index);
        _vertex_adjacent_vertices[parent2_index].erase(parent1_index);
    }
    
    // and the parent nodes are now adjacent to the new node
    _vertex_adjacent_vertices[new_index].insert(parent1_index);
    _vertex_adjacent_vertices[new_index].insert(parent2_index);
    _vertex_adjacent_vertices[parent1_index].insert(new_index);
    _vertex_adjacent_vertices[parent2_index].insert(new_index);

    // resize all vertex properties
    _vertex_properties.for_each_element([&](auto& prop) {
        prop.resize(_vertices.totalSize());
    });

    // add the vertex to the list of newly created vertices
    _latest_new_vertices.emplace_back(new_index, parent1_index, parent2_index);

    return new_index;
}

int RefinedTetMesh::_addFace(const Vec3i& new_face)
{
    // add the new face
    int new_index = _faces.push_back(new_face);

    // resize all face properties
    _face_properties.for_each_element([&](auto& prop) {
        prop.resize(_faces.totalSize());
    });

    return new_index;
}



void RefinedTetMesh::removeElement(int elem_index)
{
    // do this first - this will update the vertex, edge, and face maps for removing this element
    TetMesh::removeElement(elem_index);

    // before we remove the element, we need to update the tree structure (when applicable)
    if (auto search = _element_to_tree_node_map.find(elem_index); search != _element_to_tree_node_map.end())
    {
        int node_index = search->second;
        const ElementTreeNode& node_to_remove = _element_tree_nodes[node_index];
        ElementTreeNode& parent_tree_node = _element_tree_nodes[node_to_remove.parent];

        // since this element is being removed, all of its faces (if they are still in the mesh after the element is removed) will now be on the surface
        // so update the face nodes of the element to reflect this
        // Likely, most of the face nodes associated with the element will be removed, but the ones that are left behind (because they are still in the mesh)
        //   will correctly be marked on the surface
        for (const auto& face_node_index : node_to_remove.face_nodes)
        {
            if (face_node_index == ElementTreeNode::INVALID_INDEX)
                continue;

            _face_nodes[face_node_index].on_surface = true;
        }

        // update the feature hierarchy (i.e. remove edge nodes and face nodes that are no longer in the mesh)
        _updateFeatureHierarchyForRemovedElementTreeNode(node_index);

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
}

void RefinedTetMesh::_updateFeatureHierarchyForRemovedElementTreeNode(int element_tree_node_index)
{
    ElementTreeNode& node = _element_tree_nodes[element_tree_node_index];

    // update the feature hierarchy
    // for an edge or face to be removed, it must:
    //    - not have any children (i.e. it is a leaf)
    //    - not be in the mesh itself
    // then we can safely remove the feature from the feature hierarchy
    // std::cout "=== Updating feature hierarchy for node with vertices " << node.vertices.transpose() << std::endl;
    // check edges of the element
    for (const auto& edge_node_index : node.edge_nodes)
    {
        if (edge_node_index == ElementTreeNode::INVALID_INDEX)
            assert(0);  // this shouldn't happen
        
        EdgeNode& edge_node = _edge_nodes[edge_node_index];

        // move on if the edge node is not a leaf
        if (!edge_node.is_leaf)
        {
            assert(edge_node.child_vertex != ElementTreeNode::INVALID_INDEX);
            edge_node.in_mesh = true;
            const auto [it, success] = _hanging_vertices.insert({edge_node.child_vertex, edge_node.edge});
            if (success)
                _latest_new_hanging_vertices.emplace_back(edge_node.child_vertex, edge_node.edge.index1, edge_node.edge.index2);
            continue;
        }
            

        // move on if the edge is still present in the mesh (i.e. another element shares this exact edge)
        if (_edge_to_elements_map.count(edge_node.edge) > 0)
            continue;

        // if we get to here, it is safe to remove this edge node
        // edit the parent
        if (edge_node.parent_edge_node != ElementTreeNode::INVALID_INDEX)
        {
            EdgeNode& parent_edge_node = _edge_nodes[edge_node.parent_edge_node];
            // std::cout "  setting child vertex of EdgeNode " << edge_node.parent_edge_node << " to -1! Used to be " << parent_edge_node.child_vertex << std::endl;
            parent_edge_node.child_vertex = ElementTreeNode::INVALID_INDEX;
            parent_edge_node.is_leaf = true;
            

            if (parent_edge_node.child_edge_node1 == edge_node_index)
                parent_edge_node.child_edge_node1 = ElementTreeNode::INVALID_INDEX;
            else
                parent_edge_node.child_edge_node2 = ElementTreeNode::INVALID_INDEX;
        }
        else if (edge_node.parent_face_node != ElementTreeNode::INVALID_INDEX)
        {
            FaceNode& parent_face_node = _face_nodes[edge_node.parent_face_node];
            parent_face_node.is_leaf = true;

            for (auto& parent_child_edge_node_index : parent_face_node.child_edge_nodes)
            {
                if (parent_child_edge_node_index == edge_node_index)
                {
                    parent_child_edge_node_index = ElementTreeNode::INVALID_INDEX;
                    break;
                }
            }
        }
        // since we are removing the edge node, we need to update the vertex adjacency lists
        // i.e. the vertices that make up the edge are no longer adjacent
        _vertex_adjacent_vertices[edge_node.edge.index1].erase(edge_node.edge.index2);
        _vertex_adjacent_vertices[edge_node.edge.index2].erase(edge_node.edge.index1);

        // remove the edge node
        _edge_nodes.erase(edge_node_index);
    }

    // check faces of the element
    for (const auto& face_node_index : node.face_nodes)
    {
        if (face_node_index == ElementTreeNode::INVALID_INDEX)
            assert(0);      // this shouldn't happen
        
        FaceNode& face_node = _face_nodes[face_node_index];

        // move on if the face node is not a leaf
        if (!face_node.is_leaf)
        {
            face_node.in_mesh = true;
            continue;
        }
        
        // move on if the face is still present in the mesh (i.e. another element shares this exact face)
        if (_face_to_elements_map.count(face_node.face) > 0)
            continue;

        // if we get to here, it is safe to remove this face node
        // edit the parent
        if (face_node.parent_face_node != ElementTreeNode::INVALID_INDEX)
        {
            FaceNode& parent_face_node = _face_nodes[face_node.parent_face_node];
            parent_face_node.is_leaf = true;
            
            for (auto& parent_child_face_node_index : parent_face_node.child_face_nodes)
            {
                if (parent_child_face_node_index == face_node_index)
                {
                    parent_child_face_node_index = ElementTreeNode::INVALID_INDEX;
                    break;
                }
            }
        }

        _face_nodes.erase(face_node_index);
    }
}

void RefinedTetMesh::_updateVertexElementMapForRemovedElement(int element_index)
{
    // this is the same code as for TetMesh, but with extra logic to update the list of hanging vertices
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
            // std::cout "\nRemoving vertex " << elem_to_remove[k] << "! No longer in the mesh." << std::endl;
            _vertices.erase(elem_to_remove[k]);
            _latest_removed_vertices.emplace_back(elem_to_remove[k], -1, -1);

            int hanging_vert_removed = _hanging_vertices.erase(elem_to_remove[k]);
            if (hanging_vert_removed)
                _latest_removed_hanging_vertices.push_back(elem_to_remove[k]);
            
            // vertex no longer in mesh, so clear its adjacent vertices list
            _vertex_adjacent_vertices[elem_to_remove[k]].clear();

            
        }
    }
}

/** Refinement */

int RefinedTetMesh::_addNewElementFromElementTreeNode(int tree_node_index)
{
    ElementTreeNode& node = _element_tree_nodes[tree_node_index];
    // determine which faces (if any) or on the outer surface of the mesh so that we can add the appropriate faces to be visualized
    bool f012_on_surface = (node.face_nodes[0] != ElementTreeNode::INVALID_INDEX && _face_nodes[node.face_nodes[0]].on_surface);
    bool f013_on_surface = (node.face_nodes[1] != ElementTreeNode::INVALID_INDEX && _face_nodes[node.face_nodes[1]].on_surface);
    bool f023_on_surface = (node.face_nodes[2] != ElementTreeNode::INVALID_INDEX && _face_nodes[node.face_nodes[2]].on_surface);
    bool f123_on_surface = (node.face_nodes[3] != ElementTreeNode::INVALID_INDEX && _face_nodes[node.face_nodes[3]].on_surface);
    // add the new element to the mesh
    int elem_index = _addNewElement(node.vertices, f012_on_surface, f013_on_surface, f023_on_surface, f123_on_surface);
    // set the index of the ElementTreeNode to point to that new element we just created
    node.element_index = elem_index;

    // store the refinement level of the element we just created
    _element_refinement_level.resize(_elements.totalSize());
    _element_refinement_level[elem_index] = node.level;

    // update the element -> tree node map
    _element_to_tree_node_map.insert({elem_index, tree_node_index});

    return elem_index;
}

int RefinedTetMesh::_addNewElement(const Vec4i& new_element, bool f012_on_surface, bool f013_on_surface, bool f023_on_surface, bool f123_on_surface)
{
    // add the new element to the elements vector
    int new_elem_index = _elements.push_back(new_element);

    // resize all element properties
    _element_properties.for_each_element([&](auto& prop) {
        prop.resize(_elements.totalSize());
    });

    // compute rest volume using initial vertices
    _element_rest_volumes.resize(_elements.totalSize());
    // get initial vertices
    const Vec3r& iv1 = _initial_vertices[new_element[0]];
    const Vec3r& iv2 = _initial_vertices[new_element[1]];
    const Vec3r& iv3 = _initial_vertices[new_element[2]];
    const Vec3r& iv4 = _initial_vertices[new_element[3]];
    // compute initial edge vectors
    const Vec3r ie1 = iv1 - iv4;
    const Vec3r ie2 = iv2 - iv4;
    const Vec3r ie3 = iv3 - iv4;
    // volume
    Real rest_volume = ie1.dot(ie2.cross(ie3)) / 6.0;
    _element_rest_volumes[new_elem_index] = std::abs(rest_volume);

    // compute inverse of undeformed element basis (for deformation gradient computation later)
    _element_inv_undeformed_basis.resize(_elements.totalSize());
    Mat3r Q;
    Q.row(0) = ie2.cross(ie3);
    Q.row(1) = ie3.cross(ie1);
    Q.row(2) = ie1.cross(ie2);
    Q = Q/(6*rest_volume);
    _element_inv_undeformed_basis[new_elem_index] = Q;

    // update adjacency lists - this may be slightly redundant
    _vertex_adjacent_vertices[new_element[0]].insert({new_element[1], new_element[2], new_element[3]});
    _vertex_adjacent_vertices[new_element[1]].insert({new_element[0], new_element[2], new_element[3]});
    _vertex_adjacent_vertices[new_element[2]].insert({new_element[0], new_element[1], new_element[3]});
    _vertex_adjacent_vertices[new_element[3]].insert({new_element[0], new_element[1], new_element[2]});

    // update vertex -> element, edge -> element, face -> element maps
    _updateElementMapsForNewElement(new_elem_index);

    // helper lambda that adds a new face to the mesh and updates the maps associated with that surface face
    auto add_new_face = [&](const Vec3i& new_face) -> void
    {
        // int new_face_index = _faces.push_back(new_face);
        int new_face_index = _addFace(new_face);

        _element_to_surface_faces_map.insert({new_elem_index, new_face_index});

        // ensure that the surface face -> element vector has enough space for the new face
        if (new_face_index >= static_cast<int>(_surface_face_to_element_map.size()))     _surface_face_to_element_map.resize(new_face_index+1);
        _surface_face_to_element_map[new_face_index] = new_elem_index;

        _latest_new_faces.push_back(new_face_index);
    };

    // add surface faces and update element -> surface face and surface face -> element maps
    // F012
    if (f012_on_surface)
    {
        Vec3i new_face(new_element[0], new_element[1], new_element[2]);
        add_new_face(new_face);
    }
    // F013
    if (f013_on_surface)
    {
        Vec3i new_face(new_element[0], new_element[1], new_element[3]);
        add_new_face(new_face);
    }
    // F023
    if (f023_on_surface)
    {
        Vec3i new_face(new_element[0], new_element[2], new_element[3]);
        add_new_face(new_face);
    }
    // F123
    if (f123_on_surface)
    {
        Vec3i new_face(new_element[1], new_element[2], new_element[3]);
        add_new_face(new_face);
    }

    _latest_new_elements.emplace_back(new_elem_index);
    
    return new_elem_index;
}

void RefinedTetMesh::_prepareFeatureTreeForRefinedElement(int element_tree_node_index, int depth)
{
    // std::cout "\n=== UpdateFeatureTreeForRemovedElement ===" << std::endl;
    ElementTreeNode& root_node = _element_tree_nodes[element_tree_node_index];
    // std::cout "  element: " << root_node.vertices.transpose() << std::endl;

    /** First, figure out if any updates need to be made */

    // store the edges and faces that need to be updated 
    // (first int = node index, second int = depth)
    std::stack<std::pair<int,int>> face_nodes_to_update;
    std::stack<std::pair<int,int>> edge_nodes_to_update;

    // iterate through the root's face nodes and look for faces whose parent feature has
    // in_mesh = false and the face itself is not part of another element
    for (const auto& face_node_index : root_node.face_nodes)
    {
        if (face_node_index == ElementTreeNode::INVALID_INDEX)
            continue;
        
        FaceNode& face_node = _face_nodes[face_node_index];
        // std::cout "    checking if face " << face_node.face.index1 << ", " << face_node.face.index2 << ", " << face_node.face.index3 << " needs to be updated" << std::endl;
        if (_face_to_elements_map.count(face_node.face) == 0)
        {
            if (face_node.parent_face_node != ElementTreeNode::INVALID_INDEX && _face_nodes[face_node.parent_face_node].in_mesh)
            {
                // the parent feature is still in the mesh, so no updates need to be made
            }
            else
            {
                // std::cout "      updates needed!" << std::endl;
                // the parent feature is not in the mesh, so we need to update the face_node's children to the specified depth
                face_nodes_to_update.push({face_node_index, depth});
            }
            
        }
    }

    // iterate through the root's edge nodes and look for faces whose parent feature has
    // in_mesh = false and the edge itself is not part of another element - these are the edges that need to be updated 
    for (const auto& edge_node_index : root_node.edge_nodes)
    {
        if (edge_node_index == ElementTreeNode::INVALID_INDEX)
            continue;

        EdgeNode& edge_node = _edge_nodes[edge_node_index];
        // std::cout "    checking if edge " << edge_node.edge.index1 << ", " << edge_node.edge.index2 << " needs to be updated" << std::endl;
        if (_edge_to_elements_map.count(edge_node.edge) == 0)
        {
            if ( (edge_node.parent_edge_node != ElementTreeNode::INVALID_INDEX && _edge_nodes[edge_node.parent_edge_node].in_mesh ) ||
                 (edge_node.parent_face_node != ElementTreeNode::INVALID_INDEX && _face_nodes[edge_node.parent_face_node].in_mesh ) )
            {
                // the parent feature is still in the mesh, so no updates need to be made
            }
            else
            {
                // std::cout "      updates needed!" << std::endl;
                // the parent feature is not in the mesh, so we need to update the edge_node's children to the specified depth
                edge_nodes_to_update.push({edge_node_index, depth});
            }
        }
    }



    /** Now, update the branches marked for updates to the specified depth */
    // update branches starting with a face node
    while (!face_nodes_to_update.empty())
    {
        auto [node_index, d] = face_nodes_to_update.top();
        FaceNode& face_node = _face_nodes[node_index];
        face_node.in_mesh = false;

        face_nodes_to_update.pop();

        if (!face_node.is_leaf && d > 0)
        {
            // push the child faces and edges to be updated if they exist, and they do not belong to another element in the mesh
            for (const auto& child_face_node_index : face_node.child_face_nodes)
            {
                if (child_face_node_index != ElementTreeNode::INVALID_INDEX && _face_to_elements_map.count(_face_nodes.at(child_face_node_index).face) == 0)
                {
                    face_nodes_to_update.push({child_face_node_index, d-1});
                }
            }

            for (const auto& child_edge_node_index : face_node.child_edge_nodes)
            {
                if (child_edge_node_index != ElementTreeNode::INVALID_INDEX && _edge_to_elements_map.count(_edge_nodes.at(child_edge_node_index).edge) == 0)
                {
                    edge_nodes_to_update.push({child_edge_node_index, d-1});
                }
            }
        }
    }

    // update branches starting with an edge node
    while (!edge_nodes_to_update.empty())
    {
        auto [node_index, d] = edge_nodes_to_update.top();
        EdgeNode& edge_node = _edge_nodes[node_index];

        // std::cout "Setting edge " << edge_node.edge.index1 << ", " << edge_node.edge.index2 << " to not be in_mesh!" << std::endl;
        edge_node.in_mesh = false;

        edge_nodes_to_update.pop();

        if (!edge_node.is_leaf && d > 0)
        {
            // since the parent edge is no longer in the mesh, the child vertex (if it exists) is no longer hanging
            if (edge_node.child_vertex != ElementTreeNode::INVALID_INDEX)
            {
                // std::cout "Removing vertex " << edge_node.child_vertex << " from hanging vertices!" << std::endl;
                int hanging_vert_removed = _hanging_vertices.erase(edge_node.child_vertex);
                if (hanging_vert_removed)
                    _latest_removed_hanging_vertices.push_back(edge_node.child_vertex);
            }

            // push the child edges to be updated if they exist, and they do not belong to another element in the mesh
            if (edge_node.child_edge_node1 != ElementTreeNode::INVALID_INDEX && _edge_to_elements_map.count(_edge_nodes.at(edge_node.child_edge_node1).edge) == 0)
            {
                edge_nodes_to_update.push({edge_node.child_edge_node1, d-1});
            }
            if (edge_node.child_edge_node2 != ElementTreeNode::INVALID_INDEX && _edge_to_elements_map.count(_edge_nodes.at(edge_node.child_edge_node2).edge) == 0)
            {
                edge_nodes_to_update.push({edge_node.child_edge_node2, d-1});
            }
        }
    }
}

void RefinedTetMesh::_createMidpointVerticesAndChildEdgeNodesForElement(int element_tree_node_index, std::array<int,6>& midpoint_vertices, bool at_refinement_depth)
{
    ElementTreeNode& parent_node = _element_tree_nodes[element_tree_node_index];
    
    for (int vi = 0; vi < 4; vi++)
    {
        for (int vj = vi+1; vj < 4; vj++)
        {
            // map the nested sequence to a linear sequence
            int edge_index = 4*vi - (vi * (vi + 1)) / 2 + (vj - vi - 1);

            // get the EdgeNode for the parent edge
            int parent_edge_node_index = parent_node.edge_nodes[edge_index];

            // vertex already exists at midpoint (i.e. an adjacent element was already refined)
            if (_edge_nodes[parent_edge_node_index].child_vertex != ElementTreeNode::INVALID_INDEX)
            {
                midpoint_vertices[edge_index] = _edge_nodes[parent_edge_node_index].child_vertex;

                // if the parent edge is no longer in the mesh, update the vertex adjacency lists
                const Edge& parent_edge = _edge_nodes[parent_edge_node_index].edge;
                // std::cout << " Parent edge being split: (" << parent_edge.index1 << ", " << parent_edge.index2 << ") " << std::endl;
                if (_edge_to_elements_map.count(parent_edge) == 0)
                {
                    // std::cout << "  Edge is split and parent edge not in the mesh anymore!" << std::endl;
                    // std::cout << "  Removing vertex " << parent_edge.index2 << " from " << parent_edge.index1 << "'s adjacent vertices list!" << std::endl;
                    _vertex_adjacent_vertices[parent_edge.index1].erase(parent_edge.index2);
                    // std::cout << "  Removing vertex " << parent_edge.index1 << " from " << parent_edge.index2 << "'s adjacent vertices list!" << std::endl;
                    _vertex_adjacent_vertices[parent_edge.index2].erase(parent_edge.index1);
                }

            }
            // create new vertex at midpoint
            else
            {
                // int new_vert_index = _vertices.push_back( (_vertices.at(parent_node.vertices[vi]) + _vertices.at(parent_node.vertices[vj])) / 2.0 );
                int new_vert_index = _addVertex(parent_node.vertices[vi], parent_node.vertices[vj]);
                midpoint_vertices[edge_index] = new_vert_index;

                // create EdgeNodes for the child edges
                EdgeNode child1(parent_node.vertices[vi], midpoint_vertices[edge_index]);
                child1.parent_edge_node = parent_edge_node_index;
                child1.in_mesh = _edge_nodes[parent_edge_node_index].in_mesh || at_refinement_depth;

                EdgeNode child2(parent_node.vertices[vj], midpoint_vertices[edge_index]);
                child2.parent_edge_node = parent_edge_node_index;
                child2.in_mesh = child1.in_mesh;

                _edge_nodes[parent_edge_node_index].child_edge_node1 = _edge_nodes.push_back(std::move(child1));
                _edge_nodes[parent_edge_node_index].child_edge_node2 = _edge_nodes.push_back(std::move(child2));

                _edge_nodes[parent_edge_node_index].is_leaf = false;
                _edge_nodes[parent_edge_node_index].child_vertex = new_vert_index;

                // updating the vertex adjacency lists is handled by _addVertex, so we don't have to do that here
            }

            // the midpoint vertex is hanging if the parent edge is "in" the mesh
            if (_edge_nodes[parent_edge_node_index].in_mesh)
            {
                const auto [it, success] = _hanging_vertices.insert({midpoint_vertices[edge_index], _edge_nodes[parent_edge_node_index].edge});
                if (success)
                    _latest_new_hanging_vertices.emplace_back(midpoint_vertices[edge_index], parent_node.vertices[vi], parent_node.vertices[vj]);
            }
        }
        
    }
}

void RefinedTetMesh::_createChildFaceNodesForElement(int element_tree_node_index, const std::array<int,6>& mid_verts, bool at_refinement_depth)
{
    ElementTreeNode& parent_node = _element_tree_nodes[element_tree_node_index];

    // lambda helper function to create child edge nodes and child face nodes for a given face
    // parent_face_node_index = the index of the parent face node
    // v1,v2,v3 = indices of the parent face vertices
    // m12, m13, m23 = midpoint vertices of the parent face (m12 is on edge 12, etc.)
    auto create_child_features_for_face = [&](int parent_face_node_index, int v1, int v2, int v3, int m12, int m13, int m23) -> void
    {
        // std::cout "  === Face node " << parent_face_node_index << std::endl;
        // the parent face node must be a leaf to be split
        if (!_face_nodes[parent_face_node_index].is_leaf)
            return;

        // is the child feature in the mesh? That depends on if its parent is in the mesh, or we are at the final refinement level
        bool child_feature_in_mesh = _face_nodes[parent_face_node_index].in_mesh || at_refinement_depth;

        // create EdgeNodes for the child edges - 3 of them constructed from the midpoint vertices
        EdgeNode child_edge1(m12, m13);
        child_edge1.parent_face_node = parent_face_node_index;
        child_edge1.in_mesh = child_feature_in_mesh;

        EdgeNode child_edge2(m12, m23);
        child_edge2.parent_face_node = parent_face_node_index;
        child_edge2.in_mesh = child_feature_in_mesh;

        EdgeNode child_edge3(m13, m23);
        child_edge3.parent_face_node = parent_face_node_index;
        child_edge3.in_mesh = child_feature_in_mesh;

        _face_nodes[parent_face_node_index].child_edge_nodes[0] = _edge_nodes.push_back(std::move(child_edge1));
        _face_nodes[parent_face_node_index].child_edge_nodes[1] = _edge_nodes.push_back(std::move(child_edge2));
        _face_nodes[parent_face_node_index].child_edge_nodes[2] = _edge_nodes.push_back(std::move(child_edge3));

        // update the vertex adjacency lists - there are 3 new edges connecting vertices
        _vertex_adjacent_vertices[m12].insert({m13, m23});
        _vertex_adjacent_vertices[m13].insert({m12, m23});
        _vertex_adjacent_vertices[m23].insert({m12, m13});

        // create FaceNodes for the child faces - 4 of them
        // whether these new subfaces are on the outer surface of the mesh depends on if the parent face is on the outer surface of the mesh
        FaceNode child_face1(v1, m12, m13);
        child_face1.parent_face_node = parent_face_node_index;
        child_face1.in_mesh = child_feature_in_mesh;
        child_face1.on_surface = _face_nodes[parent_face_node_index].on_surface;

        FaceNode child_face2(v2, m12, m23);
        child_face2.parent_face_node = parent_face_node_index;
        child_face2.in_mesh = child_feature_in_mesh;
        child_face2.on_surface = _face_nodes[parent_face_node_index].on_surface;

        FaceNode child_face3(v3, m13, m23);
        child_face3.parent_face_node = parent_face_node_index;
        child_face3.in_mesh = child_feature_in_mesh;
        child_face3.on_surface = _face_nodes[parent_face_node_index].on_surface;

        FaceNode child_face4(m12, m13, m23);
        child_face4.parent_face_node = parent_face_node_index;
        child_face4.in_mesh = child_feature_in_mesh;
        child_face4.on_surface = _face_nodes[parent_face_node_index].on_surface;

        _face_nodes[parent_face_node_index].child_face_nodes[0] = _face_nodes.push_back(std::move(child_face1));
        _face_nodes[parent_face_node_index].child_face_nodes[1] = _face_nodes.push_back(std::move(child_face2));
        _face_nodes[parent_face_node_index].child_face_nodes[2] = _face_nodes.push_back(std::move(child_face3));
        _face_nodes[parent_face_node_index].child_face_nodes[3] = _face_nodes.push_back(std::move(child_face4));

        // mark the parent face node as no longer being a leaf (it has children now!)
        _face_nodes[parent_face_node_index].is_leaf = false;
    };
    
    // F012
    create_child_features_for_face(parent_node.face_nodes[0], 
        parent_node.vertices[0], parent_node.vertices[1], parent_node.vertices[2],
        mid_verts[0], mid_verts[1], mid_verts[3]);
    // F013
    create_child_features_for_face(parent_node.face_nodes[1],
        parent_node.vertices[0], parent_node.vertices[1], parent_node.vertices[3],
        mid_verts[0], mid_verts[2], mid_verts[4]);
    // F023
    create_child_features_for_face(parent_node.face_nodes[2],
        parent_node.vertices[0], parent_node.vertices[2], parent_node.vertices[3],
        mid_verts[1], mid_verts[2], mid_verts[5]);
    // F123
    create_child_features_for_face(parent_node.face_nodes[3],
        parent_node.vertices[1], parent_node.vertices[2], parent_node.vertices[3],
        mid_verts[3], mid_verts[4], mid_verts[5]);
}

std::pair<int,int> RefinedTetMesh::_matchChildEdgeNodeIndices(int parent_edge_node_index, int lower)
{
    const EdgeNode& parent_edge_node = _edge_nodes[parent_edge_node_index];
    const EdgeNode& child_edge_node1 = _edge_nodes[parent_edge_node.child_edge_node1];
    if (child_edge_node1.edge.index1 == lower || child_edge_node1.edge.index2 == lower)
    {
        return {parent_edge_node.child_edge_node1, parent_edge_node.child_edge_node2};
    }
    else
    {
        return {parent_edge_node.child_edge_node2, parent_edge_node.child_edge_node1};
    }
}

std::tuple<int,int,int> RefinedTetMesh::_matchFaceNodeToChildEdgeNodeIndices(int parent_face_node_index, int lower, int middle)
{
    const FaceNode& parent_face_node = _face_nodes[parent_face_node_index];
    const EdgeNode& child_edge_node1 = _edge_nodes[parent_face_node.child_edge_nodes[0]];
    const EdgeNode& child_edge_node2 = _edge_nodes[parent_face_node.child_edge_nodes[1]];
    if (child_edge_node1.edge.index1 == lower || child_edge_node1.edge.index2 == lower)
    {
        if (child_edge_node1.edge.index1 == middle || child_edge_node1.edge.index2 == middle)
        {
            if (child_edge_node2.edge.index1 == lower || child_edge_node2.edge.index2 == lower)
                return {parent_face_node.child_edge_nodes[0], parent_face_node.child_edge_nodes[1], parent_face_node.child_edge_nodes[2]};
            else
                return {parent_face_node.child_edge_nodes[0], parent_face_node.child_edge_nodes[2], parent_face_node.child_edge_nodes[1]};
        }
        else
        {
            if (child_edge_node2.edge.index1 == lower || child_edge_node2.edge.index2 == lower)
                return {parent_face_node.child_edge_nodes[1], parent_face_node.child_edge_nodes[0], parent_face_node.child_edge_nodes[2]};
            else
                return {parent_face_node.child_edge_nodes[2], parent_face_node.child_edge_nodes[0], parent_face_node.child_edge_nodes[1]};
        }
    }
    else
    {
        if ( (child_edge_node2.edge.index1 == lower || child_edge_node2.edge.index2 == lower) && 
                (child_edge_node2.edge.index1 == middle || child_edge_node2.edge.index2 == middle) )
            return {parent_face_node.child_edge_nodes[1], parent_face_node.child_edge_nodes[2], parent_face_node.child_edge_nodes[0]};
        else
            return {parent_face_node.child_edge_nodes[2], parent_face_node.child_edge_nodes[1], parent_face_node.child_edge_nodes[0]};
    }
}

std::tuple<int,int,int,int> RefinedTetMesh::_matchFaceNodeToChildFaceNodeIndices(int parent_face_node_index, int v0, int v1, int v2, int m01, int m02, int m12)
{
    const FaceNode& parent_face_node = _face_nodes[parent_face_node_index];
    int ind1 = -1, ind2 = -1, ind3 = -1, ind4 = -1;
    for (const auto& child_face_node_index : parent_face_node.child_face_nodes)
    {
        const FaceNode& child_face_node = _face_nodes[child_face_node_index];
        if (ind1 < 0 && child_face_node.face == Face(v0, m01, m02))
        {
            ind1 = child_face_node_index;
            continue;
        }
        if (ind2 < 0 && child_face_node.face == Face(v1, m01, m12))
        {
            ind2 = child_face_node_index;
            continue;
        }
        if (ind3 < 0 && child_face_node.face == Face(v2, m02, m12))
        {
            ind3 = child_face_node_index;
            continue;
        }
        if (ind4 < 0 && child_face_node.face == Face(m01, m02, m12))
        {
            ind4 = child_face_node_index;
            continue;
        }
    }
    return {ind1, ind2, ind3, ind4};
}

bool RefinedTetMesh::refineElement(int element_index, int refinement_level, bool absolute)
{
    
    assert(elementValid(element_index));


    /** === Step 1: Prepare for the refinement algorithm. === */

    const Vec4i base_element = element(element_index);  // don't use a ref here since the element will soon be removed


    // clear the latest added vertices and elements
    _latest_new_vertices.clear();
    _latest_removed_vertices.clear();
    _latest_new_faces.clear();
    _latest_new_elements.clear();
    _latest_removed_elements.clear();
    _latest_new_hanging_vertices.clear();
    _latest_removed_hanging_vertices.clear();


    // what we really care about is the RELATIVE refinement level
    // when absolute=true, the specified refinement level is the absolute depth to refine to ==> it is possible we are already there!
    // here, we just initialize the relative refinement level to the refinement level passed in
    int rel_refinement_level = refinement_level;

    // find the ElementTreeNode associated with the specified element to refine, and 
    // create a new ElementTreeNode if one doesn't already exist for the element
    //


    int base_node_index;    // the index of the tree node we are refining
    auto surface_faces_range = _element_to_surface_faces_map.equal_range(element_index);    // iterators for the surface faces associated with the element - useful later

    // Case 1: an ElementTreeNode already exists for the element
    if (auto search = _element_to_tree_node_map.find(element_index); search != _element_to_tree_node_map.end())
    {
        base_node_index = search->second; 

        // if absolute = true, we are only refining up to an absolute refinement level
        // so must get the relative refinement level
        // if the relative refinement level <= 0, we don't need to do anything
        // note: this only applies for the case where an element tree node already exists, since it is only in this case that its level can be > 0
        if (absolute)
        {   
            rel_refinement_level = refinement_level - _element_tree_nodes[base_node_index].level;
            if (rel_refinement_level <= 0)
                return false;
        }

        // unset the element_index for the tree node, since the element will no longer exist
        _element_tree_nodes[base_node_index].element_index = ElementTreeNode::INVALID_INDEX;
        // update the element -> element tree node map
        _element_to_tree_node_map.erase(search);
    }
    // Case 2: an ElementTreeNode does not exist for the element, we must create one
    else
    {
        // create the initial ElementTreeNode struct for the base element that we are subdividing
        ElementTreeNode base_node(element(element_index), ElementTreeNode::INVALID_INDEX, 0);
            
        // find which faces of the base element (if any) are on the outer surface of the mesh
        bool f012_on_surface = false;
        bool f013_on_surface = false;
        bool f023_on_surface = false;
        bool f123_on_surface = false;
        for (auto it = surface_faces_range.first; it != surface_faces_range.second; it++)
        {
            const Vec3i& face_vec = face(it->second);
            const Vec4i& elem = base_element;
            Face surface_face(face_vec[0], face_vec[1], face_vec[2]);
            
            if (Face(elem[0], elem[1], elem[2]) == surface_face)        f012_on_surface = true;
            else if (Face(elem[0], elem[1], elem[3]) == surface_face)   f013_on_surface = true;
            else if (Face(elem[0], elem[2], elem[3]) == surface_face)   f023_on_surface = true;
            else if (Face(elem[1], elem[2], elem[3]) == surface_face)   f123_on_surface = true;
        }

        // set or create the edge nodes
        for (int vi = 0; vi < 4; vi++)
        {
            for (int vj = vi+1; vj < 4; vj++)
            {
                // map the nested sequence to a linear sequence
                int edge_index = 4*vi - (vi * (vi + 1)) / 2 + (vj - vi - 1);

                Edge edge(base_node.vertices[vi], base_node.vertices[vj]);
                // search for the edge in the Edge -> Edge Node map
                auto search = _edge_to_edge_node_map.find(edge);
                if (search != _edge_to_edge_node_map.end())
                {
                    // the edge node already exists!
                    base_node.edge_nodes[edge_index] = search->second;
                }
                else
                {
                    // the edge node does not exist, create a new one
                    int new_edge_node_index = _edge_nodes.emplace_back(edge);
                    // if the edge has more than one element associated with it (i.e. the element we're refining + another)
                    // then the edge node is "in" the mesh
                    if (_edge_to_elements_map.count(edge) > 1)
                    {
                        _edge_nodes[new_edge_node_index].in_mesh = true;
                    }
                    base_node.edge_nodes[edge_index] = new_edge_node_index;
                    _edge_to_edge_node_map.insert({edge, new_edge_node_index});
                }
            }
        }


        // set or create the face nodes

        // lambda helper for doing this for a given face
        auto set_or_create_face_node = [&](int fi, int v1, int v2, int v3, bool surface) -> void
        {
            Face face(v1, v2, v3);
            // search for the face in the Face -> Face Node map
            auto search = _face_to_face_node_map.find(face);
            if (search != _face_to_face_node_map.end())
            {
                // the face node already exists!
                base_node.face_nodes[fi] = search->second;
            }
            else
            {
                // the face node does not exist, create a new one
                int new_face_node_index = _face_nodes.emplace_back(face);
                _face_nodes[new_face_node_index].on_surface = surface;

                // if the face has more than one element associated with it (i.e. the element we're refining + another)
                // then the face node is "in" the mesh
                if (_face_to_elements_map.count(face) > 1)
                {
                    _face_nodes[new_face_node_index].in_mesh = true;
                }

                base_node.face_nodes[fi] = new_face_node_index;
                _face_to_face_node_map.insert({face, new_face_node_index});
            }
        };

        set_or_create_face_node(0, base_node.vertices[0], base_node.vertices[1], base_node.vertices[2], f012_on_surface);    // F012
        set_or_create_face_node(1, base_node.vertices[0], base_node.vertices[1], base_node.vertices[3], f013_on_surface);    // F013
        set_or_create_face_node(2, base_node.vertices[0], base_node.vertices[2], base_node.vertices[3], f023_on_surface);    // F023
        set_or_create_face_node(3, base_node.vertices[1], base_node.vertices[2], base_node.vertices[3], f123_on_surface);    // F123

        // add the base element tree node that we just created to the tree nodes vector
        base_node_index = _element_tree_nodes.push_back(std::move(base_node));
    }

    /** === Step 3: Remove the element from the mesh === */

    // increment the topology version
    _topology_version++;

    // remove surface faces associated with the element
    for (auto it = surface_faces_range.first; it != surface_faces_range.second; it++)
    {
        _faces.erase(it->second);

        // note: we do not need to update the surface face -> element map since that will just be overwritten by whatever new faces are added
    }

    // add the removed parent element to the latest removed elements
    _latest_removed_elements.emplace_back(element_index, base_element, elementRestVolume(element_index));

    // update the edge -> element, face -> element, and element -> surface face maps
    // we need to wait to update the vertex -> element map, because if we do it now, we might accidentally remove some of the original tet's vertices!
    _updateEdgeElementMapForRemovedElement(element_index);
    _updateFaceElementMapForRemovedElement(element_index);
    _updateElementSurfaceFaceMapForRemovedElement(element_index);

    // update the feature hierarchy (i.e. whether or not a feature has an ancestor feature in the mesh or not)
    _prepareFeatureTreeForRefinedElement(base_node_index, rel_refinement_level);

    /** === Step 4: Refine the element. === */

    // std::cout "\n\n\n======================\nRefining element " << element_index << "\n======================" << std::endl;
    
    // recursively keep track of "parent" elements that we want to subdivide
    std::vector<int> parent_nodes = {base_node_index};
    int num_new_tets = 1;   // calculate the number of new tets to be added at each refinement level

    for (int level = 0; level < rel_refinement_level; level++)
    {
        num_new_tets *= 8;

        // std::cout "Level " << level << "..." << std::endl;

        // create a vector to store the next level of parent elements
        // note that this only applies when we are not at the deepest refinement level
        std::vector<int> next_parent_nodes;
        if (level < rel_refinement_level-1)
            next_parent_nodes.reserve(num_new_tets);

        // add newest level of midpoint vertices for each parent element at this level
        std::array<int,6> mid_verts;
        for (const auto& parent_node_index : parent_nodes)
        {
            // reserve space for the new ElementTreeNodes ahead of time so our reference is not invalidated
            _element_tree_nodes.reserve(_element_tree_nodes.totalSize()+8);

            ElementTreeNode& parent_node = _element_tree_nodes[parent_node_index];

            /** Create new features in the feature hierarchy */

            // each edge in the parent element -> 2 sub edges
            // each edge in the parent element also will have a new vertex created at its midpoint
            _createMidpointVerticesAndChildEdgeNodesForElement(parent_node_index, mid_verts, level == rel_refinement_level-1);

            // each face in the parent element -> 4 sub faces and 3 sub edges
            // FaceNode& parent_face_node = _face_nodes[parent_face_node_index];
            _createChildFaceNodesForElement(parent_node_index, mid_verts, level == rel_refinement_level-1);

            // splitting a tet produces 8 "internal" faces that do not have a parent face in the tet that was split
            // create the face nodes associated with these faces
            int f456_node_index = _face_nodes.emplace_back(mid_verts[0], mid_verts[1], mid_verts[2]);   _face_nodes[f456_node_index].in_mesh = level == rel_refinement_level-1;
            int f478_node_index = _face_nodes.emplace_back(mid_verts[0], mid_verts[3], mid_verts[4]);   _face_nodes[f478_node_index].in_mesh = level == rel_refinement_level-1;  
            int f579_node_index = _face_nodes.emplace_back(mid_verts[1], mid_verts[3], mid_verts[5]);   _face_nodes[f579_node_index].in_mesh = level == rel_refinement_level-1;
            int f689_node_index = _face_nodes.emplace_back(mid_verts[2], mid_verts[4], mid_verts[5]);   _face_nodes[f689_node_index].in_mesh = level == rel_refinement_level-1;
            int f467_node_index = _face_nodes.emplace_back(mid_verts[0], mid_verts[2], mid_verts[3]);   _face_nodes[f467_node_index].in_mesh = level == rel_refinement_level-1;
            int f679_node_index = _face_nodes.emplace_back(mid_verts[2], mid_verts[3], mid_verts[5]);   _face_nodes[f679_node_index].in_mesh = level == rel_refinement_level-1;
            int f567_node_index = _face_nodes.emplace_back(mid_verts[1], mid_verts[2], mid_verts[3]);   _face_nodes[f567_node_index].in_mesh = level == rel_refinement_level-1;
            int f678_node_index = _face_nodes.emplace_back(mid_verts[2], mid_verts[3], mid_verts[4]);   _face_nodes[f678_node_index].in_mesh = level == rel_refinement_level-1;

            // splitting a tet also produces 1 "internal" edge that does not have a parent edge or face in the parent tet that was split
            // create the edge node associated with this edge
            int e67_node_index = _edge_nodes.emplace_back(mid_verts[2], mid_verts[3]);
            _edge_nodes[e67_node_index].in_mesh = level == rel_refinement_level-1;
            // update the vertex adjacency lists
            _vertex_adjacent_vertices[mid_verts[2]].insert(mid_verts[3]);
            _vertex_adjacent_vertices[mid_verts[3]].insert(mid_verts[2]);


            /** Create the child element tree nodes */

            // indices for edges and faces created programmatically

            auto [e04_node_index, e14_node_index] = _matchChildEdgeNodeIndices(parent_node.edge_nodes[0], parent_node.vertices[0]);
            auto [e05_node_index, e25_node_index] = _matchChildEdgeNodeIndices(parent_node.edge_nodes[1], parent_node.vertices[0]);
            auto [e06_node_index, e36_node_index] = _matchChildEdgeNodeIndices(parent_node.edge_nodes[2], parent_node.vertices[0]);
            auto [e17_node_index, e27_node_index] = _matchChildEdgeNodeIndices(parent_node.edge_nodes[3], parent_node.vertices[1]);
            auto [e18_node_index, e38_node_index] = _matchChildEdgeNodeIndices(parent_node.edge_nodes[4], parent_node.vertices[1]);
            auto [e29_node_index, e39_node_index] = _matchChildEdgeNodeIndices(parent_node.edge_nodes[5], parent_node.vertices[2]);

            auto [e45_node_index, e47_node_index, e57_node_index] = _matchFaceNodeToChildEdgeNodeIndices(parent_node.face_nodes[0], mid_verts[0], mid_verts[1]);
            auto [e46_node_index, e48_node_index, e68_node_index] = _matchFaceNodeToChildEdgeNodeIndices(parent_node.face_nodes[1], mid_verts[0], mid_verts[2]);
            auto [e56_node_index, e59_node_index, e69_node_index] = _matchFaceNodeToChildEdgeNodeIndices(parent_node.face_nodes[2], mid_verts[1], mid_verts[2]);
            auto [e78_node_index, e79_node_index, e89_node_index] = _matchFaceNodeToChildEdgeNodeIndices(parent_node.face_nodes[3], mid_verts[3], mid_verts[4]);

            auto [f045_node_index, f147_node_index, f257_node_index, f457_node_index] = _matchFaceNodeToChildFaceNodeIndices(
                parent_node.face_nodes[0], parent_node.vertices[0], parent_node.vertices[1], parent_node.vertices[2], mid_verts[0], mid_verts[1], mid_verts[3]
            );
            auto [f046_node_index, f148_node_index, f368_node_index, f468_node_index] = _matchFaceNodeToChildFaceNodeIndices(
                parent_node.face_nodes[1], parent_node.vertices[0], parent_node.vertices[1], parent_node.vertices[3], mid_verts[0], mid_verts[2], mid_verts[4]
            );
            auto [f056_node_index, f259_node_index, f369_node_index, f569_node_index] = _matchFaceNodeToChildFaceNodeIndices(
                parent_node.face_nodes[2], parent_node.vertices[0], parent_node.vertices[2], parent_node.vertices[3], mid_verts[1], mid_verts[2], mid_verts[5]
            );
            auto [f178_node_index, f279_node_index, f389_node_index, f789_node_index] = _matchFaceNodeToChildFaceNodeIndices(
                parent_node.face_nodes[3], parent_node.vertices[1], parent_node.vertices[2], parent_node.vertices[3], mid_verts[3], mid_verts[4], mid_verts[5]
            );

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

            // std::cout "Parent element: " << parent_node.vertices.transpose() << std::endl;
            // std::cout "Midpoint verts: " << mid_verts[0] << " " << mid_verts[1] << " " << mid_verts[2] << " " << mid_verts[3] << " " << mid_verts[4] << " " << mid_verts[5] << std::endl;
            // std::cout "elem1: " << elem1.transpose() << std::endl;
            // std::cout "elem2: " << elem2.transpose() << std::endl;
            // std::cout "elem3: " << elem3.transpose() << std::endl;
            // std::cout "elem4: " << elem4.transpose() << std::endl;
            // std::cout "elem5: " << elem5.transpose() << std::endl;
            // std::cout "elem6: " << elem6.transpose() << std::endl;
            // std::cout "elem7: " << elem7.transpose() << std::endl;
            // std::cout "elem8: " << elem8.transpose() << std::endl;

            // create tree nodes for each element
            int enode1 = _element_tree_nodes.emplace_back(elem1, parent_node_index, parent_node.level+1, 
                std::array<int,6>{e04_node_index, e05_node_index, e06_node_index, e45_node_index, e46_node_index, e56_node_index},
                std::array<int,4>{f045_node_index, f046_node_index, f056_node_index, f456_node_index}
            );
            
            int enode2 = _element_tree_nodes.emplace_back(elem2, parent_node_index, parent_node.level+1, 
                std::array<int,6>{e14_node_index, e17_node_index, e18_node_index, e47_node_index, e48_node_index, e78_node_index},
                std::array<int,4>{f147_node_index, f148_node_index, f178_node_index, f478_node_index}
            );

            int enode3 = _element_tree_nodes.emplace_back(elem3, parent_node_index, parent_node.level+1, 
                std::array<int,6>{e25_node_index, e27_node_index, e29_node_index, e57_node_index, e59_node_index, e79_node_index},
                std::array<int,4>{f257_node_index, f259_node_index, f279_node_index, f579_node_index}
            );

            int enode4 = _element_tree_nodes.emplace_back(elem4, parent_node_index, parent_node.level+1, 
                std::array<int,6>{e68_node_index, e69_node_index, e36_node_index, e89_node_index, e38_node_index, e39_node_index},
                std::array<int,4>{f689_node_index, f368_node_index, f369_node_index, f389_node_index}
            );

            int enode5 = _element_tree_nodes.emplace_back(elem5, parent_node_index, parent_node.level+1,
                std::array<int,6>{e45_node_index, e47_node_index, e46_node_index, e57_node_index, e56_node_index, e67_node_index},
                std::array<int,4>{f457_node_index, f456_node_index, f467_node_index, f567_node_index}
            );

            int enode6 = _element_tree_nodes.emplace_back(elem6, parent_node_index, parent_node.level+1,
                std::array<int,6>{e46_node_index, e47_node_index, e48_node_index, e67_node_index, e68_node_index, e78_node_index},
                std::array<int,4>{f467_node_index, f468_node_index, f478_node_index, f678_node_index}
            );

            int enode7 = _element_tree_nodes.emplace_back(elem7, parent_node_index, parent_node.level+1,
                std::array<int,6>{e68_node_index, e69_node_index, e67_node_index, e89_node_index, e78_node_index, e79_node_index},
                std::array<int,4>{f689_node_index, f678_node_index, f679_node_index, f789_node_index}
            );

            int enode8 = _element_tree_nodes.emplace_back(elem8, parent_node_index, parent_node.level+1,
                std::array<int,6>{e59_node_index, e56_node_index, e57_node_index, e69_node_index, e79_node_index, e67_node_index},
                std::array<int,4>{f569_node_index, f579_node_index, f567_node_index, f679_node_index}
            );

            // add each child node to the parent
            parent_node.children.insert(parent_node.children.end(), {enode1, enode2, enode3, enode4, enode5, enode6, enode7, enode8});

            if (level == rel_refinement_level-1)
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

    return true;
}

bool RefinedTetMesh::coarsenElement(int element_index, int coarsening_level, bool absolute)
{
    // clear the latest added vertices and elements
    _latest_new_vertices.clear();
    _latest_removed_vertices.clear();
    _latest_new_faces.clear();
    _latest_new_elements.clear();
    _latest_removed_elements.clear();
    _latest_new_hanging_vertices.clear();
    _latest_removed_hanging_vertices.clear();

    // get the element tree node associated with this element
    auto search = _element_to_tree_node_map.find(element_index);

    // if the element is not a result of refinement, return
    if (search == _element_to_tree_node_map.end())
        return false;

    const ElementTreeNode& leaf_node = _element_tree_nodes[search->second];

    // if the element doesn't have a parent (for some reason), remove it and do nothing
    if (leaf_node.parent == ElementTreeNode::INVALID_INDEX)
    {
        std::cout << "Leaf element tree node does not have a parent!!" << std::endl;
        std::cout << "Leaf node level: " << leaf_node.level << std::endl;
        // assert(0);  // this shouldn't happen
        if (leaf_node.isLeaf())
        {
            _element_tree_nodes.erase(search->second);
            _element_to_tree_node_map.erase(element_index);
        }
        else
        {
            std::cout << "Element tree node does not have a parent AND is not a leaf! What?" << std::endl;
            assert(0);  // this really shouldn't happen
        }
        
        return false;
    }

    // if absolute = true, we are only coarsening up to an absolute coarsening level
    // so must get the relative coarsening level
    // if the relative coarsening level <= 0, we don't need to do anything
    int rel_coarsening_level = coarsening_level;
    if (absolute)
    {
        rel_coarsening_level = leaf_node.level - coarsening_level;
        if (rel_coarsening_level <= 0)
            return false;
    }

    // get the root of the tree branch that we are going to replace this element (and its relatives) with
    int root_index = leaf_node.parent;
    int cur_level = leaf_node.level - 1;
    while (cur_level > leaf_node.level - rel_coarsening_level)
    {
        // need to ensure that the parent has all of its children
        // otherwise we might lose information!
        int parent_index = _element_tree_nodes[root_index].parent;
        if (_element_tree_nodes[parent_index].children.size() < 8)
            break;

        // parent has all of its children - keep traversing up the tree
        root_index = parent_index;
        cur_level--;
    }

    // increment the topology version
    _topology_version++;

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

                _latest_removed_elements.emplace_back(node.element_index, node.vertices, elementRestVolume(node.element_index));

                _updateElementMapsForRemovedElement(node.element_index);
                _elements.erase(node.element_index);
                _element_to_tree_node_map.erase(node.element_index);
            }
            else
            {
                // if the node doesn't have an associated element index, it is a parent node
                // check each of its vertices to see if it is still used in the mesh
                // if not - remove it from the mesh!
                for (int k = 0; k < 4; k++)
                {
                    std::vector<int>& vk_map = _vertex_to_elements_map[node.vertices[k]];
                    // std::cout "VK map size for vertex " << node.vertices[k] << ": " << vk_map.size() << std::endl;
                    if (vk_map.size() == 0)
                    {
                        // std::cout "Removing vertex from parent element " << node.vertices[k] << "! No longer in the mesh." << std::endl;
                        _vertices.erase(node.vertices[k]);
                        /** TODO: somehow get the parent vertices of this removed vertex? Is this necessary? */
                        _latest_removed_vertices.emplace_back(node.vertices[k], -1, -1);

                        int hanging_vert_removed = _hanging_vertices.erase(node.vertices[k]);
                        if (hanging_vert_removed)
                            _latest_removed_hanging_vertices.push_back(node.vertices[k]);

                        
                    }
                }
                
            }

            _updateFeatureHierarchyForRemovedElementTreeNode(node_index);

            // remove the node
            _element_tree_nodes.erase(node_index);
        }   
    }

    // we have removed all the children so update the root node to reflect this
    root_node.children.clear();

    // the edge nodes that are not leaves in the root node now have hanging vertices
    for (const auto& edge_node_vertex : root_node.edge_nodes)
    {
        EdgeNode& edge_node = _edge_nodes[edge_node_vertex];
        if (!edge_node.is_leaf)
        {
            const auto [it, success] = _hanging_vertices.insert({edge_node.child_vertex, edge_node.edge});
            if (success)
                _latest_new_hanging_vertices.emplace_back(edge_node.child_vertex, edge_node.edge.index1, edge_node.edge.index2);
        }
        edge_node.in_mesh = true;
    }

    return true;
}

std::unordered_set<int> RefinedTetMesh::verifyHangingVertices() const
{
    std::unordered_set<int> hanging_verts;

    // iterate through all the edges in the mesh
    for (const auto& it : _edge_to_elements_map)
    {
        Edge edge = it.first;
        // // std::cout "Edge: " << edge.index1 << ", " << edge.index2 << std::endl;

        for (const auto& v_ind : _vertices.validIndices())
        {
            if (static_cast<int>(v_ind) == edge.index1 || static_cast<int>(v_ind) == edge.index2)
                continue;

            // // std::cout "Testing v " << v_ind << std::endl;
            
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
                // // std::cout "\tvertex " << v_ind << " on edge! " << std::endl;
                hanging_verts.insert(v_ind);
            }
        }
    }

    // iterate through all the faces in the mesh
    for (const auto& it : _face_to_elements_map)
    {
        Face face = it.first;
        // // std::cout "Face: " << face.index1 << ", " << face.index2 << ", " << face.index3 << std::endl;

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
            if (std::abs((p - A).dot(normal)) > 1e-8)
                continue;
            
            // Compute barycentric coordinates
            Real dot00 = v0.dot(v0);
            Real dot01 = v0.dot(v1);
            Real dot02 = v0.dot(v2);
            Real dot11 = v1.dot(v1);
            Real dot12 = v1.dot(v2);
            
            Real denom = dot00 * dot11 - dot01 * dot01;
            if (std::abs(denom) < 1e-8)
                continue;
            
            Real inv_denom = 1.0 / denom;
            Real u = (dot11 * dot02 - dot01 * dot12) * inv_denom;
            Real v = (dot00 * dot12 - dot01 * dot02) * inv_denom;
            
            // Check if point is in triangle (with small tolerance for numerical error)
            if ( (u >= 1e-8) && (v >= 1e-8) && (u + v <= 1 - 1e-8) )
            {
                // // std::cout "\tvertex " << v_ind << " on face! " << std::endl; 
                hanging_verts.insert(v_ind);
            }
        }
    }

    return hanging_verts;
}

} // namespace Geometry