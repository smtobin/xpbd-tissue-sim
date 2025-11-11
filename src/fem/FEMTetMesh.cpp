#include "fem/FEMTetMesh.hpp"

namespace FEM
{

FEMTetMesh::FEMTetMesh(Geometry::TetMesh* mesh)
    : _mesh(mesh)
{

}

/** Element-related computations */

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

/** Face-related computations */

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

/////////////////////

/** Refinement */

int FEMTetMesh::_addRefinedVertex(int parent_index1, int parent_index2,
     const Vec4i& base_element, RefinedElement* refined_element)
{
    Edge parent_edge(parent_index1, parent_index2);

    TombstoneVector<Vec3r>& global_vertices = _mesh->vertices();
    
    const typename RefinedElement::ChildVertex& p1 = refined_element->child_vertices.at(parent_index1);
    const typename RefinedElement::ChildVertex& p2 = refined_element->child_vertices.at(parent_index2);
    bool p1_is_edge = p1.edge != RefinedElement::TetEdge::NONE;
    bool p2_is_edge = p2.edge != RefinedElement::TetEdge::NONE;
    bool p1_is_face = p1.face != RefinedElement::TetFace::NONE;
    bool p2_is_face = p2.face != RefinedElement::TetFace::NONE;
    
    int new_level = std::max(p1.level, p2.level) + 1;
    typename RefinedElement::ChildVertex new_vert(-1, new_level, RefinedElement::TetEdge::NONE, RefinedElement::TetFace::NONE);

    // check if this RefinedElement already has a vertex at the midpoint of this edge
    if (auto search = refined_element->edge_to_vertex_map.find(parent_edge); search != refined_element->edge_to_vertex_map.end())
    {
        new_vert.index = search->second;
        return new_vert.index;
    }

    // now we need to determine if the new vertex is on an edge or a face, or neither
    // Case 1: one parent is not a base, edge, or face vertex - the child vertex is neither a base, edge, or face vertex
    if ( (p1.level != 0 && !p1_is_edge && !p1_is_face) || (p2.level != 0 && !p2_is_edge && !p2_is_face) )
    {
        // std::cout << "Case 1" << std::endl;
        // do nothing - the child is neither a base, edge or face
    }
    // Case 2: both parents are edge vertices:
    //   if the edges are adjacent the child is on a face
    //   if the edges are the same the child is on that edge
    //   otherwise (i.e. the edges do not share a vertex) the child is not on a face or edge
    else if ( p1_is_edge && p2_is_edge )
    {
        // std::cout << "Case 2" << std::endl;
        // Case 2a: the parents are on the same edge
        if (p1.edge == p2.edge)
        {
            // the child is on that same edge
            new_vert.edge = p1.edge;
        }
        // Case 2b: the parents are on adjacent edges (i.e. edges on the same face)
        /** TODO: There's gotta be a better way to do this - this stinks */
        // F123
        else if (p1.edge == RefinedElement::TetEdge::E12 && p2.edge == RefinedElement::TetEdge::E13) new_vert.face = RefinedElement::TetFace::F123;
        else if (p2.edge == RefinedElement::TetEdge::E12 && p1.edge == RefinedElement::TetEdge::E13) new_vert.face = RefinedElement::TetFace::F123;

        else if (p1.edge == RefinedElement::TetEdge::E12 && p2.edge == RefinedElement::TetEdge::E23) new_vert.face = RefinedElement::TetFace::F123;
        else if (p2.edge == RefinedElement::TetEdge::E12 && p1.edge == RefinedElement::TetEdge::E23) new_vert.face = RefinedElement::TetFace::F123;

        else if (p1.edge == RefinedElement::TetEdge::E13 && p2.edge == RefinedElement::TetEdge::E23) new_vert.face = RefinedElement::TetFace::F123;
        else if (p2.edge == RefinedElement::TetEdge::E13 && p1.edge == RefinedElement::TetEdge::E23) new_vert.face = RefinedElement::TetFace::F123;

        // F124
        else if (p1.edge == RefinedElement::TetEdge::E12 && p2.edge == RefinedElement::TetEdge::E14) new_vert.face = RefinedElement::TetFace::F124;
        else if (p2.edge == RefinedElement::TetEdge::E12 && p1.edge == RefinedElement::TetEdge::E14) new_vert.face = RefinedElement::TetFace::F124;

        else if (p1.edge == RefinedElement::TetEdge::E12 && p2.edge == RefinedElement::TetEdge::E24) new_vert.face = RefinedElement::TetFace::F124;
        else if (p2.edge == RefinedElement::TetEdge::E12 && p1.edge == RefinedElement::TetEdge::E24) new_vert.face = RefinedElement::TetFace::F124;

        else if (p1.edge == RefinedElement::TetEdge::E14 && p2.edge == RefinedElement::TetEdge::E24) new_vert.face = RefinedElement::TetFace::F124;
        else if (p2.edge == RefinedElement::TetEdge::E14 && p1.edge == RefinedElement::TetEdge::E24) new_vert.face = RefinedElement::TetFace::F124;

        // F134
        else if (p1.edge == RefinedElement::TetEdge::E13 && p2.edge == RefinedElement::TetEdge::E14) new_vert.face = RefinedElement::TetFace::F134;
        else if (p2.edge == RefinedElement::TetEdge::E13 && p1.edge == RefinedElement::TetEdge::E14) new_vert.face = RefinedElement::TetFace::F134;

        else if (p1.edge == RefinedElement::TetEdge::E13 && p2.edge == RefinedElement::TetEdge::E34) new_vert.face = RefinedElement::TetFace::F134;
        else if (p2.edge == RefinedElement::TetEdge::E13 && p1.edge == RefinedElement::TetEdge::E34) new_vert.face = RefinedElement::TetFace::F134;

        else if (p1.edge == RefinedElement::TetEdge::E14 && p2.edge == RefinedElement::TetEdge::E34) new_vert.face = RefinedElement::TetFace::F134;
        else if (p2.edge == RefinedElement::TetEdge::E14 && p1.edge == RefinedElement::TetEdge::E34) new_vert.face = RefinedElement::TetFace::F134;

        // F234
        else if (p1.edge == RefinedElement::TetEdge::E23 && p2.edge == RefinedElement::TetEdge::E24) new_vert.face = RefinedElement::TetFace::F234;
        else if (p2.edge == RefinedElement::TetEdge::E23 && p1.edge == RefinedElement::TetEdge::E24) new_vert.face = RefinedElement::TetFace::F234;

        else if (p1.edge == RefinedElement::TetEdge::E23 && p2.edge == RefinedElement::TetEdge::E34) new_vert.face = RefinedElement::TetFace::F234;
        else if (p2.edge == RefinedElement::TetEdge::E23 && p1.edge == RefinedElement::TetEdge::E34) new_vert.face = RefinedElement::TetFace::F234;

        else if (p1.edge == RefinedElement::TetEdge::E24 && p2.edge == RefinedElement::TetEdge::E34) new_vert.face = RefinedElement::TetFace::F234;
        else if (p2.edge == RefinedElement::TetEdge::E24 && p1.edge == RefinedElement::TetEdge::E34) new_vert.face = RefinedElement::TetFace::F234;

        // Case 2c: the parents are on non-adjacent edges
        // do nothing, child is interior
    }

    // Case 3: both parents are face vertices
    //   if the faces are the same the child is on that face
    //   otherwise the child is on the interior
    else if (p1_is_face && p2_is_face)
    {
        // std::cout << "Case 3" << std::endl;
        // Case 3a: both parents on the same face
        if (p1.face == p2.face)
        {
            new_vert.face = p1.face;
        }
        // Case 3b: parents on different faces
        else
        {
            // do nothing - child is on the interior
        }
    }
    // Case 4: one parent is a face vertex, the other is an edge
    //      if the edge is adjacent to the face, the child is on that face
    //      otherwise, the child is on the interior
    else if ( (p1_is_face && p2_is_edge) || (p2_is_face && p1_is_edge))
    {
        // std::cout << "Case 4" << std::endl;
        RefinedElement::TetFace face = p1_is_face ? p1.face : p2.face;
        RefinedElement::TetEdge edge = p1_is_edge ? p1.edge : p2.edge;

        // Case 4a: edge is adjacent to the face
        if ( (face == RefinedElement::TetFace::F123 && (edge == RefinedElement::TetEdge::E12 || edge == RefinedElement::TetEdge::E13 || edge == RefinedElement::TetEdge::E23)) ||
             (face == RefinedElement::TetFace::F124 && (edge == RefinedElement::TetEdge::E12 || edge == RefinedElement::TetEdge::E14 || edge == RefinedElement::TetEdge::E24)) ||
             (face == RefinedElement::TetFace::F134 && (edge == RefinedElement::TetEdge::E13 || edge == RefinedElement::TetEdge::E14 || edge == RefinedElement::TetEdge::E34)) ||
             (face == RefinedElement::TetFace::F234 && (edge == RefinedElement::TetEdge::E23 || edge == RefinedElement::TetEdge::E24 || edge == RefinedElement::TetEdge::E34)))
        {
            new_vert.face = face;
        }
        // Case 4b: edge is not adjacent to the face
        else
        {
            // do nothing - child is on the interior
            // std::cout << "Edge: " << static_cast<int>(edge) << "\tFace: " << static_cast<int>(face) << std::endl;
            // std::cout << "Parent 1:\n\tIndex: " << p1.index << "\n\tLevel: " << p1.level << "\n\tEdge: " << static_cast<int>(p1.edge) << "\n\tFace: " << static_cast<int>(p1.face) << std::endl;
            // std::cout << "Parent 2:\n\tIndex: " << p2.index << "\n\tLevel: " << p2.level << "\n\tEdge: " << static_cast<int>(p2.edge) << "\n\tFace: " << static_cast<int>(p2.face) << std::endl;
            // assert(0);
        }
    }
    // Case 5: one parent is a edge vertex, the other is a base vertex
    //      if the base vertex borders the edge, the child is on that edge
    //      if the base vertex and the edge share a face, then the child is on that face (shouldn't happen)
    else if ( (p1_is_edge && p2.level == 0) || (p2_is_edge && p1.level == 0) )
    {
        // std::cout << "Case 5" << std::endl;
        RefinedElement::TetEdge edge = p1_is_edge ? p1.edge : p2.edge;
        int vert = p1.level == 0 ? p1.index : p2.index;

        // Case 5a: the base vertex borders the edge
        if ( (edge == RefinedElement::TetEdge::E12 && (vert == base_element[0] || vert == base_element[1])) ||
             (edge == RefinedElement::TetEdge::E13 && (vert == base_element[0] || vert == base_element[2])) ||
             (edge == RefinedElement::TetEdge::E14 && (vert == base_element[0] || vert == base_element[3])) ||
             (edge == RefinedElement::TetEdge::E23 && (vert == base_element[1] || vert == base_element[2])) ||
             (edge == RefinedElement::TetEdge::E24 && (vert == base_element[1] || vert == base_element[3])) ||
             (edge == RefinedElement::TetEdge::E34 && (vert == base_element[2] || vert == base_element[3])) )
        {
            new_vert.edge = edge;
        }
        // Case 5b: the base vertex does not border the edge
        else
        {
            // they share a face, but this shouldn't happen, so throw an error rather than handling it
            std::cout << "Edge: " << static_cast<int>(edge) << "\tVert: " << vert << std::endl;
            std::cout << "Parent 1:\n\tIndex: " << p1.index << "\n\tLevel: " << p1.level << "\n\tEdge: " << static_cast<int>(p1.edge) << "\n\tFace: " << static_cast<int>(p1.face) << std::endl;
            std::cout << "Parent 2:\n\tIndex: " << p2.index << "\n\tLevel: " << p2.level << "\n\tEdge: " << static_cast<int>(p2.edge) << "\n\tFace: " << static_cast<int>(p2.face) << std::endl;
            assert(0);
        }
    }
    // Case 6: both parents are base vertices
    else if ( (p1.level == 0 && p2.level == 0) )
    {
        // std::cout << "Case 6" << std::endl;
        if      (p1.index == base_element[0] && p2.index == base_element[1])   new_vert.edge = RefinedElement::TetEdge::E12;
        else if (p2.index == base_element[0] && p1.index == base_element[1])   new_vert.edge = RefinedElement::TetEdge::E12;

        else if (p1.index == base_element[0] && p2.index == base_element[2])   new_vert.edge = RefinedElement::TetEdge::E13;
        else if (p2.index == base_element[0] && p1.index == base_element[2])   new_vert.edge = RefinedElement::TetEdge::E13;

        else if (p1.index == base_element[0] && p2.index == base_element[3])   new_vert.edge = RefinedElement::TetEdge::E14;
        else if (p2.index == base_element[0] && p1.index == base_element[3])   new_vert.edge = RefinedElement::TetEdge::E14;

        else if (p1.index == base_element[1] && p2.index == base_element[2])   new_vert.edge = RefinedElement::TetEdge::E23;
        else if (p2.index == base_element[1] && p1.index == base_element[2])   new_vert.edge = RefinedElement::TetEdge::E23;

        else if (p1.index == base_element[1] && p2.index == base_element[3])   new_vert.edge = RefinedElement::TetEdge::E24;
        else if (p2.index == base_element[1] && p1.index == base_element[3])   new_vert.edge = RefinedElement::TetEdge::E24;

        else if (p1.index == base_element[2] && p2.index == base_element[3])   new_vert.edge = RefinedElement::TetEdge::E34;
        else if (p2.index == base_element[2] && p1.index == base_element[3])   new_vert.edge = RefinedElement::TetEdge::E34;
    }
    else
    {
        // something weird happened, throw an error
        std::cout << "Parent 1:\n\tIndex: " << p1.index << "\n\tLevel: " << p1.level << "\n\tEdge: " << static_cast<int>(p1.edge) << "\n\tFace: " << static_cast<int>(p1.face) << std::endl;
        std::cout << "Parent 2:\n\tIndex: " << p2.index << "\n\tLevel: " << p2.level << "\n\tEdge: " << static_cast<int>(p2.edge) << "\n\tFace: " << static_cast<int>(p2.face) << std::endl;
        assert(0);
    }

    // do stuff if the new vertex was determined to be on an edge or a face of the base tet
    // i.e. add it to the global edge_vertices and/or face_vertices maps
    if (new_vert.edge != RefinedElement::TetEdge::NONE)
    {
        Edge edge;
        if (new_vert.edge == RefinedElement::TetEdge::E12)  edge = Edge(base_element[0], base_element[1]);
        else if (new_vert.edge == RefinedElement::TetEdge::E13)  edge = Edge(base_element[0], base_element[2]);
        else if (new_vert.edge == RefinedElement::TetEdge::E14)  edge = Edge(base_element[0], base_element[3]);
        else if (new_vert.edge == RefinedElement::TetEdge::E23)  edge = Edge(base_element[1], base_element[2]);
        else if (new_vert.edge == RefinedElement::TetEdge::E24)  edge = Edge(base_element[1], base_element[3]);
        else if (new_vert.edge == RefinedElement::TetEdge::E34)  edge = Edge(base_element[2], base_element[3]);
        
        // look for the edge vertex in the global edge vertices list
        bool exists = false;
        auto range = _edge_vertices.equal_range(edge);
        for (auto it = range.first; it != range.second; it++)
        {
            if (it->second.parent_index1 == std::min(parent_index1, parent_index2) && 
                it->second.parent_index2 == std::max(parent_index1, parent_index2))
            {
                /** TODO: update whether or not the node is hanging!!!
                 * 
                 * 
                 */

                // this vertex already exists! increment the shared counter
                it->second.shared_count++;
                // set the index in the ChildVertex struct to the index of the existing vertex
                new_vert.index = it->second.index;
                exists = true;
            }
        }

        // the edge vertex was not found, so create a new one
        if (!exists)
        {
            int new_index = global_vertices.push_back( (global_vertices.at(parent_index1) + global_vertices.at(parent_index2)) / 2.0 );
            new_vert.index = new_index;

            EdgeVertex new_edge_vertex;
            new_edge_vertex.index = new_index;
            new_edge_vertex.shared_count = 1;
            new_edge_vertex.edge = edge;
            new_edge_vertex.parent_index1 = std::min(parent_index1, parent_index2);
            new_edge_vertex.parent_index2 = std::max(parent_index1, parent_index2);
            new_edge_vertex.refinement_level = new_vert.level;
            new_edge_vertex.hanging = true; // this is always true if the vertex doesn't exist already
            _edge_vertices.insert({edge, std::move(new_edge_vertex)});
        }
    }

    // if the new vertex is a face vertex
    else if (new_vert.face != RefinedElement::TetFace::NONE)
    {
        Face face;
        if (new_vert.face == RefinedElement::TetFace::F123) face = Face(base_element[0], base_element[1], base_element[2]);
        else if (new_vert.face == RefinedElement::TetFace::F124) face = Face(base_element[0], base_element[1], base_element[3]);
        else if (new_vert.face == RefinedElement::TetFace::F134) face = Face(base_element[0], base_element[2], base_element[3]);
        else if (new_vert.face == RefinedElement::TetFace::F234) face = Face(base_element[1], base_element[2], base_element[3]);

        // look for the face vertex in the global face vertices list
        bool exists = false;
        auto range = _face_vertices.equal_range(face);
        for (auto it = range.first; it != range.second; it++)
        {
            if (it->second.parent_index1 == std::min(parent_index1, parent_index2) && 
                it->second.parent_index2 == std::max(parent_index1, parent_index2))
            {
                // this vertex already exists! The vertex is no longer hanging, since a face vertex can only be shared by max 2 elements
                it->second.hanging = false;

                // set the index in the ChildVertex struct to the index of the existing vertex
                new_vert.index = it->second.index;
                exists = true;
            }
        }

        // the face vertex was not found, so create a new one
        if (!exists)
        {
            int new_index = global_vertices.push_back( (global_vertices.at(parent_index1) + global_vertices.at(parent_index2)) / 2.0 );
            new_vert.index = new_index;

            FaceVertex new_face_vertex;
            new_face_vertex.index = new_index;
            new_face_vertex.face = face;
            new_face_vertex.parent_index1 = std::min(parent_index1, parent_index2);
            new_face_vertex.parent_index2 = std::max(parent_index1, parent_index2);
            new_face_vertex.refinement_level = new_vert.level;
            new_face_vertex.hanging = true; // this is always true if the vertex doesn't exist already
            _face_vertices.insert({face, std::move(new_face_vertex)});
        }
    }

    else
    {
        int new_index = global_vertices.push_back( (global_vertices.at(parent_index1) + global_vertices.at(parent_index2)) / 2.0 );
        new_vert.index = new_index;
        
    }

    // finally, push the new child vertex onto the child_vertices vector
    refined_element->child_vertices.insert({new_vert.index, new_vert});
    refined_element->edge_to_vertex_map.insert({parent_edge, new_vert.index});

    // std::cout << "New child vertex!\n\tIndex: " << new_vert.index << "\n\tLevel: " << new_vert.level << "\n\tEdge: " << static_cast<int>(new_vert.edge) << "\n\tFace: " << static_cast<int>(new_vert.face) << std::endl;

    return new_vert.index;

}

void FEMTetMesh::refineElement(int element_index, int refinement_level)
{
    
    RefinedElement* refined_element = nullptr;

    // check if the element exists
    if (_mesh->elementValid(element_index))
    {
        // if the element is valid, it has not been refined
        // so create a new refined element
        auto [it, success] = _refined_elements.emplace(
            std::make_pair(_mesh->element(element_index), RefinedElement(_mesh->element(element_index), refinement_level))
        );
        assert(success);    // if there was already an element there, something is very wrong
        refined_element = &(it->second);
    }
    else
    {
        // element is not valid...throw an error?
        assert(0);
    }

    const Vec4i& base_element = _mesh->element(element_index);

    // add the base vertices as the first child indices
    for (int i = 0; i < 4; i++)
        refined_element->child_vertices.emplace(
            std::make_pair(base_element[i], RefinedElement::ChildVertex(base_element[i], 0))
        );

    // refine the element

    // recursively keep track of "parent" elements that we want to subdivide
    std::vector<Vec4i> parent_elements = {_mesh->element(element_index)};
    int num_new_tets = 1;   // calculate the number of new tets to be added at each refinement level
    for (int level = 0; level < refinement_level; level++)
    {
        num_new_tets *= 8;

        // create a vector to store the next level of parent elements
        // note that this only applies when we are not at the deepest refinement level
        std::vector<Vec4i> next_parent_elements;
        if (level < refinement_level-1)
            next_parent_elements.reserve(num_new_tets);

        // add newest level of midpoint vertices for each parent element at this level
        std::array<int,6> mid_verts;
        for (const auto& parent_element : parent_elements)
        {
            // std::cout << "\n\n=== Parent Element: " << parent_element.transpose() << std::endl;
            int mid_vert_cnt = 0;
            for (int vi = 0; vi < 4; vi++)
            {
                for (int vj = vi+1; vj < 4; vj++)
                {
                    mid_verts[mid_vert_cnt++] = _addRefinedVertex(parent_element[vi], parent_element[vj], base_element, refined_element);
                }
            }

            // the 4 "corner" new tets
            Vec4i elem1(parent_element[0], mid_verts[0], mid_verts[1], mid_verts[2]);
            Vec4i elem2(parent_element[1], mid_verts[0], mid_verts[3], mid_verts[4]);
            Vec4i elem3(parent_element[2], mid_verts[1], mid_verts[3], mid_verts[5]);
            Vec4i elem4(mid_verts[2], mid_verts[4], mid_verts[5], parent_element[3]);

            // the 4 center tets from the octahedron
            Vec4i elem5(mid_verts[0], mid_verts[1], mid_verts[3], mid_verts[2]);
            Vec4i elem6(mid_verts[0], mid_verts[2], mid_verts[3], mid_verts[4]);
            Vec4i elem7(mid_verts[2], mid_verts[4], mid_verts[5], mid_verts[3]);
            Vec4i elem8(mid_verts[1], mid_verts[5], mid_verts[2], mid_verts[3]);

            if (level == refinement_level-1)
            {
                // we are at the lowest level
                // so add the elements to the child_elements and _temp_elements
                int e1 = _temp_elements.push_back(std::move(elem1));
                int e2 = _temp_elements.push_back(std::move(elem2));
                int e3 = _temp_elements.push_back(std::move(elem3));
                int e4 = _temp_elements.push_back(std::move(elem4));
                int e5 = _temp_elements.push_back(std::move(elem5));
                int e6 = _temp_elements.push_back(std::move(elem6));
                int e7 = _temp_elements.push_back(std::move(elem7));
                int e8 = _temp_elements.push_back(std::move(elem8));
                refined_element->child_elements.insert(refined_element->child_elements.end(), {e1, e2, e3, e4, e5, e6, e7, e8});
            }
            else
            {
                // we are not at the lowest level
                // so add the elements to next_parent_elements for the next iteration
                next_parent_elements.push_back(std::move(elem1));
                next_parent_elements.push_back(std::move(elem2));
                next_parent_elements.push_back(std::move(elem3));
                next_parent_elements.push_back(std::move(elem4));
                next_parent_elements.push_back(std::move(elem5));
                next_parent_elements.push_back(std::move(elem6));
                next_parent_elements.push_back(std::move(elem7));
                next_parent_elements.push_back(std::move(elem8));
            }
        }
        
        parent_elements = std::move(next_parent_elements);
        
    }

    // remove the base element - the refined element takes its place
    _mesh->removeElement(element_index);
}

// void FEMTetMesh::updateRefinement(const Vec4i& base_element, int refinement_level)
// {
//     // if the element is not valid, it maybe has already been refined
//     if (auto search = _refined_elements.find(element_index); search != _refined_elements.end())
//     {
//         refined_element = &(*search->second);
//         current_refinement_level = refined_element->refinement_level;
//     }
//     else
//     {
//         // the element is not valid and it is not in the refined elements list...throw an error?
        
//     }
// }


} // namespace FEM