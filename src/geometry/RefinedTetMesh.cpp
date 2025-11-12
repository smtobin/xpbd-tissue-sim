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

/** Refinement */

int RefinedTetMesh::_addRefinedVertex(int parent_index1, int parent_index2,
     const Vec4i& base_element, RefinedElement& refined_element)
{
    Edge parent_edge(parent_index1, parent_index2);
    
    const typename RefinedElement::ChildVertex& p1 = refined_element.child_vertices.at(parent_index1);
    const typename RefinedElement::ChildVertex& p2 = refined_element.child_vertices.at(parent_index2);
    bool p1_is_edge = p1.edge != TetEdge::NONE;
    bool p2_is_edge = p2.edge != TetEdge::NONE;
    bool p1_is_face = p1.face != TetFace::NONE;
    bool p2_is_face = p2.face != TetFace::NONE;
    
    int new_level = std::max(p1.level, p2.level) + 1;
    typename RefinedElement::ChildVertex new_vert(-1, new_level, TetEdge::NONE, TetFace::NONE);

    // check if the parent edge already has a vertex at the midpoint
    if (auto search = _parent_edge_to_child_vertex_map.find(parent_edge); search != _parent_edge_to_child_vertex_map.end())
    {
        // the vertex already exists!
        new_vert.index = search->second;
        refined_element.child_vertices.insert_or_assign(search->second, std::move(new_vert));   // this will add this new vertex to the child vertices, if there is not already an entry
        
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
    new_vert.index = new_index;
    refined_element.child_vertices.insert({new_index, std::move(new_vert)});

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
    
    /** === Step 1: Create a RefinedElement struct for the element that we are refining (if the element exists). === */

    RefinedElement* refined_element = nullptr;

    // check if the element exists
    if (elementValid(element_index))
    {
        // if the element is valid, it has not been refined
        // so create a new refined element
        auto [it, success] = _refined_elements.emplace(
            std::make_pair(element(element_index), RefinedElement(element(element_index), refinement_level))
        );
        assert(success);    // if there was already an element there, something is very wrong
        refined_element = &(it->second);
    }
    else
    {
        // element is not valid...throw an error?
        assert(0);
    }


    /** === Step 2: Prepare for the refinement algorithm. === */

    const Vec4i base_element = element(element_index);  // don't use a ref here since the element will soon be removed

    // add the base vertices as the first child vertices
    for (int i = 0; i < 4; i++)
        refined_element->child_vertices.emplace(
            std::make_pair(base_element[i], RefinedElement::ChildVertex(base_element[i], 0))
        );

    // this struct represents a "parent" element that we will recursively subdivide
    // we will create these as intermediate steps as we hierarchically refine
    // the reason for this struct existing is to track what faces of the parent element are on the surface of the mesh
    // that way, we know what faces of the child elements are on the surface of the mesh
    struct ParentElement
    {
        Vec4i element;
        bool f123_on_surface = false;
        bool f124_on_surface = false;
        bool f134_on_surface = false;
        bool f234_on_surface = false;

        ParentElement(const Vec4i& element_)
            : element(element_)
        {}

        ParentElement(const Vec4i& element_, bool f123, bool f124, bool f134, bool f234)
            : element(element_), f123_on_surface(f123), f124_on_surface(f124), f134_on_surface(f134), f234_on_surface(f234)
        {}
    };

    // create the initial ParentElement struct for the base element that we are subdividing
    ParentElement base_parent_element(element(element_index));
        
    // find which faces of the base element (if any) are on the outer surface of the mesh
    auto surface_faces_range = _element_to_surface_faces_map.equal_range(element_index);
    for (auto it = surface_faces_range.first; it != surface_faces_range.second; it++)
    {
        const Vec3i& face_vec = face(it->second);
        const Vec4i& elem = base_element;
        Face surface_face(face_vec[0], face_vec[1], face_vec[2]);
        
        if (Face(elem[0], elem[1], elem[2]) == surface_face)        base_parent_element.f123_on_surface = true;
        else if (Face(elem[0], elem[1], elem[3]) == surface_face)   base_parent_element.f124_on_surface = true;
        else if (Face(elem[0], elem[2], elem[3]) == surface_face)   base_parent_element.f134_on_surface = true;
        else if (Face(elem[1], elem[2], elem[3]) == surface_face)   base_parent_element.f234_on_surface = true;
    }
    
    // recursively keep track of "parent" elements that we want to subdivide
    std::vector<ParentElement> parent_elements = {base_parent_element};
    int num_new_tets = 1;   // calculate the number of new tets to be added at each refinement level



    /** === Step 3: Remove the element from the mesh === */

    // first remove surface faces associated with the element
    for (auto it = surface_faces_range.first; it != surface_faces_range.second; it++)
    {
        _faces.erase(it->second);

        // note: we do not need to update the surface face -> element map since that will just be overwritten by whatever new faces are added
    }

    // update the vertex -> element, edge -> element, face -> element maps
    _updateElementMapsForRemovedElement(element_index);

    // remove the element
    _elements.erase(element_index);



    /** === Step 4: Refine the element. === */

    
    for (int level = 0; level < refinement_level; level++)
    {
        num_new_tets *= 8;

        // create a vector to store the next level of parent elements
        // note that this only applies when we are not at the deepest refinement level
        std::vector<ParentElement> next_parent_elements;
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
                    mid_verts[mid_vert_cnt++] = _addRefinedVertex(parent_element.element[vi], parent_element.element[vj], base_element, *refined_element);
                }
            }

            // the 4 "corner" new tets
            Vec4i elem1(parent_element.element[0], mid_verts[0], mid_verts[1], mid_verts[2]);   // (0, 4, 5, 6)
            Vec4i elem2(parent_element.element[1], mid_verts[0], mid_verts[3], mid_verts[4]);   // (1, 4, 7, 8)
            Vec4i elem3(parent_element.element[2], mid_verts[1], mid_verts[3], mid_verts[5]);   // (2, 5, 7, 9)
            Vec4i elem4(mid_verts[2], mid_verts[4], mid_verts[5], parent_element.element[3]);   // (6, 8, 9, 3)

            // the 4 center tets from the octahedron
            Vec4i elem5(mid_verts[0], mid_verts[1], mid_verts[3], mid_verts[2]);    // (4, 5, 7, 6)
            Vec4i elem6(mid_verts[0], mid_verts[2], mid_verts[3], mid_verts[4]);    // (4, 6, 7, 8)
            Vec4i elem7(mid_verts[2], mid_verts[4], mid_verts[5], mid_verts[3]);    // (6, 8, 9, 7)
            Vec4i elem8(mid_verts[1], mid_verts[5], mid_verts[2], mid_verts[3]);    // (5, 9, 6, 7)

            if (level == refinement_level-1)
            {
                // we are at the lowest level
                // so add the elements to the child_elements and _temp_elements
                int e1 = _addNewElement(elem1, parent_element.f123_on_surface, parent_element.f124_on_surface, parent_element.f134_on_surface, false);
                int e2 = _addNewElement(elem2, parent_element.f123_on_surface, parent_element.f124_on_surface, parent_element.f234_on_surface, false);
                int e3 = _addNewElement(elem3, parent_element.f123_on_surface, parent_element.f134_on_surface, parent_element.f234_on_surface, false);
                int e4 = _addNewElement(elem4, false, parent_element.f124_on_surface, parent_element.f134_on_surface, parent_element.f234_on_surface);
                int e5 = _addNewElement(elem5, parent_element.f123_on_surface, false, false, false);
                int e6 = _addNewElement(elem6, false, parent_element.f124_on_surface, false, false);
                int e7 = _addNewElement(elem7, false, false, false, parent_element.f234_on_surface);
                int e8 = _addNewElement(elem8, parent_element.f134_on_surface, false, false, false);

                refined_element->child_elements.insert(refined_element->child_elements.end(), {e1, e2, e3, e4, e5, e6, e7, e8});
            }
            else
            {
                // we are not at the lowest level
                // so add the elements to next_parent_elements for the next iteration
                // the next set of parent elements may have some faces on the outer surface of the mesh, depending on the current parent element

                // elem1 is (0, 4, 5, 6)
                next_parent_elements.emplace_back(elem1, parent_element.f123_on_surface, parent_element.f124_on_surface, parent_element.f134_on_surface, false);
                // elem2 is (1, 4, 7, 8)
                next_parent_elements.emplace_back(elem2, parent_element.f123_on_surface, parent_element.f124_on_surface, parent_element.f234_on_surface, false);
                // elem3 is (2, 5, 7, 9)
                next_parent_elements.emplace_back(elem3, parent_element.f123_on_surface, parent_element.f134_on_surface, parent_element.f234_on_surface, false);
                // elem4 is (6, 8, 9, 3)
                next_parent_elements.emplace_back(elem4, false, parent_element.f124_on_surface, parent_element.f134_on_surface, parent_element.f234_on_surface);
                // elem5 is (4, 5, 7, 6)
                next_parent_elements.emplace_back(elem5, parent_element.f123_on_surface, false, false, false);
                // elem6 is (4, 6, 7, 8)
                next_parent_elements.emplace_back(elem6, false, parent_element.f124_on_surface, false, false);
                // elem7 is (6, 8, 9, 7)
                next_parent_elements.emplace_back(elem7, false, false, false, parent_element.f234_on_surface);
                // elem8 is (5, 9, 6, 7)
                next_parent_elements.emplace_back(elem8, parent_element.f134_on_surface, false, false, false);
            }
        }
        
        parent_elements = std::move(next_parent_elements);
        
    }

}

} // namespace Geometry