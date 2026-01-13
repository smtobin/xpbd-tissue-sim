#include "fem/RefinedFEMElement.hpp"

namespace FEM
{

RefinedFEMElement::RefinedFEMElement(const Vec4i& parent_element,
    const Vec3r& v1, const Vec3r& v2, const Vec3r& v3, const Vec3r& v4, int refinement_level,
    int edge12_adj_refinement, int edge13_adj_refinement, int edge14_adj_refinement, int edge23_adj_refinement, int edge24_adj_refinement, int edge34_adj_refinement)
: _parent_element(parent_element), _refinement_level(refinement_level),
  _edge12_adj_refinement(edge12_adj_refinement), _edge13_adj_refinement(edge13_adj_refinement), _edge14_adj_refinement(edge14_adj_refinement),
  _edge23_adj_refinement(edge23_adj_refinement), _edge24_adj_refinement(edge24_adj_refinement),
  _edge34_adj_refinement(edge34_adj_refinement)
{
    _child_vertices.push_back(v1);
    _child_vertices.push_back(v2);
    _child_vertices.push_back(v3);
    _child_vertices.push_back(v4);
    _createChildren();
}

void RefinedFEMElement::_addVertex(int parent_ind1, int parent_ind2)
{
    // first - determine if this new vertex is on one of the outer (i.e. original) edges of the tetrahedron
    // for this to be true:
    //   - both parent vertices must be on outer surface
    //   - if both parent vertices are on edges, they must be on the same edge
    bool on_outer_surface = _is_outer_surface_vertex[parent_ind1] && _is_outer_surface_vertex[parent_ind2]
    _is_outer_edge_vertex.push_back(on_outer_edge);
    _child_vertices.emplace_back((_child_vertices[parent_ind1] + _child_vertices[parent_ind2])/2.0);

    int new_ind = _child_vertices.size() - 1;
    if (on_outer_edge)
    {
        // get the two parent OuterEdgeVertex's
        const OuterEdgeVertex& oe_vert1 = _outer_edge_vertices[parent_ind1];
        const OuterEdgeVertex& oe_vert2 = _outer_edge_vertices[parent_ind2];
        _outer_edge_vertices[new_ind] = OuterEdgeVertex(std::min(base_parent1, base_parent2), std::max(base_parent1, base_parent2), )
    }
}

void RefinedFEMElement::_createChildren()
{
    // recursively keep track of "parent" elements that we want to subdivide
    std::vector<Vec4i> parent_elements = {Vec4i(0, 1, 2, 3)};

    int num_new_tets = 1;
    for (int i = 0; i < _refinement_level; i++)
    {
        // calculate the number of new tets to be added at this refinement level
        num_new_tets *= 8;

        // create a vector to store the next level of parent elements
        // note that this only applies when we are not at the deepest refinement level
        std::vector<Vec4i> next_parent_elements;
        if (i < _refinement_level-1)
            next_parent_elements.reserve(num_new_tets);

        // add newest level of midpoint vertices for each parent element at this level
        for (const auto& parent_element : parent_elements)
        {
            const Vec3r& v1 = _child_vertices[parent_element[0]];
            const Vec3r& v2 = _child_vertices[parent_element[1]];
            const Vec3r& v3 = _child_vertices[parent_element[2]];
            const Vec3r& v4 = _child_vertices[parent_element[3]];
            _child_vertices.emplace_back((v1+v2)/2);
            _child_vertices.emplace_back((v2+v3)/2);
            _child_vertices.emplace_back((v3+v1)/2);
            _child_vertices.emplace_back((v1+v4)/2);
            _child_vertices.emplace_back((v2+v4)/2);
            _child_vertices.emplace_back((v3+v4)/2);

            // check for outer edge vertices
            if (_is_outer_edge_vertex[parent_element[0]] && _is_outer_edge_vertex[parent_element[1]])
            {
                _is_outer_edge_vertex.push_back(true);
            }

            // the 4 "corner" new tets
            Vec4i elem1(parent_element[0], _child_vertices.size()-6, _child_vertices.size()-4, _child_vertices.size()-3);
            Vec4i elem2(parent_element[1], _child_vertices.size()-5, _child_vertices.size()-6, _child_vertices.size()-2);
            Vec4i elem3(parent_element[2], _child_vertices.size()-4, _child_vertices.size()-5, _child_vertices.size()-1);
            Vec4i elem4(_child_vertices.size()-3, _child_vertices.size()-2, _child_vertices.size()-3, parent_element[3]);

            // the 4 center tets from the octahedron
            Vec4i elem5(_child_vertices.size()-5, _child_vertices.size()-2, _child_vertices.size()-1, _child_vertices.size()-3);
            Vec4i elem6(_child_vertices.size()-4, _child_vertices.size()-1, _child_vertices.size()-3, _child_vertices.size()-5);
            Vec4i elem7(_child_vertices.size()-6, _child_vertices.size()-2, _child_vertices.size()-5, _child_vertices.size()-3);
            Vec4i elem8(_child_vertices.size()-6, _child_vertices.size()-5, _child_vertices.size()-4, _child_vertices.size()-3);

            if (i == _refinement_level-1)
            {
                // we are at the lowest level
                // so add the elements to the _child_elements
                _child_elements.push_back(std::move(elem1));
                _child_elements.push_back(std::move(elem2));
                _child_elements.push_back(std::move(elem3));
                _child_elements.push_back(std::move(elem4));
                _child_elements.push_back(std::move(elem5));
                _child_elements.push_back(std::move(elem6));
                _child_elements.push_back(std::move(elem7));
                _child_elements.push_back(std::move(elem8));
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
}

} // namespace FEM