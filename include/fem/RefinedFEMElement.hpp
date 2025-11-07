#ifndef __REFINED_FEM_ELEMENT_HPP
#define __REFINED_FEM_ELEMENT_HPP

#include "common/types.hpp"

#include <vector>
#include <map>
#include <unordered_map>

namespace FEM
{



class RefinedFEMElement
{
    public:
    struct OuterEdgeVertex
    {
        int parent1, parent2;
        Real interp_factor;
        bool hanging;

        OuterEdgeVertex(int p1, int p2, Real interp, bool hang)
            : parent1(p1), parent2(p2), interp_factor(interp), hanging(hang)
        {}
    };

    RefinedFEMElement(const Vec4i& parent_element,
    const Vec3r& v1, const Vec3r& v2, const Vec3r& v3, const Vec3r& v4, int refinement_level,
    int edge12_adj_refinement, int edge13_adj_refinement, int edge14_adj_refinement, int edge23_adj_refinement, int edge24_adj_refinement, int edge34_adj_refinement);

    const Vec4i& originalParentElement() const { return _parent_element; }

    private:
    void _createChildren();

    Vec4i _parent_element;
    int _refinement_level;

    // the minimum refinement of tets that share the specified edge
    int _edge12_adj_refinement; // i.e. minimum refinement of adjacent tets that share the v1-v2 edge
    int _edge13_adj_refinement; // v1-v3 edge, etc.
    int _edge14_adj_refinement;
    int _edge23_adj_refinement;
    int _edge24_adj_refinement;
    int _edge34_adj_refinement;

    std::vector<Vec3r> _child_vertices;
    std::vector<Vec3i> _child_faces;
    std::vector<Vec4i> _child_elements;

    std::vector<bool> _is_outer_edge_vertex;
    std::unordered_map<int, OuterEdgeVertex> _outer_edge_vertices;

    
};

} // namespace FEM

#endif // __REFINED_FEM_ELEMENT_HPP