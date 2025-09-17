#ifndef __MESH_UTILS_HPP
#define __MESH_UTILS_HPP

#include <gmsh.h>

#include <filesystem>

#include "common/types.hpp"

#include <assimp/Importer.hpp>      // C++ importer interface
#include <assimp/Exporter.hpp>
#include <assimp/scene.h>           // Output data structure
#include <assimp/postprocess.h>     // Post processing flags

#include "geometry/TetMesh.hpp"

#include <set>

namespace MeshUtils
{

Geometry::Mesh loadSurfaceMeshFromFile(const std::string& filename);

void convertToSTL(const std::string& filename);

void convertSTLtoMSH(const std::string& filename);

Geometry::TetMesh loadTetMeshFromGmshFile(const std::string& filename);

void createBeamObj(const std::string& filename, const Real l, const Real w, const Real h,  const int num_subdivisions = 1);

void createBeamMsh(const std::string& filename, Real l, Real w, Real h, int num_subdivisions=1);

void createTissueBlock(const std::string& filename, const Real l, const Real w, const Real h, const int num_low_res_subdivisions, const int high_res_multiplier);

void verticesAndFacesFromFixedFacesFile(const std::string& filename, std::set<int>& vertices, std::vector<int>& faces);

}

#endif // __MESH_UTILS_HPP