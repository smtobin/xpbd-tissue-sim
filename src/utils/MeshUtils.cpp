#include "utils/MeshUtils.hpp"

#include <iostream>
#include <fstream>
#include <sstream>

Geometry::Mesh MeshUtils::loadSurfaceMeshFromFile(const std::string& filename)
{
    Assimp::Importer importer;

    // And have it read the given file with some example postprocessing
    // Usually - if speed is not the most important aspect for you - you'll
    // probably to request more postprocessing than we do in this example.
    const aiScene* scene = importer.ReadFile( filename,
        aiProcess_Triangulate            |
        aiProcess_JoinIdenticalVertices  |
        aiProcess_SortByPType);

    // If the import failed, report it
    if (scene == nullptr)
    {
        std::cerr << "\tAssimp::Importer could not open " << filename << std::endl;
        std::cerr << "\tEnsure that the file is in a format that assimp can handle." << std::endl;
        assert(0);
    }

    const aiMesh* ai_mesh = scene->mMeshes[0];

    // Extract vertices
    std::vector<Vec3r> verts(ai_mesh->mNumVertices);
    for (unsigned i = 0; i < ai_mesh->mNumVertices; i++)
    {
        verts[i][0] = ai_mesh->mVertices[i].x;
        verts[i][1] = ai_mesh->mVertices[i].y;
        verts[i][2] = ai_mesh->mVertices[i].z;
    }

    // Extract faces
    std::vector<Vec3i> faces(ai_mesh->mNumFaces);
    for (unsigned i = 0; i < ai_mesh->mNumFaces; i++)
    {
        faces[i][0] = ai_mesh->mFaces[i].mIndices[0];
        faces[i][1] = ai_mesh->mFaces[i].mIndices[1];
        faces[i][2] = ai_mesh->mFaces[i].mIndices[2];
    }

    Geometry::Mesh mesh(verts, faces);
    return mesh;
}

Geometry::TetMesh MeshUtils::loadTetMeshFromGmshFile(const std::string& filename)
{
    std::cout << "MeshUtils::loadTetMeshFromGmshFile - loading mesh data from " << filename << " as a Geometry::Mesh..." << std::endl;

    // ensure the file exists
    if (!std::filesystem::exists(filename))
    {
        std::cerr << "\t" << filename << " does not exist!" << std::endl;
        assert(0);
    }


    std::filesystem::path file_path(filename);

    if (file_path.extension() == ".obj" || file_path.extension() == ".stl")
    {
        convertToSTL(filename);
        convertSTLtoMSH(file_path.replace_extension(".stl"));
    }

    file_path = file_path.replace_extension(".msh");

    // ensure the file is a .msh file
    if (file_path.extension() != ".msh" && file_path.extension() != ".MSH")
    {
        std::cerr << "\t" << filename << " is not a .msh file!" << std::endl;
        assert(0);
    }

    gmsh::open(file_path.string());

    // Get all the elementary entities in the model, as a vector of (dimension,
    // tag) pairs:
    std::vector<std::pair<int, int> > entities;
    gmsh::model::getEntities(entities);

    std::vector<Vec3r> vertices;
    std::vector<Vec3i> faces;
    std::vector<Vec4i> elements;

    for(auto e : entities) {
        // Dimension and tag of the entity:
        int dim = e.first, tag = e.second;

        // Mesh data is made of `elements' (points, lines, triangles, ...), defined
        // by an ordered list of their `nodes'. Elements and nodes are identified by
        // `tags' as well (strictly positive identification numbers), and are stored
        // ("classified") in the model entity they discretize. Tags for elements and
        // nodes are globally unique (and not only per dimension, like entities).

        // A model entity of dimension 0 (a geometrical point) will contain a mesh
        // element of type point, as well as a mesh node. A model curve will contain
        // line elements as well as its interior nodes, while its boundary nodes
        // will be stored in the bounding model points. A model surface will contain
        // triangular and/or quadrangular elements and all the nodes not classified
        // on its boundary or on its embedded entities. A model volume will contain
        // tetrahedra, hexahedra, etc. and all the nodes not classified on its
        // boundary or on its embedded entities.

        // Get the mesh nodes for the entity (dim, tag):
        std::vector<std::size_t> nodeTags;
        std::vector<double> nodeCoords, nodeParams;
        gmsh::model::mesh::getNodes(nodeTags, nodeCoords, nodeParams, dim, tag);

        unsigned vert_offset = vertices.size();
        vertices.resize(vert_offset + nodeTags.size());
        for (unsigned i = 0; i < nodeTags.size(); i++)
        {
            vertices[vert_offset + i][0] = static_cast<Real>(nodeCoords[i*3]);
            vertices[vert_offset + i][1] = static_cast<Real>(nodeCoords[i*3 + 1]);
            vertices[vert_offset + i][2] = static_cast<Real>(nodeCoords[i*3 + 2]);
        }

        // Get the mesh elements for the entity (dim, tag):
        std::vector<int> elemTypes;
        std::vector<std::vector<std::size_t> > elemTags, elemNodeTags;
        gmsh::model::mesh::getElements(elemTypes, elemTags, elemNodeTags, dim, tag);

        // Extract all tetrahedra into a flat vector
        std::vector<unsigned> tetrahedra_vertex_indices;
        // Extract all surface triangles into a flat vector
        std::vector<unsigned> triangle_vertex_indices;
        for (unsigned i = 0; i < elemTypes.size(); i++)
        {
            if (elemTypes[i] == 4)
            {
                tetrahedra_vertex_indices.insert(tetrahedra_vertex_indices.end(), elemNodeTags[i].begin(), elemNodeTags[i].end());
            }

            if (elemTypes[i] == 2)
            {
                triangle_vertex_indices.insert(triangle_vertex_indices.end(), elemNodeTags[i].begin(), elemNodeTags[i].end());
            }
        }

        unsigned elem_offset = elements.size();
        unsigned num_tetrahedra = tetrahedra_vertex_indices.size()/4;
        elements.resize(elem_offset + num_tetrahedra);
        for (unsigned i = 0; i < num_tetrahedra; i++)
        {
            elements[elem_offset + i][0] = tetrahedra_vertex_indices[i*4] - 1;
            elements[elem_offset + i][1] = tetrahedra_vertex_indices[i*4 + 1] - 1;
            elements[elem_offset + i][2] = tetrahedra_vertex_indices[i*4 + 2] - 1;
            elements[elem_offset + i][3] = tetrahedra_vertex_indices[i*4 + 3] - 1;
        }

        unsigned face_offset = faces.size();
        unsigned num_faces = triangle_vertex_indices.size()/3;
        faces.resize(face_offset + num_faces);
        for (unsigned i = 0; i < num_faces; i++)
        {
            faces[face_offset + i][0] = triangle_vertex_indices[i*3] - 1;
            faces[face_offset + i][1] = triangle_vertex_indices[i*3 + 1] - 1;
            faces[face_offset + i][2] = triangle_vertex_indices[i*3 + 2] - 1;
        }
    }

    Geometry::TetMesh tet_mesh(vertices, faces, elements);

    // write the loaded surface mesh part to file
    const std::string surface_mesh_filename = filename.substr(0,filename.length()-4) + "_surface_mesh.obj";
    tet_mesh.writeMeshToObjFile(surface_mesh_filename);

    return tet_mesh;

}



void MeshUtils::createBeamObj(const std::string& filename, const Real length, const Real width, const Real height, const int num_subdivisions)
{
    // try and discretize so that the elements are roughly cube
    const Real elem_size = std::min(width, height) / num_subdivisions;
    const int w = static_cast<int>(width/elem_size);
    const int h = static_cast<int>(height/elem_size);
    const int l = static_cast<int>(length/elem_size);
    std::cout << "size in elements: " << h << "x" << w << "x" << l << std::endl;
    
    auto index = [w, h] (int li, int wi, int hi)
    {
        return hi + wi*(h+1) + li*(w+1)*(h+1);
    };

    Eigen::Matrix<Real, -1, 3> verts((h+1)*(w+1)*(l+1), 3);
    for (int li = 0; li < l+1; li++) 
    {
        for (int wi = 0; wi < w+1; wi++)
        {
            for (int hi = 0; hi < h+1; hi++)
            {
                int ind = index(li, wi, hi);
                Vec3r vert({wi*elem_size, li*elem_size, hi*elem_size});
                verts.row(ind) = vert;
            }
        }
    }

    Eigen::Matrix<int, -1, 3> faces(4*w*h + 4*l*h + 4*l*w, 3);
    int face_ind = 0;
    // w*h faces
    for (int wi = 0; wi < w; wi++)
    {
        for (int hi = 0; hi < h; hi++)
        {
            int ff1 = index(0, wi, hi);
            int ff2 = index(0, wi+1, hi);
            int ff3 = index(0, wi+1, hi+1);
            int ff4 = index(0, wi, hi+1);

            int a = wi%2 - hi%2;//rand()%2;
            if (a)
            {
                Eigen::Vector3i front_face1({ff1, ff2, ff3});
                Eigen::Vector3i front_face2({ff1, ff3, ff4});
                faces.row(face_ind) = front_face1;
                faces.row(face_ind+1) = front_face2;
            }
            else
            {
                Eigen::Vector3i front_face1({ff1, ff2, ff4});
                Eigen::Vector3i front_face2({ff2, ff3, ff4});
                faces.row(face_ind) = front_face1;
                faces.row(face_ind+1) = front_face2;
            }
            

            int bf1 = index(l, wi, hi);
            int bf2 = index(l, wi+1, hi);
            int bf3 = index(l, wi+1, hi+1);
            int bf4 = index(l, wi, hi+1);

            // a = rand()%2;
            if (a)
            {
                Eigen::Vector3i back_face1({bf1, bf3, bf2});
                Eigen::Vector3i back_face2({bf1, bf4, bf3});
                faces.row(face_ind+2) = back_face1;
                faces.row(face_ind+3) = back_face2;
            }
            else
            {
                Eigen::Vector3i back_face1({bf1, bf4, bf2});
                Eigen::Vector3i back_face2({bf2, bf4, bf3});
                faces.row(face_ind+2) = back_face1;
                faces.row(face_ind+3) = back_face2;
            }
            

            face_ind += 4;
        }
    }

    // l*h faces
    for (int hi = 0; hi < h; hi++)
    {
        for (int li = 0; li < l; li++)
        {
            int rf1 = index(li, w, hi);
            int rf2 = index(li+1, w, hi);
            int rf3 = index(li+1, w, hi+1);
            int rf4 = index(li, w, hi+1);
            
            int a = hi%2 - li%2;//rand()%2;
            if (a)
            {
                Eigen::Vector3i right_face1({rf1, rf2, rf3});
                Eigen::Vector3i right_face2({rf1, rf3, rf4});
                faces.row(face_ind) = right_face1;
                faces.row(face_ind+1) = right_face2;
            }
            else
            {
                Eigen::Vector3i right_face1({rf1, rf2, rf4});
                Eigen::Vector3i right_face2({rf2, rf3, rf4});
                faces.row(face_ind) = right_face1;
                faces.row(face_ind+1) = right_face2;
            }
            

            int lf1 = index(li, 0, hi);
            int lf2 = index(li+1, 0, hi);
            int lf3 = index(li+1, 0, hi+1);
            int lf4 = index(li, 0, hi+1);
            // a = rand()%2;
            if (a)
            {
                Eigen::Vector3i left_face1({lf1, lf3, lf2});
                Eigen::Vector3i left_face2({lf1, lf4, lf3});
                faces.row(face_ind+2) = left_face1;
                faces.row(face_ind+3) = left_face2;
            }
            else
            {
                Eigen::Vector3i left_face1({lf1, lf4, lf2});
                Eigen::Vector3i left_face2({lf2, lf4, lf3});
                faces.row(face_ind+2) = left_face1;
                faces.row(face_ind+3) = left_face2;
            }
            

            face_ind += 4;
        }
    }

    // l*w faces
    for (int li = 0; li < l; li++)
    {
        for (int wi = 0; wi < w; wi++)
        {
            int tf1 = index(li, wi, h);
            int tf2 = index(li, wi+1, h);
            int tf3 = index(li+1, wi+1, h);
            int tf4 = index(li+1, wi, h);
            int a = li%2-wi%2; //rand()%2;
            if (a)
            {
                Eigen::Vector3i top_face1({tf1, tf2, tf3});
                Eigen::Vector3i top_face2({tf1, tf3, tf4});
                faces.row(face_ind) = top_face1;
                faces.row(face_ind+1) = top_face2;
            }
            else
            {
                Eigen::Vector3i top_face1({tf1, tf2, tf4});
                Eigen::Vector3i top_face2({tf2, tf3, tf4});
                faces.row(face_ind) = top_face1;
                faces.row(face_ind+1) = top_face2;
            }
            

            int bf1 = index(li, wi, 0);
            int bf2 = index(li, wi+1, 0);
            int bf3 = index(li+1, wi+1, 0);
            int bf4 = index(li+1, wi, 0);
            // a = rand()%2;
            if (a)
            {
                Eigen::Vector3i bottom_face1({bf1, bf3, bf2});
                Eigen::Vector3i bottom_face2({bf1, bf4, bf3});
                faces.row(face_ind+2) = bottom_face1;
                faces.row(face_ind+3) = bottom_face2;
            }
            else
            {
                Eigen::Vector3i bottom_face1({bf1, bf4, bf2});
                Eigen::Vector3i bottom_face2({bf2, bf4, bf3});
                faces.row(face_ind+2) = bottom_face1;
                faces.row(face_ind+3) = bottom_face2;
            }
            

            face_ind += 4;
        }
    }

    std::ofstream ss(filename);
    for (int i = 0; i < verts.rows(); i++)
    {
        ss << "v " << verts(i,0) << " " << verts(i,1) << " " << verts(i,2) << "\n";
    }

    for (int i = 0; i < faces.rows(); i++)
    {
        ss << "f " << faces(i,0)+1 << " " << faces(i,1)+1 << " " << faces(i,2)+1 << "\n";
    }

    ss.close();



}

void MeshUtils::createBeamMsh(const std::string& filename, Real length, Real width, Real height, int num_subdivisions)
{
    // try and discretize so that the elements are roughly cube
    const Real elem_size = std::min(width, height) / num_subdivisions;
    const int w = static_cast<int>(width/elem_size);
    const int h = static_cast<int>(height/elem_size);
    const int l = static_cast<int>(length/elem_size);
    std::cout << "size in elements: " << h << "x" << w << "x" << l << std::endl;
    
    auto index = [w, h] (int li, int wi, int hi)
    {
        return hi + wi*(h+1) + li*(w+1)*(h+1);
    };

    Eigen::Matrix<Real, -1, 3> verts((h+1)*(w+1)*(l+1), 3);
    for (int li = 0; li < l+1; li++) 
    {
        for (int wi = 0; wi < w+1; wi++)
        {
            for (int hi = 0; hi < h+1; hi++)
            {
                int ind = index(li, wi, hi);
                Vec3r vert({wi*elem_size, li*elem_size, hi*elem_size});
                verts.row(ind) = vert;
            }
        }
    }

    Eigen::Matrix<int, -1, 3> faces(4*w*h + 4*l*h + 4*l*w, 3);
    int face_ind = 0;
    // w*h faces
    for (int wi = 0; wi < w; wi++)
    {
        for (int hi = 0; hi < h; hi++)
        {
            int ff1 = index(0, wi, hi);
            int ff2 = index(0, wi+1, hi);
            int ff3 = index(0, wi+1, hi+1);
            int ff4 = index(0, wi, hi+1);

            int a = (wi%2 == hi%2);//rand()%2;
            if (!a)
            {
                Eigen::Vector3i front_face1({ff1, ff2, ff4});
                Eigen::Vector3i front_face2({ff2, ff3, ff4});
                faces.row(face_ind) = front_face1;
                faces.row(face_ind+1) = front_face2;
            }
            else
            {
                Eigen::Vector3i front_face1({ff1, ff2, ff3});
                Eigen::Vector3i front_face2({ff1, ff3, ff4});
                faces.row(face_ind) = front_face1;
                faces.row(face_ind+1) = front_face2;
            }
            

            int bf1 = index(l, wi, hi);
            int bf2 = index(l, wi+1, hi);
            int bf3 = index(l, wi+1, hi+1);
            int bf4 = index(l, wi, hi+1);

            // a = rand()%2;
            a = (wi%2 == hi%2) - l%2;
            if (!a)
            {
                Eigen::Vector3i back_face1({bf1, bf4, bf2});
                Eigen::Vector3i back_face2({bf2, bf4, bf3});
                faces.row(face_ind+2) = back_face1;
                faces.row(face_ind+3) = back_face2;
            }
            else
            {
                Eigen::Vector3i back_face1({bf1, bf3, bf2});
                Eigen::Vector3i back_face2({bf1, bf4, bf3});
                faces.row(face_ind+2) = back_face1;
                faces.row(face_ind+3) = back_face2;
            }
            

            face_ind += 4;
        }
    }

    // l*h faces
    for (int hi = 0; hi < h; hi++)
    {
        for (int li = 0; li < l; li++)
        {
            int rf1 = index(li, w, hi);
            int rf2 = index(li+1, w, hi);
            int rf3 = index(li+1, w, hi+1);
            int rf4 = index(li, w, hi+1);
            
            int a = (hi%2 == li%2) - w%2;//rand()%2;
            if (a)
            {
                Eigen::Vector3i right_face1({rf1, rf2, rf3});
                Eigen::Vector3i right_face2({rf1, rf3, rf4});
                faces.row(face_ind) = right_face1;
                faces.row(face_ind+1) = right_face2;
            }
            else
            {
                Eigen::Vector3i right_face1({rf1, rf2, rf4});
                Eigen::Vector3i right_face2({rf2, rf3, rf4});
                faces.row(face_ind) = right_face1;
                faces.row(face_ind+1) = right_face2;
            }
            

            int lf1 = index(li, 0, hi);
            int lf2 = index(li+1, 0, hi);
            int lf3 = index(li+1, 0, hi+1);
            int lf4 = index(li, 0, hi+1);
            // a = rand()%2;
            a = (hi%2 == li%2);
            if (!a)
            {
                Eigen::Vector3i left_face1({lf1, lf4, lf2});
                Eigen::Vector3i left_face2({lf2, lf4, lf3});
                faces.row(face_ind+2) = left_face1;
                faces.row(face_ind+3) = left_face2;
            }
            else
            {
                Eigen::Vector3i left_face1({lf1, lf3, lf2});
                Eigen::Vector3i left_face2({lf1, lf4, lf3});
                faces.row(face_ind+2) = left_face1;
                faces.row(face_ind+3) = left_face2;
            }
            

            face_ind += 4;
        }
    }

    // l*w faces
    for (int li = 0; li < l; li++)
    {
        for (int wi = 0; wi < w; wi++)
        {
            int tf1 = index(li, wi, h);
            int tf2 = index(li, wi+1, h);
            int tf3 = index(li+1, wi+1, h);
            int tf4 = index(li+1, wi, h);
            int a = (li%2 == wi%2) - h%2; //rand()%2;
            if (a)
            {
                Eigen::Vector3i top_face1({tf1, tf2, tf3});
                Eigen::Vector3i top_face2({tf1, tf3, tf4});
                faces.row(face_ind) = top_face1;
                faces.row(face_ind+1) = top_face2;
            }
            else
            {
                Eigen::Vector3i top_face1({tf1, tf2, tf4});
                Eigen::Vector3i top_face2({tf2, tf3, tf4});
                faces.row(face_ind) = top_face1;
                faces.row(face_ind+1) = top_face2;
            }
            

            int bf1 = index(li, wi, 0);
            int bf2 = index(li, wi+1, 0);
            int bf3 = index(li+1, wi+1, 0);
            int bf4 = index(li+1, wi, 0);
            // a = rand()%2;
            a = (li%2 == wi%2);
            if (a)
            {
                Eigen::Vector3i bottom_face1({bf1, bf3, bf2});
                Eigen::Vector3i bottom_face2({bf1, bf4, bf3});
                faces.row(face_ind+2) = bottom_face1;
                faces.row(face_ind+3) = bottom_face2;
            }
            else
            {
                Eigen::Vector3i bottom_face1({bf1, bf4, bf2});
                Eigen::Vector3i bottom_face2({bf2, bf4, bf3});
                faces.row(face_ind+2) = bottom_face1;
                faces.row(face_ind+3) = bottom_face2;
            }
            

            face_ind += 4;
        }
    }

    // elements
    Eigen::Matrix<int, -1, 4> elements(5*l*w*h, 4);
    // iterate through each hexahedron
    int element_ind = 0;
    for (int li = 0; li < l; li++)
    {
        for (int wi = 0; wi < w; wi++)
        {
            for (int hi = 0; hi < h; hi++)
            {
                int a = (hi%2 == wi%2) - li%2;
                std::cout << "(hi, wi, li): " << hi << ", " << wi << ", " << li << "   a: " << a << std::endl;

                int i1 = index(li, wi, hi);
                int i2 = index(li, wi+1, hi);
                int i3 = index(li+1, wi+1, hi);
                int i4 = index(li+1, wi, hi);
                int i5 = index(li, wi, hi+1);
                int i6 = index(li, wi+1, hi+1);
                int i7 = index(li+1, wi+1, hi+1);
                int i8 = index(li+1, wi, hi+1);

                // TODO: do the index orderings in the tet matter?
                if (a)
                {
                    Eigen::Vector4i tet1(i1, i2, i3, i6);
                    Eigen::Vector4i tet2(i1, i3, i4, i8);
                    Eigen::Vector4i tet3(i1, i6, i5, i8);
                    Eigen::Vector4i tet4(i3, i6, i7, i8);
                    Eigen::Vector4i tet5(i1, i3, i6, i8);
                    elements.row(element_ind++) = tet1;
                    elements.row(element_ind++) = tet2;
                    elements.row(element_ind++) = tet3;
                    elements.row(element_ind++) = tet4;
                    elements.row(element_ind++) = tet5;
                }
                else
                {
                    Eigen::Vector4i tet1(i1, i4, i2, i5);
                    Eigen::Vector4i tet2(i2, i4, i3, i7);
                    Eigen::Vector4i tet3(i2, i6, i5, i7);
                    Eigen::Vector4i tet4(i4, i5, i8, i7);
                    Eigen::Vector4i tet5(i2, i4, i5, i7);
                    elements.row(element_ind++) = tet1;
                    elements.row(element_ind++) = tet2;
                    elements.row(element_ind++) = tet3;
                    elements.row(element_ind++) = tet4;
                    elements.row(element_ind++) = tet5;
                }
            }
        }
    }

    std::ofstream ss(filename);
    ss << "$MeshFormat\n4.1 0 8\n$EndMeshFormat\n$Entities\n";
    ss << "0 0 0 1\n";
    ss << "1 0 0 0 " << (w+1)*elem_size << " " << (l+1)*elem_size << " " << (h+1)*elem_size << " 0 0 0\n";
    ss << "$EndEntities\n$Nodes\n";
    ss << "1 " << verts.rows() << " 1 " << verts.rows() << "\n";
    ss << "2 1 0 " << verts.rows() << "\n";
    for (int i = 0; i < verts.rows(); i++)
        ss << i+1 << "\n";
    for (int i = 0; i < verts.rows(); i++)
        ss << verts(i,0) << " " << verts(i,1) << " " << verts(i,2) << "\n";
    ss << "$EndNodes\n$Elements\n";
    ss << "2 " << faces.rows() + elements.rows() << " 1 " << faces.rows() + elements.rows() << "\n";
    ss << "2 1 2 " << faces.rows() << "\n";
    for (int i = 0; i < faces.rows(); i++)
        ss << i+1 << " " << faces(i,0)+1 << " " << faces(i,1)+1 << " " << faces(i,2)+1 << "\n";
    ss << "3 1 4 " << elements.rows() << "\n";
    for (int i = 0; i < elements.rows(); i++)
        ss << faces.rows()+i+1 << " " << elements(i,0)+1 << " " << elements(i,1)+1 << " " << elements(i,2)+1 << " " << elements(i,3)+1 << "\n";
    ss << "$EndElements";

    ss.close();
}

void MeshUtils::createTissueBlock(const std::string& filename, const Real length, const Real width, const Real height, const int num_low_res_subdivisions, const int high_res_multiplier)
{
    // try and discretize so that the elements are roughly cube
    const Real elem_size = std::min(std::min(length, width), height) / num_low_res_subdivisions;
    const Real high_res_elem_size = elem_size / high_res_multiplier;
    const int w = static_cast<int>(width/elem_size);
    const int h = static_cast<int>(height/elem_size);
    const int l = static_cast<int>(length/elem_size);
    std::cout << "size in elements: " << h << "x" << w << "x" << l << std::endl;
    
    auto index = [w, h] (int li, int wi, int hi)
    {
        return hi + wi*(h+1) + li*(w+1)*(h+1);
    };

    std::vector<Vec3r> verts((h+1)*(w+1)*(l+1));
    std::vector<Vec3r> high_res_verts;
    for (int li = 0; li < l+1; li++) 
    {
        for (int wi = 0; wi < w+1; wi++)
        {
            for (int hi = 0; hi < h+1; hi++)
            {
                int ind = index(li, wi, hi);
                Vec3r vert({wi*elem_size, li*elem_size, hi*elem_size});
                verts.at(ind) = vert;
            }
        }
    }

    std::vector<Eigen::Vector3i> faces;
    int face_ind = 0;
    // w*h faces
    for (int wi = 0; wi < w; wi++)
    {
        for (int hi = 0; hi < h; hi++)
        {
            int ff1 = index(0, wi, hi);
            int ff2 = index(0, wi+1, hi);
            int ff3 = index(0, wi+1, hi+1);
            int ff4 = index(0, wi, hi+1);

            Eigen::Vector3i front_face1({ff1, ff2, ff3});
            Eigen::Vector3i front_face2({ff1, ff3, ff4});
            faces.push_back(front_face1);
            faces.push_back(front_face2);

            int bf1 = index(l, wi, hi);
            int bf2 = index(l, wi+1, hi);
            int bf3 = index(l, wi+1, hi+1);
            int bf4 = index(l, wi, hi+1);
            Eigen::Vector3i back_face1({bf1, bf2, bf3});
            Eigen::Vector3i back_face2({bf1, bf3, bf4});
            faces.push_back(back_face1);
            faces.push_back(back_face2);

            face_ind += 4;
        }
    }

    // l*h faces
    for (int hi = 0; hi < h; hi++)
    {
        for (int li = 0; li < l; li++)
        {
            int rf1 = index(li, w, hi);
            int rf2 = index(li+1, w, hi);
            int rf3 = index(li+1, w, hi+1);
            int rf4 = index(li, w, hi+1);
            Eigen::Vector3i right_face1({rf1, rf2, rf3});
            Eigen::Vector3i right_face2({rf1, rf3, rf4});
            faces.push_back(right_face1);
            faces.push_back(right_face2);

            int lf1 = index(li, 0, hi);
            int lf2 = index(li+1, 0, hi);
            int lf3 = index(li+1, 0, hi+1);
            int lf4 = index(li, 0, hi+1);
            Eigen::Vector3i left_face1({lf1, lf2, lf3});
            Eigen::Vector3i left_face2({lf1, lf3, lf4});
            faces.push_back(left_face1);
            faces.push_back(left_face2);

            face_ind += 4;
        }
    }

    int center_max_w = w/2;
    int center_min_w = w/2;
    int center_max_l = l/2;
    int center_min_l = l/2;

    // l*w faces
    for (int li = 0; li < l; li++)
    {
        for (int wi = 0; wi < w; wi++)
        {
            //if (li >= li/4 && li <= 3*li/4 && wi >= wi/4 && wi <= 3*wi/4)
            if (li >= center_min_l && li <= center_max_l && wi >= center_min_w && wi <= center_max_w)
            {
                for (int i = 0; i < high_res_multiplier+1; i++)
                {
                    for (int j = 0; j < high_res_multiplier+1; j++)
                    {
                        Vec3r vert({wi*elem_size + j*high_res_elem_size, li*elem_size + i*high_res_elem_size, height});
                        high_res_verts.push_back(vert);
                    }
                }

                int v00 = verts.size() + high_res_verts.size() - (high_res_multiplier+1) * (high_res_multiplier+1);
                for (int i = 0; i < high_res_multiplier; i++)
                {
                    for (int j = 0; j < high_res_multiplier; j++)
                    {
                        int v1 = v00 + j + i*(high_res_multiplier+1);
                        int v2 = v00 + j + 1 + i*(high_res_multiplier+1);
                        int v3 = v00 + j + 1 + (i+1)*(high_res_multiplier+1);
                        int v4 = v00 + j + (i+1)*(high_res_multiplier+1);
                        Eigen::Vector3i face1(v1, v2, v3);
                        Eigen::Vector3i face2(v1, v3, v4);
                        faces.push_back(face1);
                        faces.push_back(face2);
                        std::cout << "Face1: " << v1 << ", " << v2 << ", " << v3 << std::endl;
                        std::cout << "Face2: " << v1 << ", " << v3 << ", " << v4 << std::endl;
                    }
                }
            }
            else if (wi == center_min_w-1 && li >= center_min_l && li <= center_max_l)
            {
                for (int i = 0; i < high_res_multiplier+1; i++)
                {
                    Vec3r vert({(wi+1)*elem_size, li*elem_size + i*high_res_elem_size, height});
                    high_res_verts.push_back(vert);
                }
                int v00 = verts.size() + high_res_verts.size() - (high_res_multiplier+1);
                // for (int i = 0; i < high_res_multiplier; i++)
                // {
                //     int v1 = v00 + i;
                //     int v2 = v00 + i + 1;
                //     if (i >= high_res_multiplier/2)
                //     {
                //         int v3 = index(wi, li+1, h);
                //         Eigen::Vector3i face({v1, v2, v3});
                //         faces.push_back(face);
                //     }
                //     else
                //     {
                //         int v3 = index(wi, li, h);
                //         Eigen::Vector3i face({v1, v2, v3});
                //         faces.push_back(face);
                //     }
                // }

                int v1_mid = index(wi, li, h);
                int v2_mid = v00 + high_res_multiplier/2;
                int v3_mid = index(wi, li+1, h);
                Eigen::Vector3i mid_face({v1_mid, v2_mid, v3_mid});
                faces.push_back(mid_face);
            }
            else
            {

                int tf1 = index(li, wi, h);
                int tf2 = index(li, wi+1, h);
                int tf3 = index(li+1, wi+1, h);
                int tf4 = index(li+1, wi, h);
                Eigen::Vector3i top_face1({tf1, tf2, tf3});
                Eigen::Vector3i top_face2({tf1, tf3, tf4});
                faces.push_back(top_face1);
                faces.push_back(top_face2);
                face_ind += 4;
            }

            int bf1 = index(li, wi, 0);
            int bf2 = index(li, wi+1, 0);
            int bf3 = index(li+1, wi+1, 0);
            int bf4 = index(li+1, wi, 0);
            Eigen::Vector3i bottom_face1({bf1, bf2, bf3});
            Eigen::Vector3i bottom_face2({bf1, bf3, bf4});
            faces.push_back(bottom_face1);
            faces.push_back(bottom_face2);

        }
    }
    
    verts.insert(verts.end(), high_res_verts.begin(), high_res_verts.end());

    std::ofstream ss(filename);
    for (size_t i = 0; i < verts.size(); i++)
    {
        ss << "v " << verts[i](0) << " " << verts[i](1) << " " << verts[i](2) << "\n";
    }

    for (size_t i = 0; i < faces.size(); i++)
    {
        ss << "f " << faces[i](0)+1 << " " << faces[i](1)+1 << " " << faces[i](2)+1 << "\n";
    }

    ss.close();
}

void MeshUtils::convertToSTL(const std::string& filename)
{
    std::cout << "MeshUtils::convertToSTL - converting " << filename << " to .stl format..." << std::endl;
    std::filesystem::path file_path(filename);

    Assimp::Importer importer;

    // And have it read the given file with some example postprocessing
    // Usually - if speed is not the most important aspect for you - you'll
    // probably to request more postprocessing than we do in this example.
    const aiScene* scene = importer.ReadFile( filename,
        aiProcess_Triangulate            |
        aiProcess_JoinIdenticalVertices  |
        aiProcess_FixInfacingNormals |
        aiProcess_SortByPType);

    // If the import failed, report it
    if (scene == nullptr)
    {
        std::cerr << "\tAssimp::Importer could not open " << filename << std::endl;
        std::cerr << "\tEnsure that the file is in a format that assimp can handle." << std::endl;
        return;
    }

    std::cout << "\tAssimp import successful" << std::endl;

    Assimp::Exporter exporter;
    const std::string& stl_filename = file_path.replace_extension(".stl").string();
    exporter.Export(scene, "stl", stl_filename);

    std::cout << "\tAssimp export successful - new file is " << stl_filename << "\n" << std::endl;
}

void MeshUtils::convertSTLtoMSH(const std::string& filename)
{
    std::cout << "MeshUtils::convertSTLtoMSH - converting " << filename << " from .stl to .msh format..." << std::endl;

    // ensure the file exists
    if (!std::filesystem::exists(filename))
    {
        std::cerr << "\t" << filename << " does not exist!" << std::endl;
        return;
    }


    std::filesystem::path file_path(filename);

    // ensure the file is an STL file
    if (file_path.extension() != ".stl" && file_path.extension() != ".STL")
    {
        std::cerr << "\t" << filename << " is not a .stl file!" << std::endl;
        return;
    }

    gmsh::open(filename);
    gmsh::option::setNumber("General.Verbosity", 5);

    // Get the surface (should be tag 1)
    std::vector<std::pair<int, int>> surfaces;
    gmsh::model::getEntities(surfaces, 2);
    std::cout << "Surface tag: " << surfaces[0].second << std::endl;

    // Method 1: Force create volume with the actual surface tag
    int surface_tag = surfaces[0].second;
    int surface_loop_tag = gmsh::model::geo::addSurfaceLoop(std::vector<int>{surface_tag});
    int volume_tag = gmsh::model::geo::addVolume(std::vector<int>{surface_loop_tag});
    gmsh::model::geo::synchronize();

    std::cout << "Created volume with tag: " << volume_tag << std::endl;

    // Generate mesh
    gmsh::model::mesh::generate(3);


    const std::string& msh_filename = file_path.replace_extension(".msh").string();
    gmsh::write(msh_filename);

    std::cout << "\tGMSH conversion to .msh successful - new file is " << msh_filename << "\n" << std::endl;
    
}

void MeshUtils::verticesAndFacesFromFixedFacesFile(const std::string& filename, std::set<int>& vertices, std::vector<int>& faces)
{
    std::ifstream infile(filename);
    std::string line;

    while (std::getline(infile, line))
    {
        std::istringstream iss(line);
        int a, v1, v2, v3, f;
        if (!(iss >> a >> v1 >> v2 >> v3 >> f)) { break; }

        // add vertex indices to set
        vertices.insert(v1);
        vertices.insert(v2);
        vertices.insert(v3);

        faces.push_back(f);
    }
    
}