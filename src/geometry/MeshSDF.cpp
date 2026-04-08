#include "geometry/MeshSDF.hpp"

#include "simobject/RigidMeshObject.hpp"

#ifdef HAVE_CUDA
#include "gpu/resource/MeshSDFGPUResource.hpp"
#endif

#include <filesystem>

namespace Geometry
{

MeshSDF::MeshSDF(const Sim::RigidMeshObject* mesh_obj, const Config::RigidMeshObjectConfig* config)
    : SDF(), _mesh_obj(mesh_obj)
{
    // search for a cached SDF file that has the same number of vertices and faces and unrotated size

    // filename
    int num_vertices = mesh_obj->mesh()->numVertices();
    int num_faces = mesh_obj->mesh()->numFaces();
    Vec3r size = mesh_obj->mesh()->unrotatedSize();
    std::stringstream filename;
    filename << "v" << num_vertices << "_f" << num_faces << "_s" << int(1000*size[0]) << "x" << int(1000*size[1]) << "x" << int(1000*size[2]) << ".sdf";

    // look for file
    if (std::filesystem::exists(filename.str()))
    {
        // found it, load from file
        _sdf = mesh2sdf::MeshSDF(filename.str());
    }
    else
    {
        // calculate the SDF at the mesh's un-transformed state
        // if the corresponding RigidMeshObject to this SDF has an initial rotation, the mesh is already rotated, which will throw off the SDF
        Geometry::Mesh mesh_copy(*(mesh_obj->mesh()));
        // untranslate the copy of the mesh
        mesh_copy.moveTogether(-mesh_obj->position());
        // unrotate the copy of the mesh
        const Mat3r rot_mat = GeometryUtils::quatToMat(GeometryUtils::inverseQuat(mesh_obj->orientation()));
        mesh_copy.rotateAbout(Vec3r::Zero(), rot_mat);
        
        Eigen::Matrix<Real, 3, -1, Eigen::ColMajor> verts(3, mesh_copy.vertices().size());
        Eigen::Matrix<int, 3, -1, Eigen::ColMajor> faces(3, mesh_copy.faces().size());
        
        int v_ind = 0;
        for (const auto& v : mesh_copy.vertices())
            verts.col(v_ind++) = v;
        int f_ind = 0;
        for (const auto& f : mesh_copy.faces())
            faces.col(f_ind++) = f;
        // compute the SDF
        _sdf = mesh2sdf::MeshSDF(verts, faces, 128, 5, true);
        
        // write to file to cache for later
        _sdf.writeToFile(filename.str());
    }
}

inline Real MeshSDF::evaluate(const Vec3r& x) const
{
    // transform x into body coordinates
    const Vec3r x_body = _mesh_obj->globalToBody(x);
    return _sdf.evaluate(x_body);
}

inline Vec3r MeshSDF::gradient(const Vec3r& x) const
{
    // transform x into body coordinates
    const Vec3r x_body = _mesh_obj->globalToBody(x);
    Vec3r grad = _sdf.gradient(x_body);
    return GeometryUtils::rotateVectorByQuat(grad, _mesh_obj->orientation());
}

 #ifdef HAVE_CUDA
inline void MeshSDF::createGPUResource() 
{
    _gpu_resource = std::make_unique<Sim::MeshSDFGPUResource>(this);
    _gpu_resource->allocate();
}
 #endif

} // namespace Geometry