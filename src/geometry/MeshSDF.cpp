#include "geometry/MeshSDF.hpp"

#include "simobject/RigidMeshObject.hpp"

#ifdef HAVE_CUDA
#include "gpu/resource/MeshSDFGPUResource.hpp"
#endif

namespace Geometry
{

MeshSDF::MeshSDF(const Sim::RigidMeshObject* mesh_obj, const Config::RigidMeshObjectConfig* config)
    : SDF(), _mesh_obj(mesh_obj)
{
    if (config && config->sdfFilename().has_value())
    {
        _from_file = true;
        // load SDF from file
        _sdf = mesh2sdf::MeshSDF(config->sdfFilename().value());
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
    }
}

inline Real MeshSDF::evaluate(const Vec3r& x) const
{
    // transform x into body coordinates
    const Vec3r x_body = _mesh_obj->globalToBody(x);
    // SDF may not be centered about the origin
    if (_from_file)
    {
        // TODO: fix alignment between SDF coordinates and body coordinates
        const mesh2sdf::BoundingBox sdf_mesh_bbox = _sdf.meshBoundingBox();
        const Vec3r sdf_mesh_size = sdf_mesh_bbox.second - sdf_mesh_bbox.first;
        const Vec3r obj_mesh_size = _mesh_obj->mesh()->unrotatedSize();
        const Vec3r sdf_mesh_cm = _sdf.meshMassCenter();
        const Vec3r scaling_factors_xyz = sdf_mesh_size.array() / obj_mesh_size.array();
        const Vec3r x_sdf = sdf_mesh_cm.array() + x_body.array() * scaling_factors_xyz.array();
        // std::cout << "x_body: " << x_body[0] << ", " << x_body[1] << ", " << x_body[2] << std::endl;
        // std::cout << "x_sdf: " << x_sdf[0] << ", " << x_sdf[1] << ", " << x_sdf[2] << std::endl;
        const Real dist = _sdf.evaluate(x_sdf);
        // std::cout << "dist: " << dist << std::endl;
        const Vec3r grad = _sdf.gradient(x_sdf);
        const Vec3r scaled_dist_vec =  grad.array() * dist / scaling_factors_xyz.array();
        return scaled_dist_vec.norm() * ( (dist < 0) ? -1 : 1);
    }
    return _sdf.evaluate(x_body);
}

inline Vec3r MeshSDF::gradient(const Vec3r& x) const
{
    // transform x into body coordinates
    const Vec3r x_body = _mesh_obj->globalToBody(x);
    Vec3r grad;
    // SDF may not be centered about the origin
    if (_from_file)
    {
        // TODO: fix alignment between SDF coordinates and body coordinates
        const mesh2sdf::BoundingBox sdf_mesh_bbox = _sdf.meshBoundingBox();
        const Vec3r sdf_mesh_size = sdf_mesh_bbox.second - sdf_mesh_bbox.first;
        const Vec3r obj_mesh_size = _mesh_obj->mesh()->unrotatedSize();
        const Vec3r sdf_mesh_cm = _sdf.meshMassCenter();
        const Vec3r scaling_factors_xyz = sdf_mesh_size.array() / obj_mesh_size.array();
        const Vec3r x_sdf = sdf_mesh_cm.array() + x_body.array() * scaling_factors_xyz.array();
        grad = _sdf.gradient(x_sdf);
    }
    else
    {
        grad = _sdf.gradient(x_body);
    }
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