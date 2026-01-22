#include "collision/CollisionScene.hpp"
#include "simulation/Simulation.hpp"
#include "simobject/RigidPrimitives.hpp"
#include "simobject/MeshObject.hpp"
#include "simobject/RigidMeshObject.hpp"
#include "simobject/XPBDMeshObject.hpp"
#include "simobject/VirtuosoArm.hpp"
#include "simobject/VirtuosoRobot.hpp"
#include "geometry/SphereSDF.hpp"
#include "geometry/BoxSDF.hpp"
#include "geometry/CylinderSDF.hpp"
#include "geometry/MeshSDF.hpp"
#include "geometry/Mesh.hpp"
#include "geometry/VirtuosoArmSDF.hpp"
#include "utils/GeometryUtils.hpp"

#ifdef HAVE_CUDA
#include "gpu/resource/GPUResource.hpp"
#include "gpu/resource/MeshGPUResource.hpp"
#include "gpu/Collision.cuh"
#endif

// namespace Collision
// {

CollisionScene::CollisionScene(const Sim::Simulation* sim, Geometry::EmbreeScene* embree_scene)
    : _sim(sim), _embree_scene(embree_scene)
{

}

void CollisionScene::collideObjects()
{
    // collide object pairs
    _objects.for_each_element([this](auto obj1){
        _objects.for_each_element([this, obj1](auto obj2){
            // skip when objects are the same
            if ((void*)obj1 == (void*)obj2)
                return;

            _collideObjectPair(obj1, obj2);
        });
    });

    // run self-collision tests for any objects with self-collisions enabled
    for (auto& xpbd_obj : _self_collision_objects)
    {
        _embree_scene->updateObject(xpbd_obj.getAsTetMeshObject());
        xpbd_obj.selfCollisionCheck();
    }
}

void CollisionScene::_collideObjectPair(Sim::Object* /*obj1*/, Sim::Object* /*obj2*/)
{
    // do nothing in the general case
}

template <bool IsFirstOrder>
void CollisionScene::_collideObjectPair(Sim::VirtuosoArm* virtuoso_arm, Sim::XPBDMeshObject_Base_<IsFirstOrder>* xpbd_mesh_obj)
{
    return;
    const typename Sim::XPBDMeshObject_Base::SDFType* mesh_sdf = xpbd_mesh_obj->SDF();

    // sample points along backbone to check against the Deformable SDF
    const Vec3r& inner_tube_start = virtuoso_arm->innerTubeStartFrame().origin();
    const Vec3r& inner_tube_end = virtuoso_arm->innerTubeEndFrame().origin();
    
    const Real it_dia = virtuoso_arm->innerTubeOuterDiameter();

    const Vec3r& dir = (inner_tube_end - inner_tube_start).normalized();
    const Real dist_thresh = 0.5*it_dia;

    const int num_samples = (inner_tube_end - inner_tube_start).norm() / dist_thresh;
    // std::cout << "Num samples: " << num_samples << std::endl;
    for (int i = 0; i < num_samples; i++)
    {
        const Vec3r& pos = inner_tube_start + dir*dist_thresh*i;
        const Real sdf_dist = mesh_sdf->evaluate(pos);

        if (sdf_dist <= dist_thresh)
        {
            // there is penetration
            std::cout << "DeformableSDF collision!" << std::endl;

            // find the index of closest face and the closest surface point on the deformable mesh
            const auto [face_ind, closest_point] = mesh_sdf->closestSurfacePoint(pos);

            // calculate barycentric coordinates of closest surface point
            const Eigen::Vector3i& f = xpbd_mesh_obj->mesh()->face(face_ind);
            const Vec3r& p1 = xpbd_mesh_obj->mesh()->vertex(f[0]);
            const Vec3r& p2 = xpbd_mesh_obj->mesh()->vertex(f[1]);
            const Vec3r& p3 = xpbd_mesh_obj->mesh()->vertex(f[2]);
            const auto [u, v, w] = GeometryUtils::barycentricCoords(pos, p1, p2, p3);


            // calculate collision normal
            const Vec3r collision_normal = (closest_point - pos).normalized();

            // find appropriate point on surface of Virtuoso arm (a.k.a a cylinder)
            // Hacky way: move some distance towards the surface in the opposite direction from the collision normal and query the arm's SDF
            const typename Sim::VirtuosoArm::SDFType* arm_sdf = virtuoso_arm->SDF();
            const Vec3r arm_sdf_query_point = pos - collision_normal*it_dia;
            const Real arm_sdf_dist = arm_sdf->evaluate(arm_sdf_query_point);
            const Vec3r arm_sdf_grad = arm_sdf->gradient(arm_sdf_query_point);
            const Vec3r arm_surface_point = arm_sdf_query_point - arm_sdf_dist * arm_sdf_grad;
            xpbd_mesh_obj->addStaticCollisionConstraint(arm_sdf, arm_surface_point, collision_normal, face_ind, u, v, w);
        }
    }

    // std::cout << "DeformableSDF distance: " << dist << std::endl;
}

template <bool IsFirstOrder>
void CollisionScene::_collideObjectPair(Sim::XPBDMeshObject_Base_<IsFirstOrder>* /*xpbd_mesh_obj1*/, Sim::XPBDMeshObject_Base_<IsFirstOrder>* /*xpbd_mesh_obj2*/)
{
    // deformable-deformable collision not supported
}

template <bool IsFirstOrder>
void CollisionScene::_collideObjectPair(Sim::XPBDMeshObject_Base_<IsFirstOrder>* xpbd_mesh_obj, Sim::VirtuosoArm* virtuoso_arm)
{
    // iterate through faces of mesh
    const typename Sim::VirtuosoArm::SDFType* sdf = virtuoso_arm->SDF();
    const Geometry::Mesh* mesh = xpbd_mesh_obj->mesh();

    // std::unordered_set<int> elems_to_refine;
    for (const auto& face_ind : mesh->faces().validIndices())
    {
        const Vec3i& f = mesh->face(face_ind);
        const Vec3r& p1 = mesh->vertex(f[0]);
        const Vec3r& p2 = mesh->vertex(f[1]);
        const Vec3r& p3 = mesh->vertex(f[2]);

        // check if centroid of face is close
        const Real centroid_dist = sdf->evaluate((p1+p2+p3)/3);

        const Real p1p2 = (p2-p1).norm();
        const Real p1p3 = (p3-p1).norm();
        const Real p2p3 = (p3-p2).norm();

        const Real max_edge = std::max({p1p2, p1p3, p2p3});
        if (centroid_dist > max_edge/2)
            continue;
        
        Real char_dim = 0.5e-3;
        int num_subdivisions = std::max(0.0, std::floor(std::log2(2*max_edge / 0.5e-3)));
        int num_steps = 1 << num_subdivisions;
        Real step = 1.0/num_steps;

        int num_tested_points = 0;
        for (int i = 0; i < num_steps+1; i++)
        {
            for (int j = 0; j < num_steps+1 - i; j++)
            {
                Real u1 = static_cast<Real>(i)/num_steps;
                Real v1 = static_cast<Real>(j)/num_steps;
                Real w1 = 1 - u1 - v1;
                if (w1 < 0)
                    continue;
                
                Real u2 = u1-step;
                Real v2 = v1+step;
                Real w2 = w1;
                if (v2 > 1 || u2 < 0)
                    continue;
                
                Real u3 = u2;
                Real v3 = v1;
                Real w3 = w1+step;
                if (w3 > 1)
                    continue;

                num_tested_points++;

                Real ux = (u1+u2+u3)/3;
                Real vx = (v1+v2+v3)/3;
                Real wx = (w1+w2+w3)/3;
                Vec3r x = ux*p1 + vx*p2 + wx*p3;

                auto result = sdf->evaluateWithGradientAndNodeInfo(x);
                if (result.distance <= char_dim)
                {
                    // const auto [u, v, w] = GeometryUtils::barycentricCoords(x, p1, p2, p3);
                    // std::cout << "Calculated u,v,w: " << u << ", " << v << ", " << w << std::endl;
                    // std::cout << "Assumed u,v,w: " << (u1+u2+u3)/3 << ", " << (v1+v2+v3)/3 << ", " << (w1+w2+w3)/3 << std::endl;
                    const Vec3r surface_x = x - result.gradient*result.distance;
                    Solver::ConstraintProjectorReferenceWrapper<Solver::StaticDeformableCollisionConstraint> proj_ref = 
                        xpbd_mesh_obj->addStaticCollisionConstraint(sdf, surface_x, result.gradient, face_ind, ux, vx, wx);
                    
                    virtuoso_arm->addCollisionConstraint(std::move(proj_ref), result.node_index, result.interp_factor);
                }

                // bottom triangle
                u3 = u1;
                v3 = v2;
                w3 = w1-step;
                if (w3 < 0)
                    continue;

                num_tested_points++;

                ux = (u1+u2+u3)/3;
                vx = (v1+v2+v3)/3;
                wx = (w1+w2+w3)/3;
                x = ux*p1 + vx*p2 + wx*p3;

                result = sdf->evaluateWithGradientAndNodeInfo(x);
                if (result.distance <= char_dim)
                {
                    const Vec3r surface_x = x - result.gradient*result.distance;
                    Solver::ConstraintProjectorReferenceWrapper<Solver::StaticDeformableCollisionConstraint> proj_ref = 
                        xpbd_mesh_obj->addStaticCollisionConstraint(sdf, surface_x, result.gradient, face_ind, ux, vx, wx);
                    
                    virtuoso_arm->addCollisionConstraint(std::move(proj_ref), result.node_index, result.interp_factor);
                }
                
            }
        }

        // const int num_samples = (int)(2*max_edge / 0.5e-3);

        // // const int num_samples = 4;
        // for (int si = 0; si <= num_samples; si++)
        // {
        //     for (int sj = 0; sj <= num_samples - si; sj++)
        //     {
        //         const Real u = (Real)(si+1) / (num_samples+2);
        //         const Real v = (Real)(sj+1) / (num_samples+2);
        //         const Real w = 1 - u - v;
        //         const Vec3r x = u*p1 + v*p2 + w*p3;
        //         const auto result = sdf->evaluateWithGradientAndNodeInfo(x);
        //         if (result.distance <= virtuoso_arm->innerTubeOuterDiameter())
        //         {// collision occurred, find barycentric coordinates (u,v,w) of x on triangle face
        //             // from https://ceng2.ktu.edu.tr/~cakir/files/grafikler/Texture_Mapping.pdf
        //             const auto [u, v, w] = GeometryUtils::barycentricCoords(x, p1, p2, p3);
        //             const Vec3r surface_x = x - result.gradient*result.distance;
        //             Solver::ConstraintProjectorReferenceWrapper<Solver::StaticDeformableCollisionConstraint> proj_ref = 
        //                 xpbd_mesh_obj->addStaticCollisionConstraint(sdf, surface_x, result.gradient, face_ind, u, v, w);
                    
        //             virtuoso_arm->addCollisionConstraint(std::move(proj_ref), result.node_index, result.interp_factor);
                    
        //         }
        //     }
        // }
        // std::cout << "Num subdivisions: " << num_subdivisions << std::endl;
        // std::cout << "Num subdivision points: " << num_steps << std::endl;
        // std::cout << "Num tested points: " << num_tested_points << std::endl;
    }
}

template <bool IsFirstOrder>
void CollisionScene::_collideObjectPair(Sim::XPBDMeshObject_Base_<IsFirstOrder>* xpbd_mesh_obj, Sim::RigidObject* rigid_obj)
{
    // iterate through faces of mesh
    const Geometry::SDF* sdf = rigid_obj->SDF();
    const Geometry::Mesh* mesh = xpbd_mesh_obj->mesh();
    for (const auto& i : mesh->faces().validIndices())
    {
        const Eigen::Vector3i& f = mesh->face(i);
        const Vec3r& p1 = mesh->vertex(f[0]);
        const Vec3r& p2 = mesh->vertex(f[1]);
        const Vec3r& p3 = mesh->vertex(f[2]);

        // check if centroid of face is close
        const Real p1p2 = (p2-p1).squaredNorm();
        const Real p1p3 = (p3-p1).squaredNorm();
        const Real p2p3 = (p3-p2).squaredNorm();
        const Real max_edge = std::max({p1p2, p1p3, p2p3});
        const Real centroid_dist = sdf->evaluate((p1+p2+p3)/3);
        if (centroid_dist*centroid_dist > max_edge)
            continue;

        const int num_samples = 1;
        for (int si = 0; si <= num_samples; si++)
        {
            for (int sj = 0; sj <= num_samples - si; sj++)
            {
                const Real u = (Real)(si+1) / (num_samples+2);
                const Real v = (Real)(sj+1) / (num_samples+2);
                const Real w = 1 - u - v;
                const Vec3r x = u*p1 + v*p2 + w*p3;
                const Real dist = sdf->evaluate(x);
                if (dist <= 1e-4)
                {// collision occurred, find barycentric coordinates (u,v,w) of x on triangle face
                    // from https://ceng2.ktu.edu.tr/~cakir/files/grafikler/Texture_Mapping.pdf
                    const auto [u, v, w] = GeometryUtils::barycentricCoords(x, p1, p2, p3);
                    const Vec3r grad = sdf->gradient(x);
                    const Vec3r surface_x = x - grad*dist;
                    // Solver::ConstraintProjectorReferenceWrapper<Solver::StaticDeformableCollisionConstraint> proj_ref = 
                    xpbd_mesh_obj->addStaticCollisionConstraint(sdf, surface_x, grad, i, u, v, w);
                    
                    // virtuoso_arm->addCollisionConstraint(std::move(proj_ref), result.node_index, result.interp_factor);
                    
                }
            }
        }
        // const Vec3r x = _frankWolfe(sdf, p1, p2, p3);
        // const double distance = sdf->evaluate(x);
        // if (distance <= 1e-4)
        // {// collision occurred, find barycentric coordinates (u,v,w) of x on triangle face
        //     // from https://ceng2.ktu.edu.tr/~cakir/files/grafikler/Texture_Mapping.pdf
        //     const auto [u, v, w] = GeometryUtils::barycentricCoords(x, p1, p2, p3);
        //     const Vec3r grad = sdf->gradient(x);
        //     const Vec3r surface_x = x - grad*distance;
            
        //     if (rigid_obj->isFixed())
        //     {
        //         xpbd_mesh_obj->addStaticCollisionConstraint(sdf, surface_x, grad, i, u, v, w);
        //     }
        //     else
        //     {
        //         xpbd_mesh_obj->addRigidDeformableCollisionConstraint(sdf, rigid_obj, surface_x, grad, i, u, v, w);
        //     }
            
        // }
    }
}

template <bool IsFirstOrder>
void CollisionScene::_collideObjectPair(Sim::XPBDMeshObject_Base_<IsFirstOrder>* xpbd_mesh_obj, Sim::Object* obj2)
{
    // iterate through faces of mesh
    const Geometry::SDF* sdf = obj2->SDF();
    const Geometry::Mesh* mesh = xpbd_mesh_obj->mesh();
    for (const auto& i : mesh->faces().validIndices())
    {
        const Eigen::Vector3i& f = mesh->face(i);
        const Vec3r& p1 = mesh->vertex(f[0]);
        const Vec3r& p2 = mesh->vertex(f[1]);
        const Vec3r& p3 = mesh->vertex(f[2]);

        // check if centroid of face is close
        const Real p1p2 = (p2-p1).squaredNorm();
        const Real p1p3 = (p3-p1).squaredNorm();
        const Real p2p3 = (p3-p2).squaredNorm();
        const Real max_edge = std::max({p1p2, p1p3, p2p3});
        const Real centroid_dist = sdf->evaluate((p1+p2+p3)/3);
        if (centroid_dist*centroid_dist > max_edge)
            continue;

        const Vec3r x = _frankWolfe(sdf, p1, p2, p3);
        const double distance = sdf->evaluate(x);
        if (distance <= 1e-4)
        {// collision occurred, find barycentric coordinates (u,v,w) of x on triangle face
            // from https://ceng2.ktu.edu.tr/~cakir/files/grafikler/Texture_Mapping.pdf
            const auto [u, v, w] = GeometryUtils::barycentricCoords(x, p1, p2, p3);
            const Vec3r grad = sdf->gradient(x);
            const Vec3r surface_x = x - grad*distance;
            xpbd_mesh_obj->addStaticCollisionConstraint(sdf, surface_x, grad, i, u, v, w);
            
        }
    }
}

Vec3r CollisionScene::_frankWolfe(const Geometry::SDF* sdf, const Vec3r& p1, const Vec3r& p2, const Vec3r& p3) const
{
    // find starting iterate - the triangle vertex with the smallest value of SDF
    const Real d_p1 = sdf->evaluate(p1);
    const Real d_p2 = sdf->evaluate(p2);
    const Real d_p3 = sdf->evaluate(p3);

    Vec3r x;
    if (d_p1 <= d_p2 && d_p1 <= d_p3)       x = p1;
    else if (d_p2 <= d_p1 && d_p2 <= d_p3)  x = p2;
    else                                    x = p3;

    Vec3r s;
    for (int i = 0; i < 32; i++)
    {
        const Real alpha = 2.0/(i+3);
        const Vec3r& gradient = sdf->gradient(x);
        const Real sg1 = p1.dot(gradient);
        const Real sg2 = p2.dot(gradient);
        const Real sg3 = p3.dot(gradient);

        if (sg1 < sg2 && sg1 < sg3)       s = p1;
        else if (sg2 < sg1 && sg2 < sg3)  s = p2;
        else                                s = p3;

        x = x + alpha * (s - x);
        
    }

    return x;
}

// } // namespace Collision