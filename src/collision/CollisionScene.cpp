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

template<bool IsFirstOrder>
void CollisionScene::collideObjectsWithFacesOfXPBDMeshObj(
    Sim::XPBDMeshObject_Base_<IsFirstOrder>* xpbd_mesh_obj, const std::vector<int>& face_indices) const
{
    _objects.for_each_element([this, &xpbd_mesh_obj, &face_indices](auto obj){
        if ((void*)obj == (void*)xpbd_mesh_obj)
            return;
        
        for (const auto& face_ind : face_indices)
        {
            this->_collideXPBDFaceWithObject(xpbd_mesh_obj, obj, face_ind);
        }
    });
}

void CollisionScene::_lowDiscrepancySampling(Real char_dim, const Vec3r& p1, const Vec3r& p2, const Vec3r& p3, std::function<void(Vec3r, Vec3r)> test_func) const
{
        const Real p1p2 = (p2-p1).norm();
        const Real p1p3 = (p3-p1).norm();
        const Real p2p3 = (p3-p2).norm();

        const Real max_edge = std::max({p1p2, p1p3, p2p3});

        int num_subdivisions = std::max(0.0, std::floor(std::log2(2*max_edge / char_dim)));
        int num_steps = 1 << num_subdivisions;
        Real step = 1.0/num_steps;

        for (int i = 0; i < num_steps+1; i++)
        {
            for (int j = 0; j < num_steps+1 - i; j++)
            {
                // top triangle
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

                Real ux = (u1+u2+u3)/3;
                Real vx = (v1+v2+v3)/3;
                Real wx = (w1+w2+w3)/3;

                // centroid of triangle = test point
                Vec3r x = ux*p1 + vx*p2 + wx*p3;

                test_func(x, Vec3r(ux,vx,wx));
                

                // bottom triangle
                u3 = u1;
                v3 = v2;
                w3 = w1-step;
                if (w3 < 0)
                    continue;

                ux = (u1+u2+u3)/3;
                vx = (v1+v2+v3)/3;
                wx = (w1+w2+w3)/3;

                // centroid of triangle = test point
                x = ux*p1 + vx*p2 + wx*p3;

                test_func(x, Vec3r(ux,vx,wx));
            }
        }
}


template<bool IsFirstOrder>
void CollisionScene::_collideXPBDFaceWithObject(
    Sim::XPBDMeshObject_Base_<IsFirstOrder>* xpbd_mesh_obj, Sim::VirtuosoArm* virtuoso_arm, int face_ind) const
{
    const Geometry::TetMesh* mesh = xpbd_mesh_obj->tetMesh();

    if (!mesh->faceValid(face_ind))
        return;

    const typename Sim::VirtuosoArm::SDFType* sdf = virtuoso_arm->SDF();
    Real char_dim = virtuoso_arm->characteristicDimension();

    const Vec3i& f = mesh->face(face_ind);
    int v1 = f[0]; int v2 = f[1]; int v3 = f[2];
    const Vec3r& p1 = mesh->vertex(v1);
    const Vec3r& p2 = mesh->vertex(v2);
    const Vec3r& p3 = mesh->vertex(v3);

    // check if centroid of face is close
    const Real centroid_dist = sdf->evaluate((p1+p2+p3)/3);

    const Real p1p2 = (p2-p1).norm();
    const Real p1p3 = (p3-p1).norm();
    const Real p2p3 = (p3-p2).norm();

    const Real max_edge = std::max({p1p2, p1p3, p2p3});
    if (centroid_dist > max_edge/2)
        return;
    
    int elem_ind = mesh->elementWithFace(face_ind);

    auto test_func = [&face_ind, &v1, &v2, &v3, &elem_ind, &sdf, &char_dim, &xpbd_mesh_obj, &virtuoso_arm](const Vec3r& x, const Vec3r& bary_coords) {
        auto result = sdf->evaluateWithGradientAndNodeInfo(x);
        if (result.distance <= char_dim)
        {
            const Vec3r surface_x = x - result.gradient*result.distance;
            Solver::ConstraintProjectorReferenceWrapper<Solver::StaticDeformableCollisionConstraint> proj_ref = 
                xpbd_mesh_obj->addStaticCollisionConstraint(
                    sdf, surface_x, result.gradient,
                    v1, v2, v3, bary_coords[0], bary_coords[1], bary_coords[2],
                    elem_ind, face_ind
                );
            
            virtuoso_arm->addCollisionConstraint(std::move(proj_ref), result.node_index, result.interp_factor);
        }
    };

    _lowDiscrepancySampling(char_dim, p1, p2, p3, test_func);

    // check if we should collide the Virtuoso arm with other internal faces of the element
    // we need to do this when the element is inverted, or near inverted (say det(F) < 0.1)
    // this way, if/when we remove the element in contact, the new faces created will not be penetrating the SDF!
    /** 
     * 
     * 
     * TODO: for an inverted element with multiple surface faces, this part will do duplicate work and check the interior
     * faces multiple times and create duplicate collision constraints. For now, this is acceptable. But eventually should be improved.
     * 
     */
    Real detF = mesh->elementDeformationGradient(elem_ind).determinant();
    if (detF < 0.1)
    {
        face_ind = -1;  // the face we are about to test does not correspond to a surface face
        
        std::vector<int> surface_faces = mesh->elementSurfaceFaces(elem_ind);
        const Vec4i& elem_verts = mesh->element(elem_ind);
        
        // get interior faces and run collision on these
        for (int i = 0; i < 4; i++)
        {
            for (int j = i+1; j < 4; j++)
            {
                for (int k = j+1; k < 4; k++)
                {
                    v1 = elem_verts[i]; v2 = elem_verts[j]; v3 = elem_verts[k];

                    // make sure that this face is not on the surface
                    bool on_surface = false;
                    for (const auto& surface_face_ind : surface_faces)
                    {
                        const Vec3i& surface_face = mesh->face(surface_face_ind);
                        if (Geometry::Face(v1, v2, v3) == Geometry::Face(surface_face[0], surface_face[1], surface_face[2]))
                        {
                            on_surface = true;
                            break;
                        }
                    }
                    if (on_surface)
                        continue;
                    
                    // test the face
                    _lowDiscrepancySampling(char_dim, mesh->vertex(v1), mesh->vertex(v2), mesh->vertex(v3), test_func);
                    
                }
            }
        }
    }
}

template<bool IsFirstOrder>
void CollisionScene::_collideXPBDFaceWithObject(Sim::XPBDMeshObject_Base_<IsFirstOrder>* xpbd_mesh_obj, Sim::Object* obj, int face_ind) const
{
    const Geometry::TetMesh* mesh = xpbd_mesh_obj->tetMesh();

    if (!mesh->faceValid(face_ind))
        return;

    const Geometry::SDF* sdf = obj->SDF();

    Real char_dim = obj->characteristicDimension();

    const Vec3i& f = mesh->face(face_ind);
    const Vec3r& p1 = mesh->vertex(f[0]);
    const Vec3r& p2 = mesh->vertex(f[1]);
    const Vec3r& p3 = mesh->vertex(f[2]);

    // check if centroid of face is close
    const Real p1p2 = (p2-p1).squaredNorm();
    const Real p1p3 = (p3-p1).squaredNorm();
    const Real p2p3 = (p3-p2).squaredNorm();
    const Real max_edge = std::max({p1p2, p1p3, p2p3});
    const Real centroid_dist = sdf->evaluate((p1+p2+p3)/3);
    // skip faces that are sufficiently far away
    if (centroid_dist*centroid_dist > max_edge/4)
        return;

    int elem_ind = mesh->elementWithFace(face_ind);

    auto test_func = [&face_ind, &f, &elem_ind, &sdf, &char_dim, &xpbd_mesh_obj](const Vec3r& x, const Vec3r& bary_coords) {
        Real dist = sdf->evaluate(x);
        if (dist <= 1e-4)   // some arbitrary distance threshold
        {
            const Vec3r grad = sdf->gradient(x);
            const Vec3r surface_x = x - grad*dist;
            xpbd_mesh_obj->addStaticCollisionConstraint(sdf, surface_x, grad, 
                f[0], f[1], f[2], bary_coords[0], bary_coords[1], bary_coords[2],
                elem_ind, face_ind
            );
        }
    };

    _lowDiscrepancySampling(char_dim, p1, p2, p3, test_func);

    return;
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
            // xpbd_mesh_obj->addStaticCollisionConstraint(arm_sdf, arm_surface_point, collision_normal, face_ind, u, v, w);
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
    for (const auto& face_ind : xpbd_mesh_obj->mesh()->faces().validIndices())
    {
        _collideXPBDFaceWithObject(xpbd_mesh_obj, virtuoso_arm, face_ind);
    }
}

void CollisionScene::_collideObjectPair(Sim::VirtuosoArm* virtuoso_arm, Sim::Object* obj)
{
    // for each segment of the Virtuoso arm, find the point on the segment that is most deeply penetration the other object
    // this is a univariate optimization problem: we want to find s such that SDF(p(s)) is minimized where p(s) is the position along segment centerline
    const Sim::VirtuosoArm::OuterTubeFramesArray& outer_tube_frames = virtuoso_arm->outerTubeFrames();
    const Sim::VirtuosoArm::InnerTubeFramesArray& inner_tube_frames = virtuoso_arm->innerTubeFrames();
    const Sim::VirtuosoArm::ToolTubeFramesArray& tool_tube_frames = virtuoso_arm->toolTubeFrames();
    
    for (unsigned i = 0; i < outer_tube_frames.size()-1; i++)
    {
        Vec3r p1 = outer_tube_frames[i].origin();
        Vec3r p2 = outer_tube_frames[i+1].origin();
        auto [s, dist] = _findDeepestPenetratingPointOnSegment(p1, p2, obj->SDF());
        
        if (dist < virtuoso_arm->outerTubeOuterDiameter()/2.0)
        {
            // collision between inner tube and other SDF!
            std::cout << "Collision between outer tube segment (" << i << "," << i+1 << ") and SDF! s=" << s << ", dist=" << dist << std::endl;
        }
    }

    for (unsigned i = 0; i < inner_tube_frames.size()-1; i++)
    {
        Vec3r p1 = inner_tube_frames[i].origin();
        Vec3r p2 = inner_tube_frames[i+1].origin();
        auto [s, dist] = _findDeepestPenetratingPointOnSegment(p1, p2, obj->SDF());
        
        if (dist < virtuoso_arm->innerTubeOuterDiameter()/2.0)
        {
            // collision between inner tube and other SDF!
            std::cout << "Collision between inner tube segment (" << i << "," << i+1 << ") and SDF! s=" << s << ", dist=" << dist << std::endl;
        }
    }

    if (virtuoso_arm->hasTool())
    {
        for (unsigned i = 0; i < tool_tube_frames.size()-1; i++)
        {
            Vec3r p1 = tool_tube_frames[i].origin();
            Vec3r p2 = tool_tube_frames[i+1].origin();
            auto [s, dist] = _findDeepestPenetratingPointOnSegment(p1, p2, obj->SDF());
            
            if (dist < virtuoso_arm->toolTube().outer_dia/2.0)
            {
                // collision between inner tube and other SDF!
                std::cout << "Collision between tool tube segment (" << i << "," << i+1 << ") and SDF! s=" << s << ", dist=" << dist << std::endl;
            }
        }
    }
}

std::pair<Real,Real> CollisionScene::_findDeepestPenetratingPointOnSegment(const Vec3r& p1, const Vec3r& p2, const Geometry::SDF* sdf)
{
    // fixed number of initial points to test
    const Real samples[4] = {
        0.1, 0.35, 0.65, 0.9
    };

    Real best_s = 0.0;
    Real best_f = sdf->evaluate(p1);

    for (int i = 0; i < 4; i++)
    {
        double s = samples[i];
        double f = sdf->evaluate(p1 + s*(p2-p1));

        if (f < best_f)
        {
            best_f = f;
            best_s = s;
        }
    }

    // mini golden-section search
    Real a = std::max(Real(0.0), best_s - 0.2);
    Real b = std::min(Real(1.0), best_s + 0.2);
    const Real phi = 0.61803398875;
    Real x = best_s;
    Real fx = best_f;
    for (int i = 0; i < 4; i++)
    {
        Real left = x - phi * (x - a);
        Real right = x + phi * (b - x);

        Real fl = sdf->evaluate(p1 + left*(p2 - p1));
        Real fr = sdf->evaluate(p1 + right*(p2 - p1));

        if (fl < fx)
        {
            b = x;
            x = left;
            fx = fl;
        }
        else if (fr < fx)
        {
            a = x;
            x = right;
            fx = fr;
        }
        else
        {
            a = left;
            b = right;
        }
    }

    return std::make_pair(x, fx);
}

template <bool IsFirstOrder>
void CollisionScene::_collideObjectPair(Sim::XPBDMeshObject_Base_<IsFirstOrder>* xpbd_mesh_obj, Sim::RigidObject* rigid_obj)
{
    // for now, rigid objects treated the same as any generic object
    // (i.e. no special rigid-xpbd coupling...we assume the rigid body is fixed)
    _collideObjectPair(xpbd_mesh_obj, (Sim::Object*)rigid_obj);
}

template <bool IsFirstOrder>
void CollisionScene::_collideObjectPair(Sim::XPBDMeshObject_Base_<IsFirstOrder>* xpbd_mesh_obj, Sim::Object* obj2)
{
    for (const auto& face_ind : xpbd_mesh_obj->mesh()->faces().validIndices())
    {
        _collideXPBDFaceWithObject(xpbd_mesh_obj, obj2, face_ind);
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

template void CollisionScene::collideObjectsWithFacesOfXPBDMeshObj(Sim::XPBDMeshObject_Base_<true>*, const std::vector<int>&) const;
template void CollisionScene::collideObjectsWithFacesOfXPBDMeshObj(Sim::XPBDMeshObject_Base_<false>*, const std::vector<int>&) const;

// } // namespace Collision