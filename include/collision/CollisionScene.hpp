#ifndef __COLLISION_SCENE_HPP
#define __COLLISION_SCENE_HPP

// #include "simobject/MeshObject.hpp"
#include "simobject/Object.hpp"
#include "simobject/XPBDMeshObject.hpp"
#include "simobject/RigidMeshObject.hpp"
#include "simobject/RigidObject.hpp"
#include "simobject/RigidPrimitives.hpp"
#include "simobject/VirtuosoArm.hpp"
#include "simobject/VirtuosoRobot.hpp"

#include "geometry/embree/EmbreeScene.hpp"

#include "common/VariadicVectorContainer.hpp"
#include "common/SimulationTypeDefs.hpp"

#ifdef HAVE_CUDA
#include "gpu/resource/WritableArrayGPUResource.hpp"
#include "gpu/GPUStructs.hpp"
#endif

namespace Sim
{
    class Simulation;
}

/** Responsible for determining collisions between objects in the simulation.
 * When a collision is identified between two objects, a collision constraint is created to resolve the collision over the subsequent time steps.
 * When the Simulation adds an object to the CollisionScene, a collision geometry (i.e. a SDF) is created for that object.
 */
class CollisionScene
{
    public:
    using ObjectVectorType = VariadicVectorContainerFromTypeList<SimulationObjectTypes>::ptr_type;

    public:
    /** Constructor - needs a reference back to the simulation to access the time step, current sim time, etc. */
    explicit CollisionScene(const Sim::Simulation* sim, Geometry::EmbreeScene* embree_scene);

    /** Adds a new object to the CollisionScene.
     * Creates a SDF for the object and adds the object's pointer to the vector of objects in the CollisionScene,
     *   but only if collisions = true.
     * 
     * @param obj - the pointer to the new simulation object to be added to the scene
     * @param collisions - whether or not collisions are enabled for this object. If collisions are not enabled, still add to the Embree ray scene for ray-tracing queries.
    */
    template <typename ObjectType>
    void addObject(ObjectType* obj, bool collisions=true)
    {
        _embree_scene->addObject(obj);

        if (!collisions)
            return;

        obj->createSDF();
        
#ifdef HAVE_CUDA
        const typename ObjectType::SDFType* sdf = obj->SDF();
        if (sdf)
        {
            sdf->createGPUResource();
            sdf->gpuResource()->fullCopyToDevice();
        }

        if (Sim::XPBDMeshObject_Base* mesh_obj = dynamic_cast<Sim::XPBDMeshObject_Base*>(obj))
        {
            // create a managed resource for the mesh
            Geometry::Mesh* mesh_ptr = mesh_obj->mesh();
            mesh_ptr->createGPUResource();
            mesh_ptr->gpuResource()->fullCopyToDevice();
        
            // create a block of data of GPUCollision structs that will be populated during collision detection
            // at most, we will have one collision per face in the mesh, so to be safe this is the amount of memory we allocate
            std::vector<Sim::GPUCollision> collisions_vec(mesh_obj->mesh()->numFaces());

            // initialize the time for each collision slot to some negative number so that we can distinguish when there is an active collision
            for (auto& gc : collisions_vec)
            {
                gc.penetration_dist = 100;
            }

            // CHECK_CUDA_ERROR(cudaHostRegister(collisions_vec.data(), collisions_vec.size()*sizeof(Sim::GPUCollision), cudaHostRegisterDefault));

            // create the GPUResource for the array of collision structs
            std::unique_ptr<Sim::WritableArrayGPUResource<Sim::GPUCollision>> arr_resource = 
                std::make_unique<Sim::WritableArrayGPUResource<Sim::GPUCollision>>(collisions_vec.data(), mesh_obj->mesh()->numFaces());
            arr_resource->allocate();

            assert(_gpu_collisions.count(new_obj) == 0);

            // move the vector into the member map
            _gpu_collisions[new_obj] = std::move(collisions_vec);
            _gpu_collision_resources[new_obj] = std::move(arr_resource);
        }
#endif
        _objects.template push_back<ObjectType*>(obj);

    }

    /** Specialization for VirtuosoRobot - for now, we just add its arms separately to the CollisionScene.
     * (we don't care about collisions between the endoscope and the tissue)
     */
    void addObject(Sim::VirtuosoRobot* virtuoso_robot, bool collisions=true)
    {
        if (collisions)
        {
            if (virtuoso_robot->hasArm1())
            {
                addObject(virtuoso_robot->arm1(), collisions);
                if (virtuoso_robot->arm1()->tool())
                    addObject(virtuoso_robot->arm1()->tool(), collisions);
            }
            if (virtuoso_robot->hasArm2())
            {
                addObject(virtuoso_robot->arm2(), collisions);
                if (virtuoso_robot->arm2()->tool())
                    addObject(virtuoso_robot->arm2()->tool(), collisions);
            }
        }
    }

    /** TODO: Add specializations for rigid objects that add them to the Embree scene */

    /** Specialization for XPBDMeshObject */
    template<bool IsFirstOrder>
    void addObject(Sim::XPBDMeshObject_Base_<IsFirstOrder>* xpbd_obj, bool collisions=true, bool self_collisions=false)
    {
        if (collisions)
        {
            xpbd_obj->createSDF();
            _objects.template push_back<Sim::XPBDMeshObject_Base_<IsFirstOrder>*>(xpbd_obj);
        }

            if (self_collisions)
                _self_collision_objects.emplace_back(xpbd_obj);

        // add to EmbreeScene since collisions are enabled
        _embree_scene->addObject(xpbd_obj);

        
    }

    /** Specialization for RigidMeshObject */
    // void addObject(Sim::RigidMeshObject* rigid_mesh_obj, bool collisions=true)
    // {
    //     if (collisions)
    //     {
    //         rigid_mesh_obj->createSDF();
    //         _objects.template push_back<Sim::RigidMeshObject*>(rigid_mesh_obj);
    //     }

    //     // add to EmbreeScene for ray queries (regardless of whether collisions are enabled)
    //     _embree_scene->addObject(rigid_mesh_obj);
    // }

    /** Detects collisions between objects in the CollisionScene.
     * When collisions are detected, collision constraints are created and added to the appropriate objects to resolve collisions.
     */
    void collideObjects();

    /** Collides objects in the CollisionScene with specific faces of an XPBDMeshObject.
     * When collisions are detected, collision constraints are created and added to the appropriate objects to resolve collisions.
     * 
     * This type of query is especially useful for when new faces are created during mesh topology changes (element refinement, removal),
     * and new collision constraints need to be added.
     */
    template<bool IsFirstOrder>
    void collideObjectsWithFacesOfXPBDMeshObj(Sim::XPBDMeshObject_Base_<IsFirstOrder>* xpbd_mesh_obj, const std::vector<int>& face_indices) const;

    protected:
    /** Helper function that checks for collision between a pair of objects.
     * In general, collisions between two arbitrary objects are not supported.
     * Overrides for specific pairs of objects are implemented.
     */
    void _collideObjectPair(Sim::Object* obj1, Sim::Object* obj2);  // most general, does nothing
    template<bool IsFirstOrder>
    void _collideObjectPair(Sim::VirtuosoArm* virtuoso_arm, Sim::XPBDMeshObject_Base_<IsFirstOrder>* xpbd_mesh_obj);
    template<bool IsFirstOrder>
    void _collideObjectPair(Sim::XPBDMeshObject_Base_<IsFirstOrder>* xpbd_mesh_obj1, Sim::XPBDMeshObject_Base_<IsFirstOrder>* xpbd_mesh_obj2);
    template<bool IsFirstOrder>
    void _collideObjectPair(Sim::XPBDMeshObject_Base_<IsFirstOrder>* xpbd_mesh_obj, Sim::VirtuosoArm* virtuoso_arm);
    template<bool IsFirstOrder>
    void _collideObjectPair(Sim::XPBDMeshObject_Base_<IsFirstOrder>* xpbd_mesh_obj, Sim::VirtuosoArmTool_Base* virtuoso_arm_tool);
    template<bool IsFirstOrder>
    void _collideObjectPair(Sim::XPBDMeshObject_Base_<IsFirstOrder>* xpbd_mesh_obj, Sim::RigidObject* rigid_obj);
    template<bool IsFirstOrder>
    void _collideObjectPair(Sim::XPBDMeshObject_Base_<IsFirstOrder>* xpbd_mesh_obj, Sim::Object* obj);

    void _collideObjectPair(Sim::VirtuosoArm* virtuoso_arm1, Sim::VirtuosoArm* virtuoso_arm2);
    void _collideObjectPair(Sim::VirtuosoArm* virtuoso_arm, Sim::Object* obj);
    std::pair<Real,Real> _findDeepestPenetratingPointOnSegment(const Vec3r& p1, const Vec3r& p2, const Geometry::SDF* sdf);

    template<bool IsFirstOrder>
    void _collideXPBDFaceWithObject(Sim::XPBDMeshObject_Base_<IsFirstOrder>* xpbd_mesh_obj, Sim::Object* obj, int face_ind) const;
    template<bool IsFirstOrder>
    void _collideXPBDFaceWithObject(Sim::XPBDMeshObject_Base_<IsFirstOrder>* xpbd_mesh_obj, Sim::VirtuosoArm* virtuoso_arm, int face_ind) const;
    template<bool IsFirstOrder>
    void _collideXPBDFaceWithObject(Sim::XPBDMeshObject_Base_<IsFirstOrder>* xpbd_mesh_obj, Sim::VirtuosoArmTool_Base* virtuoso_arm_tool, int face_ind) const;

    void _lowDiscrepancySampling(Real char_dim, const Vec3r& p1, const Vec3r& p2, const Vec3r& p3, std::function<void(Vec3r, Vec3r)> test_func) const;

    /** Implements the Frank-Wolfe optimization algorithm applied to finding a contact point between a SDF and a 3D triangle face. 
     * @param sdf - the signed distance function (SDF) to collide against
     * @param p1 - 1st triangle vertex
     * @param p2 - 2nd triangle vertex
     * @param p3 - 3rd tringle vertex
     * @returns the closest point on the triangle to the boundary of the SDF - if at this closest point the SDF evaluates to negative, we have a collision!
    */
    Vec3r _frankWolfe(const Geometry::SDF* sdf, const Vec3r& p1, const Vec3r& p2, const Vec3r& p3) const;

    protected:
    /** Non-owning pointer to the Simulation object that this CollisionScene belongs to */
    const Sim::Simulation* _sim;

    /** Stores the objects that have been added to the collision scene. */
    ObjectVectorType _objects;

    /** Stores the objects that have self-collisions enabled.
     * This will only ever be XPBDMeshObjects as these are the only deformables in the scene.
     */
    std::vector<Sim::XPBDMeshObject_BasePtrWrapper> _self_collision_objects;

    /** Pointer to the simulation's Embree scene.
     * The CollisionScene will update the Embree scene as it sees fit.
     */
    Geometry::EmbreeScene* _embree_scene;

    #ifdef HAVE_CUDA
    std::map<Sim::Object*, std::vector<Sim::GPUCollision> > _gpu_collisions;
    std::map<Sim::Object*, std::unique_ptr<Sim::WritableArrayGPUResource<Sim::GPUCollision> > > _gpu_collision_resources;
    #endif


};

#endif // __COLLISION_SCENE_HPP