#include "graphics/vtk/VTKGraphicsScene.hpp"
#include "graphics/vtk/VTKBoxGraphicsObject.hpp"
#include "graphics/vtk/VTKCylinderGraphicsObject.hpp"
#include "graphics/vtk/VTKSphereGraphicsObject.hpp"
#include "graphics/vtk/VTKMeshGraphicsObject.hpp"
#include "graphics/vtk/VTKVirtuosoArmGraphicsObject.hpp"
#include "graphics/vtk/VTKVirtuosoRobotGraphicsObject.hpp"

#include "simobject/Object.hpp"
#include "simobject/MeshObject.hpp"

#include <vtkOpenGLRenderer.h>
#include <vtkCamera.h>

#include <vtkOutputWindow.h>
#include <vtkObjectFactory.h>
#include <vtkAutoInit.h>

namespace Graphics
{

VTKGraphicsScene::VTKGraphicsScene(const std::string& name, const Config::SimulationRenderConfig& sim_render_config)
    : GraphicsScene(name, sim_render_config)
{

}

void VTKGraphicsScene::init()
{
    _viewer = std::make_unique<VTKViewer>(_name, _sim_render_config);
    _vtk_viewer = dynamic_cast<VTKViewer*>(_viewer.get());
}

void VTKGraphicsScene::update()
{
    // make sure graphics are up to date for all the objects in the scene
    for (auto& obj : _graphics_objects)
    {
        obj->update();
    }

    // update the viewer (signal a redraw)
    _viewer->update();
}

int VTKGraphicsScene::run()
{
    // display the window initially
    _vtk_viewer->displayWindow();

    // then start the interactor 
    _vtk_viewer->interactorStart();

    return 0;
}

int VTKGraphicsScene::addObject(const Sim::Object* obj, const Config::ObjectRenderConfig& render_config)
{
    // make sure object with name doesn't already exist in the scene
    if (getObject(obj->name()))
    {
        std::cout << "GraphicsObject with name " << obj->name() << " already exists in this GraphicsScene!" << std::endl;
        assert(0);
    }

    /** TODO: this stinks! Maybe use templates? */
    std::unique_ptr<GraphicsObject> new_graphics_obj;

    // try downcasting to MeshObject
    if (const Sim::MeshObject* mo = dynamic_cast<const Sim::MeshObject*>(obj))
    {

        // create a new MeshGraphicsObject for visualizing this MeshObject
        auto ptr = std::make_unique<VTKMeshGraphicsObject>(obj->name(), mo->mesh(), render_config);
        if (ptr->facesActor())
            _vtk_viewer->renderer()->AddActor(ptr->facesActor());
        if (ptr->edgesActor())
            _vtk_viewer->renderer()->AddActor(ptr->edgesActor());
        new_graphics_obj = std::move(ptr);
    }

    // try downcasting to a RigidSphere
    else if (const Sim::RigidSphere* sphere = dynamic_cast<const Sim::RigidSphere*>(obj))
    {
        // create a new SphereGraphicsObject for visualizing the sphere
        auto ptr = std::make_unique<VTKSphereGraphicsObject>(sphere->name(), sphere, render_config);
        _vtk_viewer->renderer()->AddActor(ptr->actor());
        new_graphics_obj = std::move(ptr);
        
    }

    // try downcasting to a RigidBox
    else if (const Sim::RigidBox* box = dynamic_cast<const Sim::RigidBox*>(obj))
    {
        auto ptr = std::make_unique<VTKBoxGraphicsObject>(box->name(), box, render_config);
        _vtk_viewer->renderer()->AddActor(ptr->actor());
        new_graphics_obj = std::move(ptr);
    }

    // try downcasting to a RigidCylinder
    else if (const Sim::RigidCylinder* cyl = dynamic_cast<const Sim::RigidCylinder*>(obj))
    {
        auto ptr = std::make_unique<VTKCylinderGraphicsObject>(cyl->name(), cyl, render_config);
        _vtk_viewer->renderer()->AddActor(ptr->actor());
        new_graphics_obj = std::move(ptr);
    }

    // try downcasting to a VirtuosoArm
    else if (const Sim::VirtuosoArm* arm = dynamic_cast<const Sim::VirtuosoArm*>(obj))
    {
        auto ptr = std::make_unique<VTKVirtuosoArmGraphicsObject>(arm->name(), arm, render_config);
        _vtk_viewer->renderer()->AddActor(ptr->actor());
        new_graphics_obj = std::move(ptr);
        
    }

    else if (const Sim::VirtuosoRobot* robot = dynamic_cast<const Sim::VirtuosoRobot*>(obj))
    {
        auto ptr = std::make_unique<VTKVirtuosoRobotGraphicsObject>(robot->name(), robot, render_config);
        _vtk_viewer->renderer()->AddActor(ptr->actor());
        _vtk_viewer->renderer()->AddLight(ptr->endoscopeLight());
        new_graphics_obj = std::move(ptr);

        if (robot->hasArm1())
        {
            std::unique_ptr<VTKVirtuosoArmGraphicsObject> arm1_graphics_obj = 
                std::make_unique<VTKVirtuosoArmGraphicsObject>(robot->arm1()->name(), robot->arm1(), render_config);
            _vtk_viewer->renderer()->AddActor(arm1_graphics_obj->actor());
            _graphics_objects.push_back(std::move(arm1_graphics_obj));
        }
        if (robot->hasArm2())
        {
            std::unique_ptr<VTKVirtuosoArmGraphicsObject> arm2_graphics_obj = 
                std::make_unique<VTKVirtuosoArmGraphicsObject>(robot->arm2()->name(), robot->arm2(), render_config);
            _vtk_viewer->renderer()->AddActor(arm2_graphics_obj->actor());
            _graphics_objects.push_back(std::move(arm2_graphics_obj));
        }

        
    }

     // finally add the GraphicsObject to the list of GraphicsObjects
    _graphics_objects.push_back(std::move(new_graphics_obj));

    return _graphics_objects.size() - 1;
}

void VTKGraphicsScene::setCameraOrthographic()
{
    _vtk_viewer->renderer()->GetActiveCamera()->SetParallelProjection(true);
}

void VTKGraphicsScene::setCameraPerspective()
{
    _vtk_viewer->renderer()->GetActiveCamera()->SetParallelProjection(false);
}

void VTKGraphicsScene::setCameraFOV(Real fov)
{
    _vtk_viewer->renderer()->GetActiveCamera()->UseHorizontalViewAngleOn();
    _vtk_viewer->renderer()->GetActiveCamera()->SetViewAngle(fov);
    _vtk_viewer->renderer()->ResetCameraClippingRange();
}

Vec3r VTKGraphicsScene::cameraViewDirection() const
{
    // double d1, d2, d3;
    // _vtk_viewer->renderer()->GetActiveCamera()->GetDirectionOfProjection(d1, d2, d3);
    // return Vec3r(d1, d2, d3);
    return _vtk_viewer->cameraViewDirection();
}

void VTKGraphicsScene::setCameraViewDirection(const Vec3r& view_dir)
{
    double dist = _vtk_viewer->renderer()->GetActiveCamera()->GetDistance();
    Vec3r pos = cameraPosition();
    Vec3r new_focal_point = pos + view_dir*dist;
    _vtk_viewer->renderer()->GetActiveCamera()->SetFocalPoint(new_focal_point[0], new_focal_point[1], new_focal_point[2]);
    _vtk_viewer->renderer()->ResetCameraClippingRange();
}

Vec3r VTKGraphicsScene::cameraUpDirection() const
{
    // double d1, d2, d3;
    // _vtk_viewer->renderer()->GetActiveCamera()->GetViewUp(d1, d2, d3);
    // return Vec3r(d1, d2, d3);
    return _vtk_viewer->cameraUpDirection();
}

void VTKGraphicsScene::setCameraUpDirection(const Vec3r& up_dir)
{
    _vtk_viewer->renderer()->GetActiveCamera()->SetViewUp(up_dir[0], up_dir[1], up_dir[2]);
    _vtk_viewer->renderer()->ResetCameraClippingRange();
}

Vec3r VTKGraphicsScene::cameraRightDirection() const
{
    return cameraViewDirection().cross(cameraUpDirection());
}

Vec3r VTKGraphicsScene::cameraPosition() const
{
    // double d1, d2, d3;
    // _vtk_viewer->renderer()->GetActiveCamera()->GetPosition(d1, d2, d3);
    // return Vec3r(d1, d2, d3);
    return _vtk_viewer->cameraPosition();
}

void VTKGraphicsScene::setCameraPosition(const Vec3r& pos)
{
    _vtk_viewer->renderer()->GetActiveCamera()->SetPosition(pos[0], pos[1], pos[2]);
    _vtk_viewer->renderer()->ResetCameraClippingRange();
}



} // namespace Graphics