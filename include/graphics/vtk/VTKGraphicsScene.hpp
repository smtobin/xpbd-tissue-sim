#ifndef __VTK_GRAPHICS_SCENE_HPP
#define __VTK_GRAPHICS_SCENE_HPP

#include "graphics/GraphicsScene.hpp"
#include "graphics/vtk/VTKViewer.hpp"

#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkOpenGLRenderer.h>
#include <vtkSmartPointer.h>
#include <vtkCallbackCommand.h>

#include <vector>
#include <atomic>

namespace Sim
{
    class Simulation;
}

namespace Graphics
{

class VTKGraphicsScene : public GraphicsScene
{
    public:

    explicit VTKGraphicsScene(const std::string& name, const Config::SimulationRenderConfig& sim_render_config);

    virtual void init() override;

    virtual void update() override;

    void updateGraphicsBuffers();

    virtual int run() override;

    /** Creates a MeshGraphicsObject from a supplied MeshObject and adds it to the GraphicsScene
     * @param obj : the simulation Object to add to the GraphicsScene for visualization
     * @param obj_config : the ObjectRednerConfig that contains rendering parameters (e.g. coloring, draw points, textures, etc.)
     * @returns the index of the provided object in the _graphics_objects array (can be used to fetch it in the future)
     */
    virtual int addObject(const Sim::Object* obj, const Config::ObjectRenderConfig& obj_config) override;

    /** Sets the camera mode to Orthographic */
    virtual void setCameraOrthographic() override;

    /** Sets the camera mode to Perspective */
    virtual void setCameraPerspective() override;

    /** Sets the FOV of the camera (FOV in degrees). */
    virtual void setCameraFOV(Real fov) override;


    /** Gets the camera view direction. */
    virtual Vec3r cameraViewDirection() const override;
    /** Sets the camera view direction */
    virtual void setCameraViewDirection(const Vec3r& view_dir) override;

    /** Gets the camera up direction. */
    virtual Vec3r cameraUpDirection() const override;
    /** Sets the camera up direction */
    virtual void setCameraUpDirection(const Vec3r& up_dir) override;

    /** Gets the camera right direction. */
    virtual Vec3r cameraRightDirection() const override;
    /** Sets the camera right direction. */
    // virtual void setCameraRightDirection(const Vec3r& right_dir) const = 0;

    /** Gets the camera position. */
    virtual Vec3r cameraPosition() const override;
    /** Sets the camera position. */
    virtual void setCameraPosition(const Vec3r& position) override;

    private:
    /** Non-owning downcasted pointer for convenience in calling VTKViewer specific methods */
    VTKViewer* _vtk_viewer;

};

} // namespace Graphics

#endif // __VTK_GRAPHICS_SCENE_HPP