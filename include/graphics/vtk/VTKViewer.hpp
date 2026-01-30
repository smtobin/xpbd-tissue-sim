#ifndef __VTK_VIEWER_HPP
#define __VTK_VIEWER_HPP

#include "graphics/Viewer.hpp"
#include "config/render/SimulationRenderConfig.hpp"

#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkOpenGLRenderer.h>
#include <vtkSmartPointer.h>
#include <vtkCallbackCommand.h>
#include <vtkTextActor.h>
#include <vtkTextProperty.h>
#include <vtkWindowToImageFilter.h>

#include <map>
#include <atomic>
#include <functional>
#include <mutex>

namespace Graphics
{

class VTKViewer : public Viewer
{
    // give CustomVTKInteractorStyle access to protected methods for processing
    // keyboard and mouse events
    friend class CustomVTKInteractorStyle;

    public:
    static void renderCallback(vtkObject* caller, long unsigned int event_id, void* client_data, void* call_data);
    const vtkSmartPointer<vtkOpenGLRenderer> renderer() const { return _renderer; }
    vtkSmartPointer<vtkOpenGLRenderer> renderer() { return _renderer; }
    void displayWindow() { _render_window->Render(); }
    void interactorStart() { _interactor->Start(); }

    void setPreRenderCallback(std::function<void()> cb) { _prerender_callback = std::move(cb); }

    explicit VTKViewer(const std::string& title, const Config::SimulationRenderConfig& render_config);

    virtual void update() override;

    virtual int width() const override;

    virtual int height() const override;

    /** Add text to the Viewer to be rendered
     * Simply creates a TextSpec according to the parameters passed in
     * Default parameters are used for convenience - at minimum, the name of the text and the text itself are required
     */
    virtual void addText(const std::string& name,
                 const std::string& text,
                 const float& x = 0.0f,
                 const float& y = 0.0f,
                 const float& font_size = 20.0f,
                 const TextAlignment& alignment = TextAlignment::LEFT,
                 const Font& font = Font::MAO,
                 const std::array<float,3>& color = {0.0f, 0.0f, 0.0f},
                 const float& line_spacing = 0.5f,
                 const bool& upper_left = true) override;

    /** Removes a text with the specified name
     * @param name : the name of the TextSpec to remove
     */
    virtual void removeText(const std::string& name) override;

    /** Modifies the text of a rendered TextSpec 
     * @param name : the name of the TextSpec to edit
     * @param new_text : the new text that the TextSpec should have
    */
    virtual void editText(const std::string& name, const std::string& new_text) override;

    /** Modifies the text, position, and font size of a rendered TextSpec 
     * @param name : the name of the TextSpec to edit
     * @param new_text : the new text that the TextSpec should have
     * @param new_x : the new x position of the text
     * @param new_y : the new y position of the text
     * @param new_font_size : the new font size of the text
    */
    virtual void editText(const std::string& name,
                  const std::string& new_text,
                  const float& new_x,
                  const float& new_y,
                  const float& new_font_size) override;

    Vec3r cameraViewDirection() const { return _cam_view_dir; }
    Vec3r cameraUpDirection() const { return _cam_up_dir; }
    Vec3r cameraPosition() const { return _cam_pos; }

    /** Copoies the image buffer to some external buffer. It is assumed that the external buffer has the appropriate amount of space.
     * This can be ensured by first querying width and height of the viewer.
     */
    void copyImageBufferToExternalBuffer(unsigned char* external_buffer);

    protected:
    /** Shared viewer behavior on keyboard events. */
    virtual void _processKeyboardEvent(SimulationInput::Key key, SimulationInput::KeyAction action, int modifiers) override;

    /** Shared viewer behavior on mouse button events. */
    virtual void _processMouseButtonEvent(SimulationInput::MouseButton button, SimulationInput::MouseAction action, int modifiers) override;

    /** Shared viewer behavior on mouse move events. */
    virtual void _processCursorMoveEvent(double x, double y);

    /** Shared viewer behavior on mouse scroll events. */
    virtual void _processScrollEvent(double dx, double dy);

    private:
    /** Set up rendering settings */
    void _setupRenderWindow(const Config::SimulationRenderConfig& render_config);

    private:
    vtkSmartPointer<vtkOpenGLRenderer> _renderer;
    vtkSmartPointer<vtkRenderWindow> _render_window;
    vtkSmartPointer<vtkRenderWindowInteractor> _interactor;

    /** Whether or not we are doing offscreen rendering. Set by the config. */
    bool _offscreen_rendering = false;

    /** Renders the current window to a vtkImage */
    vtkSmartPointer<vtkWindowToImageFilter> _window_to_image;
    /** Stores the pixel data */
    std::vector<unsigned char> _image_data;

    /** Guards the pixel data. The simulation thread and render thread may try to access the pixel data simultanesously. */
    std::mutex _image_data_mutex;


    std::map<std::string, vtkSmartPointer<vtkTextActor>> _text_actor_map;

    std::atomic<bool> _should_render = false;

    std::function<void()> _prerender_callback;

    /** Store current camera frame.
     * For some reason, repeated calling _vtk_viewer->renderer()->GetActiveCamera()->GetDirectionOfProjection() and things similar
     * can cause segfaults (probably due to thread safety things).
     * Instead, we store the camera frame here and update it when the camera view updates.
     */
    Vec3r _cam_view_dir;
    Vec3r _cam_up_dir;
    Vec3r _cam_pos;
};

} // namespace Graphics

#endif // __VTK_VIEWER_HPP