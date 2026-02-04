#include "graphics/vtk/VTKViewer.hpp"
#include "graphics/vtk/CustomVTKInteractorStyle.hpp"
#include "graphics/vtk/VTKCameraSyncCallback.hpp"

#include <vtkActor.h>
#include <vtkImageActor.h>
#include <vtkImageSliceMapper.h>
#include <vtkCamera.h>
#include <vtkCubeSource.h>
#include <vtkSphereSource.h>
#include <vtkPlaneSource.h>
#include <vtkHDRReader.h>
#include <vtkImageFlip.h>
#include <vtkImageReader2.h>
#include <vtkImageReader2Factory.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkLight.h>
#include <vtkNamedColors.h>
#include <vtkOpenGLRenderer.h>
#include <vtkOpenGLTexture.h>
#include <vtkPBRIrradianceTexture.h>
#include <vtkPNGReader.h>
#include <vtkPolyDataMapper.h>
#include <vtkPolyDataTangents.h>
#include <vtkProperty.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkSkybox.h>
#include <vtkTexture.h>
#include <vtkTriangleFilter.h>
#include <vtkAxesActor.h>
#include <vtkOrientationMarkerWidget.h>

#include <vtkSequencePass.h>
#include <vtkShadowMapBakerPass.h>
#include <vtkShadowMapPass.h>
#include <vtkCameraPass.h>
#include <vtkRenderPassCollection.h>
#include <vtkRenderStepsPass.h>
#include <vtkToneMappingPass.h>
#include <vtkLightsPass.h>
#include <vtkOpaquePass.h>
#include <vtkTranslucentPass.h>

#include <vtkAutoInit.h>
VTK_MODULE_INIT(vtkInteractionStyle);
VTK_MODULE_INIT(vtkRenderingFreeType);
VTK_MODULE_INIT(vtkRenderingOpenGL2);

#include <filesystem>

namespace Graphics
{

VTKViewer::VTKViewer(const std::string& title, const Config::SimulationRenderConfig& render_config)
    : Viewer(title)
{
    _setupRenderWindow(render_config);

    _offscreen_rendering = render_config.offscreenRendering();
    if (_offscreen_rendering)
    {
        _window_to_image = vtkSmartPointer<vtkWindowToImageFilter>::New();
        _window_to_image->SetInput(_render_window);
        _window_to_image->ReadFrontBufferOff();
        _window_to_image->SetInputBufferTypeToRGB();
        _image_data.resize(render_config.windowHeight() * render_config.windowWidth() * 3, 0);
    }

    _circle_crop = render_config.circleCrop();
}

void VTKViewer::_setupRenderWindow(const Config::SimulationRenderConfig& render_config)
{
    // create renderer for actors in the scene
    _renderer = vtkSmartPointer<vtkOpenGLRenderer>::New();

    _renderer->SetBackground(render_config.background()[0], render_config.background()[1], render_config.background()[2]);
    _renderer->SetAutomaticLightCreation(false);

    //////////////////////////////////////////////////////////
    // Create HDR lighting (if specified in the config)
    /////////////////////////////////////////////////////////

    std::optional<std::string> hdr_filename = render_config.hdrImageFilename();
    if (hdr_filename.has_value())
    {
        vtkNew<vtkTexture> hdr_texture;
        vtkNew<vtkHDRReader> reader;
        reader->SetFileName(hdr_filename.value().c_str());
        hdr_texture->SetInputConnection(reader->GetOutputPort());
        hdr_texture->SetColorModeToDirectScalars();
        hdr_texture->MipmapOn();
        hdr_texture->InterpolateOn();

        if (render_config.createSkybox())
        {
            vtkNew<vtkSkybox> skybox;
            skybox->SetTexture(hdr_texture);
            skybox->SetFloorRight(0,0,1);
            skybox->SetProjection(vtkSkybox::Sphere);
            _renderer->AddActor(skybox);
        }

        _renderer->UseImageBasedLightingOn();
        _renderer->UseSphericalHarmonicsOn();
        _renderer->SetEnvironmentTexture(hdr_texture, false);
    }
    

    ////////////////////////////////////////////////////////
    // Add lights
    ////////////////////////////////////////////////////////

    // vtkNew<vtkLight> light;
    // light->SetLightTypeToSceneLight();
    // light->SetPositional(true);
    // light->SetPosition(0.0, 10, 0);
    // light->SetFocalPoint(0,0,0);
    // light->SetConeAngle(90);
    // light->SetAttenuationValues(1,0,0);
    // light->SetColor(1.0, 1.0, 1.0);
    // light->SetIntensity(1.0);
    // _renderer->AddLight(light);


    ///////////////////////////////////////////////////////
    // Set up ground plane
    ///////////////////////////////////////////////////////

    // vtkNew<vtkPlaneSource> plane;
    // plane->SetCenter(0.0, 0.0, 0.0);
    // plane->SetNormal(0.0, 1.0, 0.0);
    // // plane->SetResolution(10, 10);
    // plane->Update();

    // vtkNew<vtkPolyDataMapper> plane_mapper;
    // plane_mapper->SetInputData(plane->GetOutput());

    // vtkNew<vtkPNGReader> plane_tex_reader;
    // plane_tex_reader->SetFileName("../resource/textures/ground_plane_texture.png");
    // vtkNew<vtkTexture> plane_color;
    // plane_color->UseSRGBColorSpaceOn();
    // plane_color->SetMipmap(true);
    // plane_color->InterpolateOn();
    // plane_color->SetInputConnection(plane_tex_reader->GetOutputPort());

    // vtkNew<vtkActor> plane_actor;
    // plane_actor->SetMapper(plane_mapper);
    // // plane_actor->GetProperty()->SetColor(0.9, 0.9, 0.9);
    // plane_actor->GetProperty()->SetInterpolationToPBR();
    // plane_actor->GetProperty()->SetMetallic(0.0);
    // plane_actor->GetProperty()->SetRoughness(0.3);
    // plane_actor->SetScale(0.1,1.0,0.1);

    // plane_actor->GetProperty()->SetBaseColorTexture(plane_color);
    // _renderer->AddActor(plane_actor);

    //////////////////////////////////////////////////////
    // Create the render window and interactor
    //////////////////////////////////////////////////////
    _render_window = vtkSmartPointer<vtkRenderWindow>::New();
    _render_window->SetNumberOfLayers(3);
    _render_window->AddRenderer(_renderer);
    _render_window->SetSize(render_config.windowWidth(), render_config.windowHeight());
    _render_window->SetWindowName(_name.c_str());
    

    _interactor = vtkSmartPointer<vtkRenderWindowInteractor>::New();
    vtkNew<CustomVTKInteractorStyle> style;
    style->registerViewer(this);
    _interactor->SetInteractorStyle(style);
    _interactor->SetRenderWindow(_render_window);


    // create a renderer for the circle mask (when applicable)
    if (render_config.circleCrop())
    {
        vtkNew<vtkRenderer> mask_renderer;
        mask_renderer->SetInteractive(0);
        mask_renderer->SetViewport(0, 0, 1, 1);
        mask_renderer->SetLayer(1);
        mask_renderer->SetBackground(0,0,0);
        mask_renderer->SetBackgroundAlpha(0.0);

        vtkSmartPointer<vtkImageData> circle_mask = _createCircleMask(render_config.windowWidth(), render_config.windowHeight());

        vtkNew<vtkImageSliceMapper> slice_mapper;
        slice_mapper->SetInputData(circle_mask);
        slice_mapper->SetOrientationToZ();
        slice_mapper->SetSliceNumber(0);
        slice_mapper->BorderOff();

        vtkNew<vtkImageSlice> slice;
        slice->SetMapper(slice_mapper);
        slice->SetPosition(0.0, 0.0, 0.0);

        vtkCamera* cam = mask_renderer->GetActiveCamera();
        cam->ParallelProjectionOn();

        int w = render_config.windowWidth();
        int h = render_config.windowHeight();
        cam->SetFocalPoint(w/2.0, h/2.0, 0.0);
        cam->SetPosition(w/2.0, h/2.0, 1.0);
        cam->SetParallelScale(h/2.0);

        mask_renderer->AddViewProp(slice);
        _render_window->AddRenderer(mask_renderer);
    }


    // Create a second renderer for the axes overlay
    vtkNew<vtkRenderer> axes_renderer;
    axes_renderer->SetViewport(0.0, 0.0, 0.2, 0.2);  // Top-right corner
    axes_renderer->SetInteractive(0);  // Don't respond to mouse
    axes_renderer->SetLayer(2);        // Render on top

    // Clear background is transparent
    axes_renderer->SetBackground(0, 0, 0);
    axes_renderer->SetBackgroundAlpha(0.0);

    // Add axes to the overlay renderer
    vtkNew<vtkAxesActor> axes;
    axes->SetTotalLength(1.0, 1.0, 1.0);
    axes->SetShaftType(vtkAxesActor::CYLINDER_SHAFT);
    axes->SetTipType(vtkAxesActor::CONE_TIP);
    axes_renderer->AddActor(axes);

    // Add both renderers to the render window
    _render_window->AddRenderer(axes_renderer);  // Axes overlay (layer 2)

    // Sync cameras so axes rotate with main view
    vtkNew<CameraSyncCallback> callback;
    callback->axesCamera = axes_renderer->GetActiveCamera();
    callback->cam_up_dir = &_cam_up_dir;
    callback->cam_view_dir = &_cam_view_dir;
    callback->cam_pos = &_cam_pos;
    _renderer->GetActiveCamera()->AddObserver(vtkCommand::ModifiedEvent, callback);

    /////////////////////////////////////////////////////////
    // Initialize camera state matrix
    ////////////////////////////////////////////////////////
    _camera_state.pos = Eigen::Map<Vec3r>(_renderer->GetActiveCamera()->GetPosition());
    Vec3r focal_point = Eigen::Map<Vec3r>(_renderer->GetActiveCamera()->GetFocalPoint());
    double dist = _renderer->GetActiveCamera()->GetDistance();
    _camera_state.view_dir = (focal_point - _camera_state.pos)/dist;
    _camera_state.up_dir = Eigen::Map<Vec3r>(_renderer->GetActiveCamera()->GetViewUp());
    _camera_state.hfov = _renderer->GetActiveCamera()->GetViewAngle();
    _camera_state.is_orthographic = _renderer->GetActiveCamera()->GetParallelProjection();
    
    /////////////////////////////////////////////////////////
    // Create the rendering passes and settings
    ////////////////////////////////////////////////////////
    // Enable proper transparency rendering
    _renderer->SetUseDepthPeeling(false);
    _renderer->SetMaximumNumberOfPeels(4);
    _render_window->SetAlphaBitPlanes(true);
    _render_window->SetMultiSamples(0);
    // _render_window->SetMultiSamples(10);
    
    vtkNew<vtkSequencePass> seqP;
    vtkNew<vtkOpaquePass> opaqueP;
    vtkNew<vtkTranslucentPass> translucentP;
    vtkNew<vtkLightsPass> lightsP;

    vtkNew<vtkShadowMapPass> shadows;
    shadows->GetShadowMapBakerPass()->SetResolution(1024);

    vtkNew<vtkRenderPassCollection> passes;
    passes->AddItem(lightsP);
    passes->AddItem(shadows);
    passes->AddItem(opaqueP);
    passes->AddItem(translucentP);
    passes->AddItem(shadows->GetShadowMapBakerPass());
    
    seqP->SetPasses(passes);

    vtkNew<vtkCameraPass> cameraP;
    cameraP->SetDelegatePass(seqP);

    vtkNew<vtkToneMappingPass> toneMappingP;
    toneMappingP->SetToneMappingType(vtkToneMappingPass::GenericFilmic);
    toneMappingP->SetGenericFilmicDefaultPresets();
    toneMappingP->SetDelegatePass(cameraP);
    toneMappingP->SetExposure(render_config.exposure());

    _renderer->SetPass(toneMappingP);

    vtkNew<vtkCallbackCommand> render_callback;
    render_callback->SetCallback(VTKViewer::renderCallback);
    render_callback->SetClientData(this);
    _interactor->Initialize();
    _interactor->AddObserver(vtkCommand::TimerEvent, render_callback);
    _interactor->CreateRepeatingTimer(5);
}

vtkSmartPointer<vtkImageData> VTKViewer::_createCircleMask(int width, int height)
{
    vtkSmartPointer<vtkImageData> image = vtkSmartPointer<vtkImageData>::New();
    image->SetDimensions(width, height, 1);
    image->SetSpacing(1.0, 1.0, 1.0);
    image->SetOrigin(0.0, 0.0, 0.0);
    image->AllocateScalars(VTK_UNSIGNED_CHAR, 4);
    
    int centerX = width / 2;
    int centerY = height / 2;
    double radius = std::min(width, height) / 2.0;
    
    for (int y = 0; y < height; y++)
    {
        for (int x = 0; x < width; x++)
        {
            double dx = x - centerX;
            double dy = y - centerY;
            double distance = sqrt(dx*dx + dy*dy);
            
            unsigned char* pixel = static_cast<unsigned char*>(image->GetScalarPointer(x, y, 0));
            pixel[0] = 0;   // R
            pixel[1] = 0;   // G
            pixel[2] = 0;   // B
            
            // Alpha channel: 0 inside circle, 255 outside
            if (distance <= radius)
            {
                pixel[3] = 0;  // Transparent
            }
            else
            {
                pixel[3] = 255;  // Opaque black
            }
        }
    }
    
    return image;
}

void VTKViewer::renderCallback(vtkObject* /*caller*/, long unsigned int /*event_id*/, void* client_data, void* /*call_data*/)
{
    
    VTKViewer* viewer = static_cast<VTKViewer*>(client_data);
    if (viewer->_should_render.exchange(false))
    {
        // call the prerender callback (update the graphics objects for all the sim objects)
        if (viewer->_prerender_callback)
            viewer->_prerender_callback();

        // update the camera
        viewer->_updateCamera();
            
        viewer->_render_window->Render();

        if (viewer->_offscreen_rendering)
        {
            // update the window -> image object to render offscreen
            viewer->_window_to_image->Modified();
            viewer->_window_to_image->Update();
            // get the latest rendered frame
            vtkImageData* latest_frame = viewer->_window_to_image->GetOutput();
            int* dims = latest_frame->GetDimensions();
            size_t num_bytes = dims[0] * dims[1] * latest_frame->GetNumberOfScalarComponents();     // width * height * num_channels

            unsigned char* pixels = static_cast<unsigned char*>(latest_frame->GetScalarPointer()); 
            // copy to buffer
            {
                std::lock_guard<std::mutex> lock(viewer->_image_data_mutex);
                
                // make sure the image buffer has the appropriate amount of space
                if (viewer->_image_data.size() != num_bytes)
                    viewer->_image_data.resize(num_bytes);
                
                std::memcpy(viewer->_image_data.data(), pixels, num_bytes);
            }
        }
    }
}

void VTKViewer::_updateCamera()
{
    VTKCameraState new_state;
    {
        std::lock_guard<std::mutex> lock(_camera_state_mutex);
        new_state = _camera_state;
        _camera_state.updated = false;
    }

    if (!new_state.updated)
        return;

    // set position
    _renderer->GetActiveCamera()->SetPosition(new_state.pos[0], new_state.pos[1], new_state.pos[2]);
    _renderer->ResetCameraClippingRange();

    // set view angle
    _renderer->GetActiveCamera()->UseHorizontalViewAngleOn();
    _renderer->GetActiveCamera()->SetViewAngle(new_state.hfov);
    _renderer->ResetCameraClippingRange();

    // set view direction
    double dist = _renderer->GetActiveCamera()->GetDistance();
    Vec3r new_focal_point = new_state.pos + new_state.view_dir*dist;
    _renderer->GetActiveCamera()->SetFocalPoint(new_focal_point[0], new_focal_point[1], new_focal_point[2]);
    _renderer->ResetCameraClippingRange();

    // set up direction
    _renderer->GetActiveCamera()->SetViewUp(new_state.up_dir[0], new_state.up_dir[1], new_state.up_dir[2]);
    _renderer->ResetCameraClippingRange();

    // set camera orthographic
    if (new_state.is_orthographic)
        _renderer->GetActiveCamera()->SetParallelProjection(true);
    else
        _renderer->GetActiveCamera()->SetParallelProjection(false);
}

void VTKViewer::setCameraOrthographic()
{
    // _renderer->GetActiveCamera()->SetParallelProjection(true);
    std::lock_guard<std::mutex> lock(_camera_state_mutex);
    _camera_state.is_orthographic = true;
    _camera_state.updated = true;
}

void VTKViewer::setCameraPerspective()
{
    // _renderer->GetActiveCamera()->SetParallelProjection(false);
    std::lock_guard<std::mutex> lock(_camera_state_mutex);
    _camera_state.is_orthographic = false;
    _camera_state.updated = true;
}

void VTKViewer::setCameraFOV(Real fov)
{
    // _renderer->GetActiveCamera()->UseHorizontalViewAngleOn();
    // _renderer->GetActiveCamera()->SetViewAngle(fov);
    // _renderer->ResetCameraClippingRange();
    std::lock_guard<std::mutex> lock(_camera_state_mutex);
    _camera_state.hfov = fov;
    _camera_state.updated = true;
}

Vec3r VTKViewer::cameraViewDirection() const
{
    return _camera_state.view_dir;
}

void VTKViewer::setCameraViewDirection(const Vec3r& view_dir)
{
    // double dist = _vtk_viewer->renderer()->GetActiveCamera()->GetDistance();
    // Vec3r pos = cameraPosition();
    // Vec3r new_focal_point = pos + view_dir*dist;
    // _vtk_viewer->renderer()->GetActiveCamera()->SetFocalPoint(new_focal_point[0], new_focal_point[1], new_focal_point[2]);
    // _vtk_viewer->renderer()->ResetCameraClippingRange();

    std::lock_guard<std::mutex> lock(_camera_state_mutex);
    _camera_state.view_dir = view_dir;
    _camera_state.updated = true;
}

Vec3r VTKViewer::cameraUpDirection() const
{
    return _camera_state.up_dir;
}

void VTKViewer::setCameraUpDirection(const Vec3r& up_dir)
{
    // _vtk_viewer->renderer()->GetActiveCamera()->SetViewUp(up_dir[0], up_dir[1], up_dir[2]);
    // _vtk_viewer->renderer()->ResetCameraClippingRange();
    std::lock_guard<std::mutex> lock(_camera_state_mutex);
    _camera_state.up_dir = up_dir;
    _camera_state.updated = true;
}

Vec3r VTKViewer::cameraRightDirection() const
{
    return cameraViewDirection().cross(cameraUpDirection());
}

Vec3r VTKViewer::cameraPosition() const
{
    return _camera_state.pos;
}

void VTKViewer::setCameraPosition(const Vec3r& pos)
{
    // _vtk_viewer->renderer()->GetActiveCamera()->SetPosition(pos[0], pos[1], pos[2]);
    // _vtk_viewer->renderer()->ResetCameraClippingRange();
    std::lock_guard<std::mutex> lock(_camera_state_mutex);
    _camera_state.pos = pos;
    _camera_state.updated = true;
}

void VTKViewer::update()
{
    _should_render.store(true);
}

int VTKViewer::width() const
{
    return _render_window->GetSize()[0];
}

int VTKViewer::height() const
{
    return _render_window->GetSize()[1];
}

void VTKViewer::addText(const std::string& name,
                const std::string& text,
                const float& x,
                const float& y,
                const float& font_size,
                const TextAlignment& alignment,
                const Font& font,
                const std::array<float,3>& color,
                const float& line_spacing,
                const bool& upper_left)
{
    Viewer::addText(name, text, x, y, font_size, alignment, font, color, line_spacing, upper_left);

    // add new text actor
    vtkNew<vtkTextActor> text_actor;
    text_actor->SetInput(text.c_str());
    text_actor->SetDisplayPosition(x, y);
    text_actor->GetTextProperty()->SetFontFamilyToArial();
    text_actor->GetTextProperty()->SetFontSize(font_size);
    text_actor->GetTextProperty()->SetColor(color[0], color[1], color[2]);
    
    _text_actor_map[name] = text_actor;

    _renderer->AddActor(text_actor);
}

void VTKViewer::removeText(const std::string& name)
{
    Viewer::removeText(name);

    auto it = _text_actor_map.find(name);
    if (it != _text_actor_map.end())
        _renderer->RemoveActor(it->second);
    
    _text_actor_map.erase(name);
}

void VTKViewer::editText(const std::string& name, const std::string& new_text)
{
    Viewer::editText(name, new_text);

    auto it = _text_actor_map.find(name);
    if (it != _text_actor_map.end())
    {
        vtkSmartPointer<vtkTextActor> txt = it->second;
        txt->SetInput(new_text.c_str());
    }
}

void VTKViewer::editText(const std::string& name,
                const std::string& new_text,
                const float& new_x,
                const float& new_y,
                const float& new_font_size)
{
    Viewer::editText(name, new_text, new_x, new_y, new_font_size);
    
    auto it = _text_actor_map.find(name);
    if (it != _text_actor_map.end())
    {
        vtkSmartPointer<vtkTextActor> txt = it->second;
        txt->SetInput(new_text.c_str());
        txt->SetDisplayPosition(new_x, new_y);
        txt->GetTextProperty()->SetFontSize(new_font_size);
    }
}

void VTKViewer::copyImageBufferToExternalBuffer(unsigned char* external_buffer)
{
    if (!_offscreen_rendering)
    {
        std::cerr << KRED << BOLD << "FATAL: " << RST << KRED << "Offscreen rendering must be enabled to copy image buffer." << std::endl;
        throw std::runtime_error("Offscreen rendering not enabled");
        return;
    }

    std::lock_guard<std::mutex> lock(_image_data_mutex);
    std::memcpy(external_buffer, _image_data.data(), _image_data.size());
}

void VTKViewer::_processKeyboardEvent(SimulationInput::Key key, SimulationInput::KeyAction action, int modifiers)
{
    Viewer::_processKeyboardEvent(key, action, modifiers);
}

void VTKViewer::_processMouseButtonEvent(SimulationInput::MouseButton button, SimulationInput::MouseAction action, int modifiers)
{
    Viewer::_processMouseButtonEvent(button, action, modifiers);
}

void VTKViewer::_processCursorMoveEvent(double x, double y)
{
    Viewer::_processCursorMoveEvent(x, y);
}

void VTKViewer::_processScrollEvent(double dx, double dy)
{
    Viewer::_processScrollEvent(dx, dy);
}

} // namespace Graphics