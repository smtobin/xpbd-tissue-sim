#include "graphics/vtk/VTKViewer.hpp"
#include "graphics/vtk/CustomVTKInteractorStyle.hpp"
#include "graphics/vtk/VTKCameraSyncCallback.hpp"

#include <vtkActor.h>
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
}

void VTKViewer::_setupRenderWindow(const Config::SimulationRenderConfig& render_config)
{
    // create renderer for actors in the scene
    _renderer = vtkSmartPointer<vtkOpenGLRenderer>::New();

    _renderer->SetBackground(0.3, 0.3, 0.3);
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
    _render_window->SetNumberOfLayers(2);
    _render_window->AddRenderer(_renderer);
    _render_window->SetSize(render_config.windowWidth(), render_config.windowHeight());
    _render_window->SetWindowName(_name.c_str());
    

    _interactor = vtkSmartPointer<vtkRenderWindowInteractor>::New();
    vtkNew<CustomVTKInteractorStyle> style;
    style->registerViewer(this);
    _interactor->SetInteractorStyle(style);
    _interactor->SetRenderWindow(_render_window);

    // Create a second renderer for the axes overlay
    vtkNew<vtkRenderer> axes_renderer;
    axes_renderer->SetViewport(0.0, 0.0, 0.2, 0.2);  // Top-right corner
    axes_renderer->SetInteractive(0);  // Don't respond to mouse
    axes_renderer->SetLayer(1);        // Render on top

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
    _render_window->AddRenderer(axes_renderer);  // Axes overlay (layer 1)

    // Sync cameras so axes rotate with main view
    vtkNew<CameraSyncCallback> callback;
    callback->axesCamera = axes_renderer->GetActiveCamera();
    callback->cam_up_dir = &_cam_up_dir;
    callback->cam_view_dir = &_cam_view_dir;
    callback->cam_pos = &_cam_pos;
    _renderer->GetActiveCamera()->AddObserver(vtkCommand::ModifiedEvent, callback);
    
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

void VTKViewer::renderCallback(vtkObject* /*caller*/, long unsigned int /*event_id*/, void* client_data, void* /*call_data*/)
{
    
    VTKViewer* viewer = static_cast<VTKViewer*>(client_data);
    if (viewer->_should_render.exchange(false))
    {
        viewer->_render_window->Render();
    }
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