#ifndef __SIMULATION_RENDER_CONFIG_HPP
#define __SIMULATION_RENDER_CONFIG_HPP

#include "config/Config.hpp"

#include <optional>

namespace Config
{

class SimulationRenderConfig : public Config
{
    public:
    explicit SimulationRenderConfig()
        : Config()
    {
        
    }

    explicit SimulationRenderConfig(const YAML::Node& node)
        : Config(node)
    {
        _extractParameter("hdr-image-filename", node, _hdr_image_filename);
        _extractParameter("create-skybox", node, _create_skybox);
        _extractParameter("exposure", node, _exposure);

        _extractParameter("offscreen-rendering", node, _offscreen_rendering);

        _extractParameter("window-width", node, _window_width);
        _extractParameter("window-height", node, _window_height);
    }

    const std::optional<std::string>& hdrImageFilename() const { return _hdr_image_filename.value; }
    bool createSkybox() const { return _create_skybox.value; }
    Real exposure() const { return _exposure.value; }

    bool offscreenRendering() const { return _offscreen_rendering.value; }

    int windowWidth() const { return _window_width.value; }
    int windowHeight() const { return _window_height.value; }

    protected:
    ConfigParameter<std::optional<std::string>> _hdr_image_filename;
    ConfigParameter<bool> _create_skybox = ConfigParameter<bool>(true);
    ConfigParameter<Real> _exposure = ConfigParameter<Real>(0.5);

    ConfigParameter<bool> _offscreen_rendering = ConfigParameter<bool>(false);

    ConfigParameter<int> _window_width = ConfigParameter<int>(600);
    ConfigParameter<int> _window_height = ConfigParameter<int>(600);

};

} // namespace Config

#endif // __SIMULATION_RENDER_CONFIG_HPP