#ifndef __OBJECT_RENDER_CONFIG_HPP
#define __OBJECT_RENDER_CONFIG_HPP

#include "config/Config.hpp"

#include <string>
#include <map>
#include <optional>

namespace Config
{

class ObjectRenderConfig : public Config
{
    public:
    enum class RenderType
    {
        PBR=0,
        PHONG,
        FLAT
    };

    static std::map<std::string, RenderType> RENDER_TYPE_MAP()
    {
        static std::map<std::string, RenderType> render_map{
            {"PBR", RenderType::PBR},
            {"Phong", RenderType::PHONG},
            {"Flat", RenderType::FLAT}
        };

        return render_map;
    }

    public:
    explicit ObjectRenderConfig()
        : Config()
    {

    }

    explicit ObjectRenderConfig(const YAML::Node& node)
        : Config(node)
    {
        _extractParameterWithOptions("render-type", node, _render_type, RENDER_TYPE_MAP());

        _extractParameter("orm-texture-filename", node, _orm_texture_filename);
        _extractParameter("normals-texture-filename", node, _normals_texture_filename);
        _extractParameter("base-color-texture-filename", node, _base_color_texture_filename);

        _extractParameter("metallic", node, _metallic);
        _extractParameter("roughness", node, _roughness);
        _extractParameter("opacity", node, _opacity);
        _extractParameter("color", node, _color);
        _extractParameter("cut-color", node, _cut_color);
        _extractParameter("colors", node, _colors);

        _extractParameter("smooth-normals", node, _smooth_normals);
        _extractParameter("draw-faces", node, _draw_faces);
        _extractParameter("draw-edges", node, _draw_edges);
        _extractParameter("draw-points", node, _draw_points);
    }

    explicit ObjectRenderConfig(
        RenderType render_type,
        std::optional<std::string> orm_texture_filename, std::optional<std::string> normals_texture_filename, std::optional<std::string> base_color_texture_filename,
        Real metallic, Real roughness, Real opacity, const Vec3r& color,
        bool smooth_normals, bool draw_faces, bool draw_edges, bool draw_points
    )
    {
        _render_type.value = render_type;

        _orm_texture_filename.value = orm_texture_filename;
        _normals_texture_filename.value = normals_texture_filename;
        _base_color_texture_filename.value = base_color_texture_filename;

        _metallic.value = metallic;
        _roughness.value = roughness;
        _opacity.value = opacity;
        _color.value = color;

        _smooth_normals.value = smooth_normals;
        _draw_faces.value = draw_faces;
        _draw_edges.value = draw_edges;
        _draw_points.value = draw_points;
    }

    RenderType renderType() const { return _render_type.value; }
    std::optional<std::string> ormTextureFilename() const { return _orm_texture_filename.value; }
    std::optional<std::string> normalsTextureFilename() const { return _normals_texture_filename.value; }
    std::optional<std::string> baseColorTextureFilename() const { return _base_color_texture_filename.value; }

    Real metallic() const { return _metallic.value; }
    Real roughness() const { return _roughness.value; }
    Real opacity() const { return _opacity.value; }
    std::optional<Vec3r> color() const { return _color.value; }
    std::optional<std::vector<Vec3r>> colors() const { return _colors.value; }
    std::optional<Vec3r> cutColor() const { return _cut_color.value; }

    bool smoothNormals() const { return _smooth_normals.value; }
    bool drawFaces() const { return _draw_faces.value; }
    bool drawEdges() const { return _draw_edges.value; }
    bool drawPoints() const { return _draw_points.value; }

    protected:
    ConfigParameter<RenderType> _render_type = ConfigParameter<RenderType>(RenderType::PHONG); 

    // PBR texture filenames
    ConfigParameter<std::optional<std::string>> _orm_texture_filename;
    ConfigParameter<std::optional<std::string>> _normals_texture_filename;
    ConfigParameter<std::optional<std::string>> _base_color_texture_filename;

    ConfigParameter<Real> _metallic = ConfigParameter<Real>(0.0);
    ConfigParameter<Real> _roughness = ConfigParameter<Real>(0.5);
    ConfigParameter<Real> _opacity = ConfigParameter<Real>(1.0);
    ConfigParameter<std::optional<Vec3r>> _color = ConfigParameter<std::optional<Vec3r>>();
    ConfigParameter<std::optional<Vec3r>> _cut_color = ConfigParameter<std::optional<Vec3r>>();
    ConfigParameter<std::optional<std::vector<Vec3r>>> _colors = ConfigParameter<std::optional<std::vector<Vec3r>>>();

    ConfigParameter<bool> _smooth_normals = ConfigParameter<bool>(true);
    ConfigParameter<bool> _draw_faces = ConfigParameter<bool>(true);
    ConfigParameter<bool> _draw_edges = ConfigParameter<bool>(false);
    ConfigParameter<bool> _draw_points = ConfigParameter<bool>(false);
};

} // namespace Config

#endif // __OBJECT_RENDER_CONFIG_HPP