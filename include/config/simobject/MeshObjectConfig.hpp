#ifndef __MESH_OBJECT_CONFIG_HPP
#define __MESH_OBJECT_CONFIG_HPP

#include "config/Config.hpp"

namespace Config
{

class MeshObjectConfig
{
    public:

    MeshObjectConfig(const YAML::Node& node)
    {
        Config::_extractParameter("filename", node, _filename);
        Config::_extractParameter("draw-points", node, _draw_points);
        Config::_extractParameter("draw-edges", node, _draw_edges);
        Config::_extractParameter("draw-faces", node, _draw_faces);
        Config::_extractParameter("color", node, _color);

        Config::_extractParameter("max-size", node, _max_size);
        Config::_extractParameter("size", node, _size);
        Config::_extractParameter("scaling", node, _scaling);
    }

    explicit MeshObjectConfig(const std::string& filename, const std::optional<Real>& max_size, const std::optional<Vec3r>& size, const std::optional<Vec3r>& scaling,
                             bool draw_points, bool draw_edges, bool draw_faces, const Vec4r& color)
    {
        _filename.value = filename;
        _max_size.value = max_size;
        _size.value = size;
        _scaling.value = scaling;
        _draw_points.value = draw_points;
        _draw_edges.value = draw_edges;
        _draw_faces.value = draw_faces;
        _color.value = color;
    }

    std::string filename() const { return _filename.value; }
    bool drawPoints() const { return _draw_points.value; }
    bool drawEdges() const { return _draw_edges.value; }
    bool drawFaces() const { return _draw_faces.value; }
    Vec4r color() const { return _color.value; }

    std::optional<Real> maxSize() const { return _max_size.value; }
    std::optional<Vec3r> size() const { return _size.value; }
    std::optional<Vec3r> scaling() const { return _scaling.value; }

    protected:
    ConfigParameter<std::string> _filename = ConfigParameter<std::string>("");  // this should probably be an optional
    ConfigParameter<bool> _draw_points = ConfigParameter<bool>(false);
    ConfigParameter<bool> _draw_edges = ConfigParameter<bool>(false);
    ConfigParameter<bool> _draw_faces = ConfigParameter<bool>(true);
    ConfigParameter<Vec4r> _color = ConfigParameter<Vec4r>(Vec4r(1.0, 1.0, 1.0, 1.0));

    ConfigParameter<std::optional<Real>> _max_size;
    ConfigParameter<std::optional<Vec3r>> _size;
    ConfigParameter<std::optional<Vec3r>> _scaling;

};

} // namespace Config

#endif // __MESH_OBJECT_CONFIG_HPP