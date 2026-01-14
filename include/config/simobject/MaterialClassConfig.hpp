#ifndef __MATERIAL_CLASS_CONFIG
#define __MATERIAL_CLASS_CONFIG

#include "config/Config.hpp"
#include "config/simobject/ElasticMaterialConfig.hpp"
#include "config/render/ObjectRenderConfig.hpp"

namespace Config
{

class MaterialClassConfig : public Config
{
public:
    explicit MaterialClassConfig(const YAML::Node& node)
        : Config(node), _material_config(node), _render_config(node)
    {

        // extract the label for this material
        // if none provided, use the name as the label
        _extractParameter("label", node, _label);
        if (_label.value == "")
            _label.value = _name.value;
    }
    
    explicit MaterialClassConfig()
        : Config(), _material_config(), _render_config()
    {}

    explicit MaterialClassConfig(const std::string& name)
        : Config(name), _material_config(name), _render_config()
    {
        _label.value = _name.value;
    }

    explicit MaterialClassConfig(const std::string& label,
                                 const ElasticMaterialConfig& material_config, const ObjectRenderConfig& render_config)
        : _material_config(material_config), _render_config(render_config)
    {
        _label.value = label;
    }

    const ElasticMaterialConfig& materialConfig() const { return _material_config; }
    const ObjectRenderConfig& renderConfig() const { return _render_config; }

    std::string label() const { return _label.value; }

protected:
    ElasticMaterialConfig _material_config;
    ObjectRenderConfig _render_config;

    ConfigParameter<std::string> _label = ConfigParameter<std::string>("");
};

} // namespace Config


#endif // __MATERIAL_CLASS_CONFIG