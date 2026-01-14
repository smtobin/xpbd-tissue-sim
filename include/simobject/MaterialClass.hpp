#ifndef __MATERIAL_CLASS_HPP
#define __MATERIAL_CLASS_HPP

#include "simobject/ElasticMaterial.hpp"
#include "config/simobject/MaterialClassConfig.hpp"

namespace Sim
{

class MaterialClass
{
public:
    using ConfigType = Config::MaterialClassConfig;

    explicit MaterialClass(const ConfigType* config)
        : _material(&config->materialConfig()), _render_config(config->renderConfig()),
          _label(config->label())
    {
    }

    const ElasticMaterial& material() const { return _material; }
    const Config::ObjectRenderConfig& renderConfig() const { return _render_config; }

    const std::string& label() const { return _label; }
    const std::string& name() const { return _material.name(); }

private:
    /** The elastic material for this class */
    ElasticMaterial _material;

    /** Rendering settings */
    Config::ObjectRenderConfig _render_config;

    /** A separate label for this material. This can be used to label different anatomy, or to lump different materials under the same label.
     * E.g. when partial view point clouds for trachea/tumor are generated, they look at the label, not the name, of the material to see which part
     *   corresponds to the trachea and which part corresponds to the tumor.
     */
    std::string _label;

};

} // namespace Sim

#endif // __MATERIAL_CLASS_HPP