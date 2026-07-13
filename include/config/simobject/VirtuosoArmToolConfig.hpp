#ifndef __VIRTUOSO_ARM_TOOL_CONFIG_HPP
#define __VIRTUOSO_ARM_TOOL_CONFIG_HPP

#include "config/simobject/ObjectConfig.hpp"

namespace Config
{

class VirtuosoArmToolConfig : public ObjectConfig
{

public:
    explicit VirtuosoArmToolConfig()
        : ObjectConfig()
    {}

    explicit VirtuosoArmToolConfig(const YAML::Node& node)
        : ObjectConfig(node)
    {
        _extractParameter("extra-tool-offset", node, _extra_tool_offset);
    }

    Real extraToolOffset() const { return _extra_tool_offset.value; }

protected:
    ConfigParameter<Real> _extra_tool_offset = ConfigParameter<Real>(0.0);

};

} // namespace Config

#endif // __VIRTUOSO_ARM_TOOL_CONFIG_HPP