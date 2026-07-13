#ifndef __VIRTUOSO_ARM_CONFIG_HPP
#define __VIRTUOSO_ARM_CONFIG_HPP

#include "config/simobject/ObjectConfig.hpp"
#include "config/simobject/VirtuosoArmToolConfig.hpp"

#include "simobject/VirtuosoArm.hpp"

namespace Sim
{
    class VirtuosoArm;
}

namespace Config
{

class VirtuosoArmConfig : public ObjectConfig
{
    public:
    using ObjectType = Sim::VirtuosoArm;

    public:
    /** Static predifined options for the tool type. Maps strings to the ToolType enum. */
    static std::map<std::string, Sim::VirtuosoArm::ToolType> TOOL_TYPE_OPTIONS()
    {
        static std::map<std::string, Sim::VirtuosoArm::ToolType> tool_type_options{
            {"none", Sim::VirtuosoArm::ToolType::NONE},
            {"palpation", Sim::VirtuosoArm::ToolType::PALPATION},
            {"spatula", Sim::VirtuosoArm::ToolType::SPATULA},
            {"grasper", Sim::VirtuosoArm::ToolType::GRASPER},
            {"cautery", Sim::VirtuosoArm::ToolType::CAUTERY}
        };
        return tool_type_options;
    }
    /** Static predefined options for the cutting model. Maps strings to the CuttingModel enum. */
    static std::map<std::string, Sim::VirtuosoArm::CuttingModel> CUTTING_MODEL_OPTIONS()
    {
        static std::map<std::string, Sim::VirtuosoArm::CuttingModel> cutting_model_options{
            {"none", Sim::VirtuosoArm::CuttingModel::NONE},
            {"instant", Sim::VirtuosoArm::CuttingModel::INSTANT},
            {"timer", Sim::VirtuosoArm::CuttingModel::TIMER},
            {"thermal", Sim::VirtuosoArm::CuttingModel::THERMAL}
        };  

        return cutting_model_options;
    }

    explicit VirtuosoArmConfig()
        : ObjectConfig()
    {}

    explicit VirtuosoArmConfig(const YAML::Node& node)
        : ObjectConfig(node), _tool_config(node)
    {
        _extractParameter("inner-tube-outer-diameter", node, _it_outer_diameter);
        _extractParameter("inner-tube-inner-diameter", node, _it_inner_diameter);
        _extractParameter("inner-tube-translation", node, _it_initial_translation);
        _extractParameter("inner-tube-rotation", node, _it_initial_rotation);

        _extractParameter("outer-tube-outer-diameter", node, _ot_outer_diameter);
        _extractParameter("outer-tube-inner-diameter", node, _ot_inner_diameter);
        _extractParameter("outer-tube-radius-of-curvature", node, _ot_r_curvature);
        _extractParameter("outer-tube-translation", node, _ot_initial_translation);
        _extractParameter("outer-tube-rotation", node, _ot_initial_rotation);
        _extractParameter("outer-tube-distal-straight-length", node, _ot_distal_straight_length);

        _extractParameterWithOptions("tool-type", node, _tool_type, TOOL_TYPE_OPTIONS());
        _extractParameterWithOptions("cutting-model", node, _cutting_model, CUTTING_MODEL_OPTIONS());
        _extractParameter("time-threshold", node, _cutting_model_time_threshold);
        _extractParameter("tool-tube-length", node, _tool_tube_length);

        _extractParameter("base-position", node, _base_initial_position);
        _extractParameter("base-rotation", node, _base_initial_rotation);

        _extractParameter("max-ot-translation-speed", node, _max_ot_translation_speed);
        _extractParameter("max-it-translation-speed", node, _max_it_translation_speed);
        _extractParameter("max-ot-rotation-speed", node, _max_ot_rotation_speed);
        _extractParameter("max-it-rotation-speed", node, _max_it_rotation_speed);
    }

    explicit VirtuosoArmConfig( const std::string& name, const std::string& material_class, 
        const Vec3r& initial_pos, const Vec3r& initial_rot, const Vec3r& initial_velocity, bool collisions, bool graphics_only,
        Real ot_outer_dia, Real ot_inner_dia, Real ot_r_curve, Real ot_d_s_length, Real it_outer_dia, Real it_inner_dia,
        Real ot_rot, Real ot_trans, Real it_rot, Real it_trans,
        Sim::VirtuosoArm::ToolType tool_type, Sim::VirtuosoArm::CuttingModel cutting_model, Real tool_tube_length,
        const ObjectRenderConfig& render_config
    )
        : ObjectConfig(name, material_class, initial_pos, initial_rot, initial_velocity, collisions, graphics_only, render_config)
    {
        _ot_outer_diameter.value = ot_outer_dia;
        _ot_inner_diameter.value = ot_inner_dia;
        _ot_r_curvature.value = ot_r_curve;
        _ot_distal_straight_length.value = ot_d_s_length;
        _it_outer_diameter.value = it_outer_dia;
        _it_inner_diameter.value = it_inner_dia;

        _ot_initial_rotation.value = ot_rot;
        _ot_initial_translation.value = ot_trans;
        _it_initial_rotation.value = it_rot;
        _it_initial_translation.value = it_trans;

        _tool_type.value = tool_type;
        _cutting_model.value = cutting_model;
        _tool_tube_length.value = tool_tube_length; 

        _base_initial_position.value = initial_pos;
        _base_initial_rotation.value = initial_rot;
    }

    std::unique_ptr<ObjectType> createObject(const Sim::Simulation* sim) const;

    // Getters and setters
    Real innerTubeOuterDiameter() const { return _it_outer_diameter.value; }
    Real innerTubeInnerDiameter() const { return _it_inner_diameter.value; }
    Real innerTubeInitialTranslation() const { return _it_initial_translation.value; }
    Real innerTubeInitialRotation() const { return _it_initial_rotation.value; }
    Real outerTubeOuterDiameter() const { return _ot_outer_diameter.value; }
    Real outerTubeInnerDiameter() const { return _ot_inner_diameter.value; }
    Real outerTubeRadiusOfCurvature() const { return _ot_r_curvature.value; }
    Real outerTubeInitialTranslation() const { return _ot_initial_translation.value; }
    Real outerTubeInitialRotation() const { return _ot_initial_rotation.value; }
    Real outerTubeDistalStraightLength() const { return _ot_distal_straight_length.value; }

    Sim::VirtuosoArm::ToolType toolType() const { return _tool_type.value; }
    Sim::VirtuosoArm::CuttingModel cuttingModel() const { return _cutting_model.value; }
    Real cuttingModelTimeThreshold() const { return _cutting_model_time_threshold.value; }
    Real toolTubeLength() const { return _tool_tube_length.value; }

    Vec3r baseInitialPosition() const { return _base_initial_position.value; }
    Vec3r baseInitialRotation() const { return _base_initial_rotation.value; }

    Real maxOTTranslationSpeed() const { return _max_ot_translation_speed.value; }
    Real maxITTranslationSpeed() const { return _max_it_translation_speed.value; }
    Real maxOTRotationSpeed() const { return _max_ot_rotation_speed.value; }
    Real maxITRotationSpeed() const { return _max_it_rotation_speed.value; }

    const VirtuosoArmToolConfig& toolConfig() const { return _tool_config; }

    protected:
    ConfigParameter<Real> _it_outer_diameter = ConfigParameter<Real>(1.04e-3);
    ConfigParameter<Real> _it_inner_diameter = ConfigParameter<Real>(0.82e-3);
    ConfigParameter<Real> _it_initial_translation = ConfigParameter<Real>(15e-3);
    ConfigParameter<Real> _it_initial_rotation = ConfigParameter<Real>(0);

    ConfigParameter<Real> _ot_outer_diameter = ConfigParameter<Real>(1.56e-3);
    ConfigParameter<Real> _ot_inner_diameter = ConfigParameter<Real>(1.14e-3);
    ConfigParameter<Real> _ot_r_curvature = ConfigParameter<Real>(90);
    ConfigParameter<Real> _ot_distal_straight_length = ConfigParameter<Real>(5e-3);
    ConfigParameter<Real> _ot_initial_translation = ConfigParameter<Real>(15e-3);
    ConfigParameter<Real> _ot_initial_rotation = ConfigParameter<Real>(0);

    ConfigParameter<Sim::VirtuosoArm::ToolType> _tool_type = ConfigParameter<Sim::VirtuosoArm::ToolType>(Sim::VirtuosoArm::ToolType::NONE);
    ConfigParameter<Sim::VirtuosoArm::CuttingModel> _cutting_model = ConfigParameter<Sim::VirtuosoArm::CuttingModel>(Sim::VirtuosoArm::CuttingModel::INSTANT);
    ConfigParameter<Real> _cutting_model_time_threshold = ConfigParameter<Real>(100e-3);
    ConfigParameter<Real> _tool_tube_length = ConfigParameter<Real>(5e-3);

    ConfigParameter<Vec3r> _base_initial_position = ConfigParameter<Vec3r>(Vec3r(0,0,0));
    ConfigParameter<Vec3r> _base_initial_rotation = ConfigParameter<Vec3r>(Vec3r(0,0,0));

    ConfigParameter<Real> _max_ot_translation_speed = ConfigParameter<Real>(0.04);
    ConfigParameter<Real> _max_it_translation_speed = ConfigParameter<Real>(0.0529);
    ConfigParameter<Real> _max_ot_rotation_speed = ConfigParameter<Real>(20);
    ConfigParameter<Real> _max_it_rotation_speed = ConfigParameter<Real>(52.4);

    VirtuosoArmToolConfig _tool_config;
};

} // namespace Config

#endif // __VIRTUOSO_ARM_CONFIG_HPP