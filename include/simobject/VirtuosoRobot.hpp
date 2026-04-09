#ifndef __VIRTUOSO_ROBOT_HPP
#define __VIRTUOSO_ROBOT_HPP

#include "config/simobject/VirtuosoRobotConfig.hpp"

#include "simobject/Object.hpp"
#include "simobject/VirtuosoArm.hpp"

#include "geometry/VirtuosoRobotSDF.hpp"

namespace Sim
{

class VirtuosoRobot : public Object
{
    public:
    using SDFType = Geometry::VirtuosoRobotSDF;
    using ConfigType = Config::VirtuosoRobotConfig;

    public:
    VirtuosoRobot() = default;
    explicit VirtuosoRobot(const Simulation* sim, const ConfigType* config);

    virtual void serialize(std::vector<std::byte>& buf) const override;
    virtual void deserialize(const std::byte*& buf) override;


    /** Returns a string with all relevant information about this object. 
     * @param indent : the level of indentation to use for formatting new lines of the string
    */
    virtual std::string toString(const int indent) const override;
    
    /** Returns a string with the type of the object. */
    virtual std::string type() const override { return "VirtuosoRobot"; }

    bool hasArm1() const { return _arm1.has_value(); }
    bool hasArm2() const { return _arm2.has_value(); }
    int numArms() const { return _arm1.has_value() + _arm2.has_value(); }
    const VirtuosoArm* arm1() const { return _arm1.has_value() ? &_arm1.value() : nullptr; }
    const VirtuosoArm* arm2() const { return _arm2.has_value() ? &_arm2.value() : nullptr; }
    VirtuosoArm* arm1() { return _arm1.has_value() ? &_arm1.value() : nullptr; }
    VirtuosoArm* arm2() { return _arm2.has_value() ? &_arm2.value() : nullptr; }

    Real endoscopeLength() const { return _endoscope_length; }
    Real endoscopeDiameter() const { return _endoscope_dia; }
    Real armSeparationDistance() const { return _arm_separation_dist; }

    const Geometry::CoordinateFrame& endoscopeFrame() const { return _endoscope_frame; }
    const Geometry::CoordinateFrame& VBFrame() const { return _VB_frame; }
    const Geometry::CoordinateFrame& VRFrame() const { return _VR_frame; }
    const Geometry::CoordinateFrame& camFrame() const { return _cam_frame; }

    /** Updates the camera transform based on its relative position to VB.
     * Returns true if the camera transform is different from before.
     */
    bool setVBtoCamTransform(const Geometry::TransformationMatrix& new_transform);

    virtual Geometry::AABB boundingBox() const override
    {
        // TODO: valid AABB
        return Geometry::AABB(0,0,0,1,1,1);
    }

    virtual void createSDF() override 
    { 
        if(!_sdf.has_value()) 
            _sdf = SDFType(this); 
    }

    virtual const SDFType* SDF() const override { return _sdf.has_value() ? &_sdf.value() : nullptr; }
    

    /** Performs any necessary setup for this object.
     * Called after instantiation (i.e. outside the constructor) and before update() is called for the first time.
     */
    virtual void setup() override;

    /** Evolves this object one time step forward in time. 
     * Completely up to the derived classes to decide how they should step forward in time.
    */
    virtual void update() override;

    virtual void velocityUpdate() override;

    #ifdef HAVE_CUDA
    virtual void createGPUResource() override { assert(0); /* not implemented */ }
    #endif
    private:
    Real _endoscope_dia;              // diameter (in m) of the endoscope
    Real _endoscope_length;           // length (in m) of the endoscope
    Real _arm_separation_dist;        // horizontal distance (in m) between the centers of the two arms
    Real _optic_vertical_dist;        // vertical distance (in m) between the centers of the arms and the optic
    Real _optic_forward_dist;         // distance along the +z axis (in m) between the optic center and the base of the arms
    Real _optic_tilt;            // rotation (in rad) of the optic around the positive X axis

    // frame at the center of the end of the endoscope
    // Z points forward away from the endoscope (aligned with initial arm Z axes)
    // Y up, X left
    Geometry::CoordinateFrame _endoscope_frame;

    Geometry::CoordinateFrame _VB_frame;    // frame of the base of the left Virtuoso arm
    Geometry::CoordinateFrame _VR_frame;    // frame of the base of the right Virtuoso arm
    Geometry::CoordinateFrame _cam_frame;   // frame of the camera

    std::optional<VirtuosoArm> _arm1;
    std::optional<VirtuosoArm> _arm2;

    /** Signed Distance Field for the Virtuoso robot. Must be created explicitly with createSDF(). */
    std::optional<SDFType> _sdf;
};

} // namespace Sim

#endif // __VIRTUOSO_ROBOT_HPP