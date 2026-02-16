#ifndef __RIGID_OBJECT_HPP
#define __RIGID_OBJECT_HPP

#include "common/types.hpp"
#include "simobject/Object.hpp"
#include "config/simobject/RigidObjectConfig.hpp"
#include "geometry/TransformationMatrix.hpp"
#include "utils/GeometryUtils.hpp"

namespace Sim
{

class RigidObject : public Object
{
    // public typedefs
    public:
    using ConfigType = Config::RigidObjectConfig;
    
    public:
    RigidObject(const Simulation* sim, const ConfigType* config);

    /** Returns a string with all relevant information about this object. 
     * @param indent : the level of indentation to use for formatting new lines of the string
    */
    virtual std::string toString(const int indent) const override;
    
    /** Returns a string with the type of the object. */
    virtual std::string type() const override { return "RigidObject"; }

    /** Evolves this object one time step forward in time. 
     * Completely up to the derived classes to decide how they should step forward in time.
    */
    virtual void update() override;

    virtual void velocityUpdate() override;
    
    /** Getters and setters */
    virtual void setGlobalLinearVelocity(const Vec3r& lin_velocity) { _v = lin_velocity; }
    const Vec3r& globalLinearVelocity() const { return _v; }
    Vec3r bodyLinearVelocity() const { return GeometryUtils::rotateVectorByQuat(_v, GeometryUtils::inverseQuat(_q)); }

    virtual void setGlobalAngularVelocity(const Vec3r& ang_velocity) { _w = ang_velocity; }
    const Vec3r& globalAngularVelocity() const { return _w; }
    Vec3r bodyAngularVelocity() const { return GeometryUtils::rotateVectorByQuat(_w, GeometryUtils::inverseQuat(_q)); }

    virtual void setPosition(const Vec3r& position) { if (!_fixed) _p = position; }
    const Vec3r& position() const { return _p; }
    const Vec3r& prevPosition() const { return _p_prev; }

    virtual void setOrientation(const Vec4r& orientation) { if (!_fixed) _q = orientation; }
    const Vec4r& orientation() const { return _q; }
    const Vec4r& prevOrientation() const { return _q_prev; }

    Geometry::TransformationMatrix transform() const;

    Real mass() const { return _m; }
    Mat3r I() const { return _I; }
    Mat3r invI() const { return _I_inv; }

    bool isFixed() const { return _fixed; }

    /** Applies the force f to the rigid body at point p for a single time step.
     * @param f : the force vector in global coordinates
     * @param p : the point at which to apply the force to the rigid body, in global coordinates
     */
    void applyForceAtPoint(const Vec3r& f, const Vec3r& p);

    /** Returns the coordinates of p (which is specified in global coords) w.r.t the current body frame of this rigid object.
     * @param p : the point expressed in world frame coords
     * @returns the point p expressed in the current body-frame coordinates
    */
    Vec3r globalToBody(const Vec3r& x) const
    {
        return GeometryUtils::rotateVectorByQuat(x - _p, GeometryUtils::inverseQuat(_q));
    }

    /** Returns the coordinates of p (which is specified in body-frame coords) w.r.t the global XYZ coordinate system.
     * @param p : a point in body-frame coordinates
     * @returns the point p expressed in world frame coords
    */
    Vec3r bodyToGlobal(const Vec3r& x) const
    {
        return _p + GeometryUtils::rotateVectorByQuat(x, _q);
    }

    protected:
    /** Position of rigid body */
    Vec3r _p;
    /** Preious position of rigid body. */
    Vec3r _p_prev;
    /** Orientation of rigid body, expressed as a quaternion. */
    Vec4r _q;
    /** Previous orientation of rigid body. */
    Vec4r _q_prev;

    /** Total mass of rigid body. */
    Real _m;
    /** Moment of inertia matrix. */
    Mat3r _I;
    /** Inerse of moment of inertia matrix. */
    Mat3r _I_inv;
    

    /** Translational velocity of rigid body in the global frame */
    Vec3r _v;
    /** Rotational velocity of rigid body in the global frame */
    Vec3r _w;

    /** If the rigid body is fixed (i.e. can't move or rotate). */
    bool _fixed;
    

};

} // namespace Simulation

#endif // __RIGID_OBJECT_HPP