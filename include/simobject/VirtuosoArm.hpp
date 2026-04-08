#ifndef __VIRTUOSO_ARM_HPP
#define __VIRTUOSO_ARM_HPP

#include "simobject/Object.hpp"
#include "simobject/XPBDMeshObjectBaseWrapper.hpp"

#include "geometry/CoordinateFrame.hpp"
#include "geometry/VirtuosoArmSDF.hpp"
#include "geometry/Capsule.hpp"

#include <array>
#include <variant>
#include <unordered_set>

namespace Config
{
    class VirtuosoArmConfig;
}

namespace Sim
{

class XPBDMeshObject_BasePtrWrapper;

/** Simple struct to store information about active collisions between the arm and another object. */
struct VirtuosoArmRigidCollision
{
    /** The node index for the first node of the segment in collision */
    int node_index;
    /** Interpolation parameter in [0,1] for the specific point in collision along the segment. */
    mutable Real interp;

    /** The SDF for the rigid body in collision. */
    const Geometry::SDF* rigid_sdf;

    /** The current force magnitude associated with the collision.
     * This will get incremented or decremented depending on the penetration.
     */
    mutable Real force_mag;

    VirtuosoArmRigidCollision(int node_index_, Real interp_, const Geometry::SDF* rigid_sdf_)
        : node_index(node_index_), interp(interp_), rigid_sdf(rigid_sdf_), force_mag(0)
    {
    }
};

struct VirtuosoArmRigidCollision_Hash
{
    std::size_t operator()(const VirtuosoArmRigidCollision& col) const 
    {
        std::size_t h1 = std::hash<const Geometry::SDF*>{}(col.rigid_sdf);
        std::size_t h2 = std::hash<int>{}(col.node_index);
        return h1 ^ (h2 << 1);  // simple combine
    }
};

struct VirtuosoArmRigidCollision_Equal
{
    bool operator() (const VirtuosoArmRigidCollision& col1, const VirtuosoArmRigidCollision& col2) const
    {
        return col1.rigid_sdf == col2.rigid_sdf && col1.node_index == col2.node_index;
    }
};

struct VirtuosoArmTool
{
    Real outer_dia;
    Real inner_dia;
    Real E;
    Real G;
    Real I;
    Real J;

    VirtuosoArmTool(Real outer_dia_, Real inner_dia_, Real E_)
        : outer_dia(outer_dia_), inner_dia(inner_dia_), E(E_)
    {
        G = E / (2*(1+0.3));
        I = M_PI/4 * (outer_dia*outer_dia*outer_dia*outer_dia/16 - inner_dia*inner_dia*inner_dia*inner_dia/16);
        J = 2*I;
    }

    /** TODO: Add tool action function somehow */
};

static VirtuosoArmTool VirtuosoArmTool_None(0, 0, 0);
static VirtuosoArmTool VirtuosoArmTool_Palpation(0.7e-3, 0.5e-3, 60e9);
static VirtuosoArmTool VirtuosoArmTool_Spatula(0.7e-3, 0.5e-3, 60e9);     /** TODO: find actual properties of spatula tool tube */
static VirtuosoArmTool VirtuosoArmTool_Grasper(0.7e-3, 0.5e-3, 60e9);     /** These are just made up for now */
static VirtuosoArmTool VirtuosoArmTool_Cautery(0.7e-3, 0.5e-3, 60e9);     /** TODO: find actual properties of cautery tool tube */

class VirtuosoArm : public Object
{

    private:
    /** Number of frames along the tube (i.e. number of integration points for each section) */
    constexpr static int NUM_OT_CURVE_FRAMES = 10;      // number of coordinate frames defined along the curved section of the outer tube
    constexpr static int NUM_OT_STRAIGHT_FRAMES = 5;    // number of coordinate frames defined along the straight distal section of the outer tube
    constexpr static int NUM_OT_FRAMES = NUM_OT_CURVE_FRAMES + NUM_OT_STRAIGHT_FRAMES; // total number of coordinate frames defined along the outer tube
    constexpr static int NUM_IT_FRAMES = 10;            // number of coordinate frames defined along the (exposed part of the) inner tube
    constexpr static int NUM_TT_FRAMES = 10;            // number of coordinate frames defined along the (exposed part of the) tool tube

    /** Joint limits */
    constexpr static Real MAX_OT_TRANSLATION = 20e-3;    // maximum outer tube translation (joint limit on Virtuoso system)
    constexpr static Real MAX_IT_TRANSLATION = 40e-3;    // maximum inner tube translation (joint limit on Virtuoso system)

    /** Physical parameters */
    constexpr static Real OT_ENDOSCOPE_CLEARANCE = 0.3e-3;  // clearance between the outer tube and the endoscope sheath [m]
    constexpr static Real OT_RADIUS_OF_CURVATURE = 1.0/60.0;    // nominal radius of curvature of the outer tube [m]
    constexpr static Real E = 60e9;     // nominal Young's modulus of Nitinol
    constexpr static Real G = E / (2*(1+0.3));  // nominal shear modulus of Nitinol

    constexpr static double GRASPING_RADIUS = 0.002;    // grasping radius for the grasper tool

    public:
    using ConfigType = Config::VirtuosoArmConfig;
    using SDFType = Geometry::VirtuosoArmSDF;
    
    using OuterTubeFramesArray = std::array<Geometry::CoordinateFrame, NUM_OT_CURVE_FRAMES + NUM_OT_STRAIGHT_FRAMES>;
    using InnerTubeFramesArray = std::array<Geometry::CoordinateFrame, NUM_IT_FRAMES>;
    using ToolTubeFramesArray = std::array<Geometry::CoordinateFrame, NUM_TT_FRAMES>;

    /** The type of tool attached to the tip of the arm */
    enum class ToolType
    {
        NONE=0,
        PALPATION,
        SPATULA,
        GRASPER,
        CAUTERY
    };

    /** The cutting model used when the cautery tool is used. */
    enum class CuttingModel
    {
        NONE=0,       // elements are not removed
        INSTANT,    // removes elements upon contact with the tool tip
        TIMER,      // uses a timer and a threshold to approximate the time taken to cut
        THERMAL     // applies heat input to the tissue according to power-resistance curve
    };

    /** Maps types in the ToolType enum to their corresponding structs with properties.
     * TODO: this probably shouldn't be necessary, but we'll keep it for now
     */
    static std::map<ToolType, VirtuosoArmTool> TOOL_TYPE_TO_STRUCT;

    /** State of a tube at a given point along the tube.
     * Used in the statics model for the Virtuoso arm, where we integrate from the base to the tip, keeping track of
     *  - position and orientation
     *  - internal force and moment
     *  - total angle swept about z-axis (i.e. torsional angle displacement)
     * 
     * Two helper methods are provided to convert between the struct form and a 1D vector of states.
     */
    struct TubeIntegrationState
    {
        using VecType = Eigen::Vector<Real, 19>; 

        Vec3r position;                 // position at a point along the tube
        Mat3r orientation;              // orientation at a point along the tube
        Vec3r internal_force;           // (global) internal force at a point along the tube
        Vec3r internal_moment;          // (global) internal moment at a point along the tube
        Real torsional_displacement;    // total torsional displacement at this point along the tube

        /** Converts a TubeIntegrationState struct to a 1D vector (for use in integration) */
        static VecType toVec(const TubeIntegrationState& state)
        {
            VecType vec;
            vec << state.position, state.orientation.reshaped(), state.internal_force, state.internal_moment, state.torsional_displacement;
            return vec;
        }

        /** Converts from a 1D vector back to a TubeIntegrationState */
        static TubeIntegrationState fromVec(const VecType& vec)
        {
            TubeIntegrationState state;
            state.position = vec( Eigen::seqN(0,3) );
            state.orientation = vec( Eigen::seqN(3, 9) ).reshaped(3,3);
            state.internal_force = vec( Eigen::seqN(12, 3) );
            state.internal_moment = vec( Eigen::seqN(15, 3) );
            state.torsional_displacement = vec(18);
            return state;
        }

        // helpers for getting/setting parts of the state when it is represented as a vector
        // this way, we can edit the state directly as a vector without having to convert the whole thing back and forth
        static Vec3r positionFromVec(const VecType& vec) { return vec.head<3>(); }
        static Mat3r orientationFromVec(const VecType& vec) { return vec( Eigen::seqN(3,9) ).reshaped(3,3); }
        static Vec3r internalForceFromVec(const VecType& vec) { return vec( Eigen::seqN(12,3) ); }
        static Vec3r internalMomentFromVec(const VecType& vec) { return vec( Eigen::seqN(15,3) ); }
        static Real torsionalDisplacementFromVec(const VecType& vec) { return vec(18); }

        static void setPositionInVec(VecType& vec, const Vec3r& pos) { vec( Eigen::seqN(0,3) ) = pos; }
        static void setOrientationInVec(VecType& vec, const Mat3r& ori) { vec( Eigen::seqN(3,9) ) = ori.reshaped(); }
        static void setInternalForceInVec(VecType& vec, const Vec3r& int_f) { vec( Eigen::seqN(12,3) ) = int_f; }
        static void setInternalMomentInVec(VecType& vec, const Vec3r& int_m) { vec( Eigen::seqN(15,3) ) = int_m; }
        static void setTorsionalDisplacementInVec(VecType& vec, Real tor_disp) { vec(18) = tor_disp; }

    };

    /** Parameters of a tube needed for integration.
     * These are static parameters of the tube, such as the inverse bending/torsion stiffness and the tube's precurvature.
     */
    struct TubeIntegrationParams
    {
        using VecType = Eigen::Vector<Real, 6>;

        Vec3r precurvature;     // precurvature of the tube
        Vec3r K_inv;            // inverse stiffnesses of the tube (in body frame, hence K is diagonal and can be represented by a 3-vector)
    };

    /** Stores information for applying collision forces onto the tube at the appropriate location.
     * This includes the constraint projector for the collision constraint on the deformable object (calculates the force)
     * as well as the node index and interpolation (between 0 and 1) of the force.
     * The force is assumed to be applied between the node at node index and the next node.
     */
    struct CollisionConstraintInfo
    {
        using ProjectorRefType = Solver::ConstraintProjectorReferenceWrapper<Solver::StaticDeformableCollisionConstraint>;

        ProjectorRefType proj_ref;  // the constraint projector reference that can calculate the constraint force
        int node_index; // "global" node index, i.e. from 0 to NUM_OT_FRAMES + NUM_IT_FRAMES
        Real interp;    // between 0 and 1

        CollisionConstraintInfo(ProjectorRefType&& proj_ref_, int node_index_, Real interp_)
            : proj_ref(proj_ref_), node_index(node_index_), interp(interp_)
        {}

        CollisionConstraintInfo(const ProjectorRefType& proj_ref_, int node_index_, Real interp_)
            : proj_ref(proj_ref_), node_index(node_index_), interp(interp_)
        {}
        };

    public:
    VirtuosoArm(const Simulation* sim, const ConfigType* config);

    /** Returns a string with all relevant information about this object. 
     * @param indent : the level of indentation to use for formatting new lines of the string
    */
    virtual std::string toString(const int indent) const override;
    
    /** Returns a string with the type of the object. */
    virtual std::string type() const override { return "VirtuosoArm"; }

    /** Performs any necessary setup for this object.
     * Called after instantiation (i.e. outside the constructor) and before update() is called for the first time.
     */
    virtual void setup() override;

    /** Evolves this object one time step forward in time. 
     * Completely up to the derived classes to decide how they should step forward in time.
    */
    virtual void update() override;

    virtual void velocityUpdate() override;

    /** Returns the axis-aligned bounding-box (AABB) for this Object in global simulation coordinates. */
    virtual Geometry::AABB boundingBox() const override;

    virtual void createSDF() override 
    { 
        if(!_sdf.has_value()) 
            _sdf = SDFType(this); 
    }

    virtual const SDFType* SDF() const override { return _sdf.has_value() ? &_sdf.value() : nullptr; }

    /** Returns the number of integration segments, i.e. the number of distinct segments between integration points */
    int numSegments() const;

    /** Returns a Capsule object for the segment between integration points index and index+1.
     * Throws an error if the index > number of segments
     */
    Geometry::Capsule segment(int index) const;

    Real innerTubeOuterDiameter() const { return _it_outer_dia; }
    Real innerTubeInnerDiameter() const { return _it_inner_dia; }
    Real innerTubeTranslation() const { return _it_translation; }
    Real innerTubeRotation() const { return _it_rotation; }

    Real outerTubeOuterDiameter() const { return _ot_outer_dia; }
    Real outerTubeInnerDiameter() const { return _ot_inner_dia; }
    Real outerTubeRadiusOfCurvature() const { return _ot_r_curvature; }
    Real outerTubeTranslation() const { return _ot_translation; }
    Real outerTubeRotation() const { return _ot_rotation; }
    Real outerTubeDistalStraightLength() const { return _ot_distal_straight_length; }
    int toolState() const { return _tool_state; }
    bool hasTool() const { return (_tool_type != ToolType::NONE); }
    ToolType toolType() const { return _tool_type; }
    const VirtuosoArmTool& toolTube() const { return _tool_tube; }

    void setInnerTubeTranslation(double t) { _it_translation = (t >= 0) ? t : 0; _stale_frames = true; }
    void setInnerTubeRotation(double r) { _it_rotation = r; _stale_frames = true; }
    void setOuterTubeTranslation(double t) { _ot_translation = (t >= 0) ? t : 0; _stale_frames = true; }
    void setOuterTubeRotation(double r) { _ot_rotation = r; _stale_frames = true; }
    void setToolState(int tool) { _tool_state = tool; }
    void setBasePosition(const Vec3r& pos) { _arm_base_position = pos; _stale_frames = true; }
    void setBaseRotation(const Mat3r& rot_mat) { _arm_base_rotation = rot_mat; _stale_frames = true;}

    const Geometry::CoordinateFrame& armBaseFrame() const { return _arm_base_frame; }
    const Geometry::CoordinateFrame& outerTubeStartFrame() const { return _ot_frames[0]; }
    const Geometry::CoordinateFrame& outerTubeCurveEndFrame() const { return _ot_frames[NUM_OT_CURVE_FRAMES - 1]; }
    const Geometry::CoordinateFrame& outerTubeEndFrame() const { return _ot_frames.back(); }
    const Geometry::CoordinateFrame& innerTubeStartFrame() const { return _it_frames[0]; }
    const Geometry::CoordinateFrame& innerTubeEndFrame() const { return _it_frames.back(); }

    const OuterTubeFramesArray& outerTubeFrames() const { return _ot_frames; }
    const InnerTubeFramesArray& innerTubeFrames() const { return _it_frames; }
    const ToolTubeFramesArray& toolTubeFrames() const { return _tt_frames; }

    Vec3r actualTipPosition() const;
    Vec3r commandedTipPosition() const{ return _commanded_tip_position; }
    void setCommandedTipPosition(const Vec3r& new_position);

    Vec3r tipForce() const { return _tip_force; }
    Vec3r tipMoment() const { return _tip_moment; }
    void setTipForce(const Vec3r& new_tip_force);
    void setTipMoment(const Vec3r& new_tip_moment);
    void setTipForceAndMoment(const Vec3r& new_tip_force, const Vec3r& new_tip_moment);

    /** Computes the compliance matrix numerically at the specified node index. */
    Mat3r complianceMatrixAtIntegrationPoint(int node_index);

    /** Computes the compliance matrix at an arbitrary point along the arm,
     * specified by a starting integration point index and an interpolation value in [0,1] indicating the distance along the segment
     * (node_index, node_index+1).
     * 
     * This uses a previously computed cubic interpolation of the compliance matrix.
     * If this interpolation has not already been computed, for the relevant section of the arm (outer tube, inner tube, tool tube),
     * then the interpolation will be computed.
     * 
     * This is useful to determine the force necessary to move a point along the arm a certain distance, which comes in handy during
     * collision resolution.
     */
    Mat3r interpolatedComplianceMatrix(int node_index, Real interp);

    Vec3r unfilteredCollisionForce() const { return _unfiltered_collision_force; }
    Vec3r filteredCollisionForce() const { return _filtered_collision_force; }

    void addCollisionConstraint(const CollisionConstraintInfo::ProjectorRefType& proj_ref, int node_index, Real interp);
    void clearCollisionConstraints();

    void addRigidCollision(int node_index, Real interp, const Geometry::SDF* sdf);

    const Vec3r& outerTubeNodalForce(int node_index) const { return _ot_nodal_forces[node_index]; }
    const Vec3r& innerTubeNodalForce(int node_index) const { return _it_nodal_forces[node_index]; }
    const Vec3r& toolTubeNodalForce(int node_index) const { return _tt_nodal_forces[node_index]; }
    void setOuterTubeNodalForce(int node_index, const Vec3r& force);
    void setInnerTubeNodalForce(int node_index, const Vec3r& force);
    void setToolTubeNodalForce(int node_index, const Vec3r& force);
    void applyNodalForce(int node_index, Real interp, const Vec3r& force);

    const XPBDMeshObject_BasePtrWrapper& toolManipulatedObject() const { return _tool_manipulated_object; }
    void setToolManipulatedObject(const XPBDMeshObject_BasePtrWrapper& obj) { _tool_manipulated_object = obj; }

    void setJointState(double ot_rotation, double ot_translation, double it_rotation, double it_translation, int tool);

    private:
    /** Computes the clearance angle between the outer tube and the endoscope sheath given the current outer tube translation.
     * This angle rotates the outer tube base frame about its local x-axis.
    */
    Real _outerTubeClearanceAngle(Real ot_trans);

    /** Recomputes coordinate frames along the Virtuoso arm using purely geometry and not including any tip forces.
     * This is not used.
     */
    void _recomputeCoordinateFrames();

    /** Recomputes coordinate frames along the Virtuoso arm using a small-deflection assumption statics model.
     * This is able to incorporate tip forces and moments into the model.
     */
    void _recomputeCoordinateFramesStaticsModel();

    /** Recomputes coordinate frames along the Virtuoso arm using a small-deflection assumption statics model,
     * with forces at each "node" (i.e. coordinate frame along the backbone) included.
     */
    void _recomputeCoordinateFramesStaticsModelWithNodalForces();

    /** Computes the coefficients for the cubic compliance matrix interpolation over each section of the arm.
     * Uses finite differencing at 4 points along each section (outer tube, inner tube, tool tube) to compute the compliance matrix,
     * and then fits a polynomial for each matrix entry (6 of these, since compliance matrix is symmetric).
     * 
     * i.e. C_ij = a0 + a1*s + a2*s^2 + a3*s^3
     * 
     * Each section of the arm (outer tube, inner tube, tool tube) has an interpolation that varies according to an s in [0,1].
     * 
     * 
     */
    void _computeComplianceMatrixInterpolation(bool ot, bool it, bool tt);

    std::vector<TubeIntegrationState::VecType> _integrateTubeRK4(
        const TubeIntegrationState& tube_base_state, const std::vector<Real>& s, const Vec3r& K_inv, const Vec3r& u_star) const;

    template <typename ForceIterator>
    std::vector<TubeIntegrationState::VecType> _integrateTubeWithForceBoundariesRK4(
        const TubeIntegrationState& tube_base_state, const std::vector<Real>& s, ForceIterator force_iterator,
        const Vec3r& K_inv, const Vec3r& u_star) const;

    /** Performs any tool actions, if applicable.
     * 
     * For example, if the grasper tool is used and the tool state changes from 0 to 1, we will grasp all mesh vertices in the
     *   _tool_manipulated_object that are within the grasping radius. When the tool state changes from 1 to 0, we will release these vertices.
     */
    void _toolAction();

    void _spatulaToolAction();

    void _grasperToolAction();

    void _cauteryToolAction();

    Geometry::TransformationMatrix _computeTipTransform(Real ot_rot, Real ot_trans, Real it_rot, Real it_trans);

    /** Computes the new joint positions given a change in tip position, using only the analytical Jacobian.
     * The Jacobian used is the analytical derivative of the tip transformation matrix (i.e. not a numerical Jacobian)
     *   that does not incorporate the effect of tip forces.
     * 
     * Note: this method easily breaks when the Jacobian becomes singular or the commanded position is outside the reachable workspace of the robot.
     */
    void _jacobianDifferentialInverseKinematics(const Vec3r& dx);

    /** Computes the new joint positions given a change in tip position, using a hybrid approach combining analytical geometry and the analytical Jacobian.
     * The outer tube rotation is given by the angle of the commanded position in cylindrical coordinates (i.e. easily solved for analytically).
     *   - note: we limit the change in outer tube rotation (which may be quite large around a singularity) if the change is larger than what the motors can physically do.
     * 
     * The updates in outer tube and inner tube translation are given by the Jacobian.
     * 
     * Note: this method is more robust to the singularity while maintaining the favorable properties of Jacobian-based inverse kinematics.
     * Another note: the Jacobian used also does not incorporate applied forces. 
     */
    void _hybridDifferentialInverseKinematics(const Vec3r& dx);

    /** Computes the spatial Jacobian for the tip position w.r.t the outer tube rotation, outer tube translation, and inner tube translation.
     * Used in the 3DOF positional differential inverse kinematics.
     * 
     * (The only joint variables that affect tip position are outer tube rotation, outer tube translation, inner tube translation)
     */
    Eigen::Matrix<Real,6,3> _3DOFSpatialJacobian();

    /** Computes the spatial Jacobian for the tip position w.r.t the outer tube rotation, outer tube translation, and inner tube translation.
     * Uses a NUMERICAL approach - varies each joint variable and computes the change in the tip transform.
     */
    Eigen::Matrix<Real,6,3> _3DOFNumericalSpatialJacobian();

    /** Computes the hybrid Jacobian for the tip position w.r.t the outer tube rotation, outer tube translation, and inner tube translation.
     * This is a 3x3 matrix (J_a) that relates the velocity of the tip position in the world frame (p_dot) to the actuator velocities (q_dot):
     *      p_dot = J_a * q_dot
     */
    Mat3r _3DOFAnalyticalHybridJacobian();

    private:
    Real _it_outer_dia; // inner tube outer diameter, in m
    Real _ot_outer_dia; // outer tube outer diameter, in m
    Real _ot_inner_dia; // outer tube inner diameter, in m
    Real _it_inner_dia; // inner tube inner diameter, in m
    Real _ot_r_curvature; // outer tube radius of curvature, in m

    Real _it_translation; // translation of the inner tube. Right now, assuming that when translation=0, inner tube is fully retracted
    Real _it_rotation;    // rotation of inner tube. Right now, assuming angle is measured CCW from positive x-axis 
    Real _ot_translation; // translation of the outer tube. Right now, assuming that when translation=0, outer tube is fully retracted
    Real _ot_rotation;    // rotation of the outer tube. Right now, assuming rotation=0 corresponds to a curve to the left in the XY plane
    Real _ot_distal_straight_length; // the length of the straight section on the distal part of the outer tube

    Real _max_ot_translation_speed;     // max translation speed of the outer tube (m/s)
    Real _max_it_translation_speed;     // max translation speed of the inner tube (m/s)
    Real _max_ot_rotation_speed;        // max rotation speed of the outer tube (rad/s)
    Real _max_it_rotation_speed;        // max rotation speed of the inner tube (rad/s)

    int _tool_state; // state of the tool (i.e. 1=ON, 0=OFF)
    int _last_tool_state; // the previous state of the tool (needed so that we know when tool state has changed)
    ToolType _tool_type; // type of tool used on this arm
    CuttingModel _cutting_model; // the type of cutting model to use (only applies when the cautery tool is equipped)
    Real _cutting_model_time_threshold; // the time threshold to be used for the "timer" cutting model (only applies when this cutting model is used)
    Real _tool_tube_length; // exposed length of the tool tube, in m
    VirtuosoArmTool _tool_tube = VirtuosoArmTool_None; // stores the tool tube properties
    


    XPBDMeshObject_BasePtrWrapper _tool_manipulated_object; // the deformable object that this tool is manipulating
    Vec3r _tool_position; // position of the tool in global coordinates (note that this may be different than the inner tube tip position)
    Vec3r _commanded_tip_position; // tip position of the arm in the absence of tip forces (i.e. where we tell the arm tip to be at)
    std::vector<int> _grasped_vertices; // vertices that are actively being grasped
    std::vector<Solver::ConstraintProjectorReferenceWrapper<Solver::AttachmentConstraint>> _grasping_constraints; // attachment constraints associated with the grasping
    std::vector<CollisionConstraintInfo> _collision_constraints;

    /** Stores information about collisions with rigid objects */
    std::unordered_set<VirtuosoArmRigidCollision, VirtuosoArmRigidCollision_Hash, VirtuosoArmRigidCollision_Equal> _rigid_collisions;

    Vec3r _arm_base_position;
    Mat3r _arm_base_rotation;

    Vec3r _tip_force;
    Vec3r _tip_moment;

    /** The unfiltered net collision force felt by the tube.
     * This is the nominal total collision force (expressed in the global frame), added up across all the collision constraints.
     * This is NOT the actual force used by the quasistatic model - the force used by the model is smoothed using a complementary filter.
     */
    Vec3r _unfiltered_collision_force;

    /** The "filtered" net collision force felt by the tube.
     * This is the force that is used by the quasistatic model - it is smoothed using a complementary filter.
     */
    Vec3r _filtered_collision_force;

    Geometry::CoordinateFrame _arm_base_frame;        // coordinate frame at the tool channel (where it leaves the endoscope)
    
    OuterTubeFramesArray _ot_frames;  // coordinate frames along the backbone of the exposed part of the outer tube
    InnerTubeFramesArray _it_frames;  // coordinate frames along the backbone of the exposed part of the inner tube
    ToolTubeFramesArray _tt_frames;   // coordinate frames along the backbone of the exposed part of the tool tube

    std::array<Vec3r, NUM_OT_FRAMES> _ot_nodal_forces;
    std::array<Vec3r, NUM_IT_FRAMES> _it_nodal_forces;
    std::array<Vec3r, NUM_TT_FRAMES> _tt_nodal_forces;

    bool _stale_frames;     // true if the joint variables have been updated and the coordinate frames need to be recomputed

    /** If the compliance matrix interpolation is "Stale" and needs to be recomputed
     * This occurs when (1) the joint variables change or (2) the applied force changes.
     */
    bool _stale_ot_compliance_interp = true;
    bool _stale_it_compliance_interp = true;
    bool _stale_tt_compliance_interp = true;

    /** Interpolation coefficients for the compliance matrix interpolation along the appropriate section of the arm.
     * i.e. C_ij = a0 + a1*s + a2*s^2 + a3*s^3
     * 
     * Each section of the arm (outer tube, inner tube, tool tube) has an interpolation that varies according to an s in [0,1].
     * E.g., for the inner tube, s=0.5 corresponds to the middle of the inner tube, s=1.0 corresponds to the tip of the inner tube
     */
    std::array<Mat3r, 4> _ot_compliance_coeff;
    std::array<Mat3r, 4> _it_compliance_coeff;
    std::array<Mat3r, 4> _tt_compliance_coeff;

    /** Signed Distance Field for the Virtuoso arm. Must be created explicitly with createSDF(). */
    std::optional<SDFType> _sdf;


};

} // namespace Sim

#endif // __VRITUOSO_ARM_HPP