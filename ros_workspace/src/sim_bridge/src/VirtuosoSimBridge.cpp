#include "sim_bridge/VirtuosoSimBridge.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "utils/MathUtils.hpp"

VirtuosoSimBridge::VirtuosoSimBridge(Sim::VirtuosoSimulation* sim)
    : SimBridge<Sim::VirtuosoSimulation>(sim)
{
    _setupTransformBroadcaster();
    _setupTransformBufferAndListener();
    _setupVBtoCamTransformListener();

    _setupPublishers();

    _setupSubscribers();
}


std::tuple<double, double, double, double, int> VirtuosoSimBridge::_jointMsgToJointState(sensor_msgs::msg::JointState* msg) const
{
    // joint states must contain five positions: inner rotation, outer rotation, inner
    // translation, and outer translation, and tool (in any order).
    if (msg->name.size() != 5) {
        // joint state doesn't contain the necessary fields; it is malformed.
        RCLCPP_WARN(
            get_logger(),
            "Joint state contains %lu states in 'name'. Valid joint states contain 5 states.",
            msg->name.size());
        return std::tuple<double, double, double, double, int>();
    } else if (msg->position.size() != 5) {
        // joint state doesn't contain the necessary fields; it is malformed.
        RCLCPP_WARN(
            get_logger(),
            "Joint state contains %lu states in 'position'. Valid joint states contain 5 states.",
            msg->position.size());
        return std::tuple<double, double, double, double, int>();
    }

    bool irSet = false, orSet = false, itSet = false, otSet = false, tSet = false;
    double ot_rot, ot_trans, it_rot, it_trans;
    int tool;

    for (uint32_t idx = 0; idx < 5; ++idx) {
        const auto &name = msg->name[idx];
        const auto scalar = msg->position[idx];

        if (name == "inner_rotation" || name == "innerRotation" || name == "ir") {
            irSet = true;
            it_rot = scalar;
        } else if (name == "outer_rotation" || name == "outerRotation" || name == "or") {
            orSet = true;
            ot_rot = scalar;
        } else if (name == "inner_translation" || name == "innerTranslation" || name == "it") {
            itSet = true;
            it_trans = scalar;
        } else if (name == "outer_translation" || name == "outerTranslation" || name == "ot") {
            otSet = true;
            ot_trans = scalar;
        } else if (name == "tool") {
            tSet = true;
            tool = scalar;
        } else {
            // this joint state was referenced incorrectly
            RCLCPP_WARN(
            get_logger(), "Attempt to set nonexistent joint '%s' failed.", name.c_str());
            return std::tuple<double, double, double, double, int>();
        }
    }

    // ensure that every joint was set
    if (irSet == false || orSet == false || itSet == false || otSet == false || tSet == false) {
        if (irSet == false) {
            RCLCPP_WARN(get_logger(), "Joint state message did not set inner_rotation joint.");
        } else if (orSet == false) {
            RCLCPP_WARN(get_logger(), "Joint state message did not set outer_rotation joint.");
        } else if (itSet == false) {
            RCLCPP_WARN(get_logger(), "Joint state message did not set inner_translation joint.");
        } else if (otSet == false) {
            RCLCPP_WARN(get_logger(), "Joint state message did not set outer_translation joint.");
        } else if (tSet == false) {
            RCLCPP_WARN(get_logger(), "Joint state message did not set tool joint.");
        }
        return std::tuple<double, double, double, double, int>();
    }

    return std::make_tuple(ot_rot, ot_trans, it_rot, it_trans, tool);
}

sensor_msgs::msg::JointState VirtuosoSimBridge::_createJointStateMsgForArm(const Sim::VirtuosoArm* arm) const
{
    sensor_msgs::msg::JointState msg;
    msg.name = {"inner_rotation", "outer_rotation", "inner_translation", "outer_translation", "tool"};
    msg.position = {arm->innerTubeRotation(), arm->outerTubeRotation(), arm->innerTubeTranslation(), arm->outerTubeTranslation(), (double)arm->toolState()};

    msg.header.stamp = this->now();

    return msg;
}

geometry_msgs::msg::Pose VirtuosoSimBridge::_poseFromTransformationMatrix(const Geometry::TransformationMatrix& transform) const
{
    geometry_msgs::msg::Pose pose;

    const Vec3r& origin = transform.translation();
    const Vec4r& quat = GeometryUtils::matToQuat(transform.rotMat());
    pose.position.x = origin[0];
    pose.position.y = origin[1];
    pose.position.z = origin[2];

    pose.orientation.x = quat[0];
    pose.orientation.y = quat[1];
    pose.orientation.z = quat[2];
    pose.orientation.w = quat[3];

    return pose;
}

void VirtuosoSimBridge::_setupTransformBufferAndListener()
{
    _tf_buffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    _tf_listener = std::make_shared<tf2_ros::TransformListener>(*_tf_buffer);
}


/** === Publishers === */

void VirtuosoSimBridge::_setupVBtoCamTransformListener()
{
    this->declare_parameter("cam_frame_name", "ves/camera");

    auto vb_to_cam_callback = [this] () -> void {
        std::string source_frame = this->get_parameter("cam_frame_name").as_string();
        std::string target_frame = "ves/left/base";

        // get the latest transform (if it exists)
        if (this->_tf_buffer->canTransform(target_frame, source_frame, tf2::TimePointZero))
        {
            try
            {
                // try getting the transform
                auto transform_msg = this->_tf_buffer->lookupTransform(
                    target_frame, source_frame, tf2::TimePointZero);
                
                // create TransformationMatrix object from transform
                Vec3r origin(transform_msg.transform.translation.x, transform_msg.transform.translation.y, transform_msg.transform.translation.z);
                Vec4r quat(transform_msg.transform.rotation.x, transform_msg.transform.rotation.y, transform_msg.transform.rotation.z, transform_msg.transform.rotation.w);
                Mat3r rot_mat = GeometryUtils::quatToMat(quat);
                Geometry::TransformationMatrix vb_to_cam(rot_mat, origin);
                
                bool updated = _sim->virtuosoRobot()->setVBtoCamTransform(vb_to_cam);
                if (updated) _sim->updateGraphicsCameraPoseToRobotCamFrame();   // only update the viewer if a change was actually made
                    
            }
            catch (tf2::TransformException& ex)
            {
                RCLCPP_WARN(this->get_logger(), "Error getting transform: %s", ex.what());
            }
        }
        else
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "Transform from %s to %s does not exist yet", 
                source_frame.c_str(), target_frame.c_str());
        }

    };

    _sim->addCallback(0.5, vb_to_cam_callback);
}

void VirtuosoSimBridge::_setupTransformBroadcaster()
{
    _tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    // set up callback to publish transforms
    std::cout << "Setting up callback for publishing transforms with tf..." << std::endl;

    auto tf_callback = 
        [this]() -> void {
            geometry_msgs::msg::TransformStamped t;
            t.header.stamp = this->now();
            t.header.frame_id = "ves/left/base";
            t.child_frame_id = "sim/world";

            const Geometry::TransformationMatrix& vb_transform_inv = this->_sim->virtuosoRobot()->VBFrame().transform().inverse();
            t.transform.translation.x = vb_transform_inv.translation()[0];
            t.transform.translation.y = vb_transform_inv.translation()[1];
            t.transform.translation.z = vb_transform_inv.translation()[2];

            Vec4r quat = GeometryUtils::matToQuat(vb_transform_inv.rotMat());
            t.transform.rotation.x = quat[0];
            t.transform.rotation.y = quat[1];
            t.transform.rotation.z = quat[2];
            t.transform.rotation.w = quat[3];

            this->_tf_broadcaster->sendTransform(t);
        };
    
    _sim->addCallback(1.0/this->get_parameter("publish_rate_hz").as_double(), tf_callback);
}


void VirtuosoSimBridge::_setupPublishers()
{
    // set up publishers for arm1 (if it exists)
    if (_sim->virtuosoRobot()->hasArm1())
    {
        const Sim::VirtuosoArm* arm1 = _sim->virtuosoRobot()->arm1();

        _arm1_joint_state_publisher = this->create_publisher<sensor_msgs::msg::JointState>("/sim/output/arm1_joint_state", 10);
        _arm1_frames_publisher = this->create_publisher<geometry_msgs::msg::PoseArray>("/sim/output/arm1_frames", 10);
        _arm1_tip_frame_publisher = this->create_publisher<geometry_msgs::msg::PoseStamped>("/sim/output/arm1_tip_frame", 10);
        _arm1_commanded_tip_frame_publisher = this->create_publisher<geometry_msgs::msg::PoseStamped>("/sim/output/arm1_commanded_tip_frame", 10);
        _arm1_net_force_publisher = this->create_publisher<geometry_msgs::msg::Vector3Stamped>("/sim/output/arm1_net_force", 10);


        _setupArmJointStatePublisher(arm1, _arm1_joint_state_publisher);
        _setupArmFramesPublisher(arm1, _arm1_frames_publisher);
        _setupArmTipFramePublisher(arm1, _arm1_tip_frame_publisher);
        _setupArmCommandedTipFramePublisher(arm1, _arm1_commanded_tip_frame_publisher);
        _setupArmNetForcePublisher(arm1, _arm1_net_force_publisher);

        if (arm1->hasTool())
        {
            _arm1_tool_tip_frame_publisher = this->create_publisher<geometry_msgs::msg::PoseStamped>("/sim/output/arm1_tool_tip_frame", 10);
            _setupArmToolTipFramePublisher(arm1, _arm1_tool_tip_frame_publisher);
        }
    }

    // set up publishers for arm2 (if it exists)
    if (_sim->virtuosoRobot()->hasArm2())
    {
        const Sim::VirtuosoArm* arm2 = _sim->virtuosoRobot()->arm2();

        _arm2_joint_state_publisher = this->create_publisher<sensor_msgs::msg::JointState>("/sim/output/arm2_joint_state", 10);
        _arm2_frames_publisher = this->create_publisher<geometry_msgs::msg::PoseArray>("/sim/output/arm2_frames", 10);
        _arm2_tip_frame_publisher = this->create_publisher<geometry_msgs::msg::PoseStamped>("/sim/output/arm2_tip_frame", 10);
        _arm2_commanded_tip_frame_publisher = this->create_publisher<geometry_msgs::msg::PoseStamped>("/sim/output/arm2_commanded_tip_frame", 10);
        _arm2_net_force_publisher = this->create_publisher<geometry_msgs::msg::Vector3Stamped>("/sim/output/arm2_net_force", 10);

        _setupArmJointStatePublisher(arm2, _arm2_joint_state_publisher);
        _setupArmFramesPublisher(arm2, _arm2_frames_publisher);
        _setupArmTipFramePublisher(arm2, _arm2_tip_frame_publisher);
        _setupArmCommandedTipFramePublisher(arm2, _arm2_commanded_tip_frame_publisher);
        _setupArmNetForcePublisher(arm2, _arm2_net_force_publisher);

        if (arm2->hasTool())
        {
            _arm2_tool_tip_frame_publisher = this->create_publisher<geometry_msgs::msg::PoseStamped>("/sim/output/arm2_tool_tip_frame", 10);
            _setupArmToolTipFramePublisher(arm2, _arm2_tool_tip_frame_publisher);
        }
    }

    _virtuoso_self_collision_publisher = this->create_publisher<sim_bridge::msg::ArmArmCollision>("/sim/output/virtuoso_self_collision", 10);
    auto arms_in_collision_callback = 
        [this]() -> void {
            if (!this->_sim->virtuosoRobot()->hasArm1() || !this->_sim->virtuosoRobot()->hasArm2())
                return;

            Sim::VirtuosoArm* arm1 = this->_sim->virtuosoRobot()->arm1();
            Sim::VirtuosoArm* arm2 = this->_sim->virtuosoRobot()->arm2();

            sim_bridge::msg::ArmArmCollision msg;
            if (arm1->virtuosoArmCollisions().empty() && arm2->virtuosoArmCollisions().empty())
                msg.collision = 0;
            else
            {
                const Sim::VirtuosoArmVirtuosoArmCollision* collision;
                if (!arm1->virtuosoArmCollisions().empty())
                    collision = &arm1->virtuosoArmCollisions().front();
                else
                    collision = &arm2->virtuosoArmCollisions().front();
                    
                Real arm1_ot_trans = arm1->outerTubeTranslation();
                Real arm1_it_trans = arm1->innerTubeTranslation();
                Real arm1_length = std::max(arm1_ot_trans, arm1_it_trans);
                Real interp1 = collision->interp1 - static_cast<int>(collision->interp1);

                Real arm2_ot_trans = arm2->outerTubeTranslation();
                Real arm2_it_trans = arm2->innerTubeTranslation();
                Real arm2_length = std::max(arm2_ot_trans, arm2_it_trans);
                Real interp2 = collision->interp2 - static_cast<int>(collision->interp2);
                
                Real arm1_s = -1, arm2_s = -1;

                if (collision->is_tool1)
                {
                    Real z_coord = collision->contact_point1[2];
                    arm1_s = (arm1_length + z_coord) / arm1_length;
                }
                else
                {
                    if (collision->node_index1 < Sim::VirtuosoArm::NUM_OT_FRAMES)
                        arm1_s = arm1_ot_trans/arm1_length * (collision->node_index1 + interp1) / Sim::VirtuosoArm::NUM_OT_FRAMES;
                    else
                        arm1_s = (arm1_ot_trans + (arm1_it_trans - arm1_ot_trans)*(collision->node_index1 - Sim::VirtuosoArm::NUM_OT_FRAMES + interp1) / Sim::VirtuosoArm::NUM_IT_FRAMES) / arm1_length;
                }

                if (collision->is_tool2)
                {
                    Real z_coord = collision->contact_point2[2];
                    arm2_s = (arm2_length + z_coord) / arm2_length;
                }
                else
                {
                    if (collision->node_index2 < Sim::VirtuosoArm::NUM_OT_FRAMES)
                        arm2_s = arm2_ot_trans/arm2_length * (collision->node_index2 + interp2) / Sim::VirtuosoArm::NUM_OT_FRAMES;
                    else
                        arm2_s = (arm2_ot_trans + (arm2_it_trans - arm2_ot_trans)*(collision->node_index2 - Sim::VirtuosoArm::NUM_OT_FRAMES + interp2) / Sim::VirtuosoArm::NUM_IT_FRAMES) / arm2_length;
                }

                // std::cout << "Virtuoso arm collision!\n\tarm1: " << arm1->name() << " arm1 s: " << arm1_s << " arm1 is tool: " << collision->is_tool1 << " arm1 node index: " << collision->node_index1 << " arm1 interp: " << interp1 << 
                    // "\n\tarm2:" << arm2->name() << " arm2 s: " << arm2_s << "  arm2 is tool: " << collision->is_tool2 << " arm2 node index: " << collision->node_index2 << " arm2 interp: " << interp2 << std::endl;

                msg.collision = 1;
                msg.arm1_location = arm1_s;
                msg.arm2_location = arm2_s;
            }
                

            this->_virtuoso_self_collision_publisher->publish(msg);
        };
    _sim->addCallback(1.0/this->get_parameter("publish_rate_hz").as_double(), arms_in_collision_callback);

    _virtuoso_grasped_publisher = this->create_publisher<std_msgs::msg::Int8>("/sim/output/virtuoso_grasped", 10);
    auto grasping_callback = 
        [this]() -> void {
            bool is_grasping = false;
            if (this->_sim->virtuosoRobot()->hasArm1() && this->_sim->virtuosoRobot()->arm1()->toolType() == Sim::VirtuosoArm::ToolType::GRASPER)
            {
                is_grasping = this->_sim->virtuosoRobot()->arm1()->graspingForce().norm() > 1e-6;
            }
            if (this->_sim->virtuosoRobot()->hasArm2() && this->_sim->virtuosoRobot()->arm2()->toolType() == Sim::VirtuosoArm::ToolType::GRASPER)
            {
                is_grasping = this->_sim->virtuosoRobot()->arm2()->graspingForce().norm() > 1e-6;
            }

            std_msgs::msg::Int8 msg;
            msg.data = static_cast<int>(is_grasping);
            this->_virtuoso_grasped_publisher->publish(msg);
        };
    _sim->addCallback(1.0/this->get_parameter("publish_rate_hz").as_double(), grasping_callback);
}


void VirtuosoSimBridge::_setupArmJointStatePublisher(const Sim::VirtuosoArm* arm, rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher)
{
    if (!arm)
        return;

    auto callback = 
        [this, arm, publisher]() -> void {
            auto message = this->_createJointStateMsgForArm(arm);
            publisher->publish(message);
        };
    
    _sim->addCallback(1.0/this->get_parameter("publish_rate_hz").as_double(), callback);
}


void VirtuosoSimBridge::_setupArmFramesPublisher(const Sim::VirtuosoArm* arm, rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr publisher)
{
    auto callback = 
        [this, arm, publisher]() -> void {
            const Sim::VirtuosoArm::OuterTubeFramesArray& ot_frames = arm->outerTubeFrames();
            const Sim::VirtuosoArm::InnerTubeFramesArray& it_frames = arm->innerTubeFrames();


            auto message = geometry_msgs::msg::PoseArray();
            message.header.stamp = this->now();
            message.header.frame_id = "ves/left/base";

            const Geometry::CoordinateFrame& vb_frame = this->_sim->virtuosoRobot()->VBFrame();
            const Geometry::TransformationMatrix vb_transform_inv = vb_frame.transform().inverse();
            for (const auto& frame : ot_frames)
            {
                const Geometry::TransformationMatrix transform = vb_transform_inv * frame.transform();
                message.poses.push_back(this->_poseFromTransformationMatrix(transform));
            }

            for (const auto& frame : it_frames)
            {
                const Geometry::TransformationMatrix transform = vb_transform_inv * frame.transform();
                message.poses.push_back(this->_poseFromTransformationMatrix(transform));
            }
            

            // RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
            publisher->publish(message);
        };

    _sim->addCallback(1.0/this->get_parameter("publish_rate_hz").as_double(), callback);
}


void VirtuosoSimBridge::_setupArmTipFramePublisher(const Sim::VirtuosoArm* arm, rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher)
{
    auto callback = 
        [this, arm, publisher]() -> void {
            const Sim::VirtuosoArm::InnerTubeFramesArray& it_frames = arm->innerTubeFrames();

            auto message = geometry_msgs::msg::PoseStamped();
            message.header.stamp = this->now();
            message.header.frame_id = "ves/left/base";

            const Geometry::CoordinateFrame& vb_frame = this->_sim->virtuosoRobot()->VBFrame();
            const Geometry::TransformationMatrix vb_transform_inv = vb_frame.transform().inverse();
            const Geometry::CoordinateFrame& tip_frame = it_frames.back();

            const Geometry::TransformationMatrix transform = vb_transform_inv * tip_frame.transform();
            message.pose = this->_poseFromTransformationMatrix(transform);
            
            publisher->publish(message);
        };

    _sim->addCallback(1.0/this->get_parameter("publish_rate_hz").as_double(), callback);
}


void VirtuosoSimBridge::_setupArmCommandedTipFramePublisher(const Sim::VirtuosoArm* arm, rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher)
{
    auto callback = 
        [this, arm, publisher]() -> void {
            const Vec3r& arm_commanded_position = arm->commandedTipPosition();


            auto message = geometry_msgs::msg::PoseStamped();
            message.header.stamp = this->now();
            message.header.frame_id = "ves/left/base";

            const Geometry::CoordinateFrame& vb_frame = this->_sim->virtuosoRobot()->VBFrame();
            const Geometry::TransformationMatrix vb_transform_inv = vb_frame.transform().inverse();
            const Geometry::TransformationMatrix arm_commanded_pose = arm->commandedTipPose();

            const Geometry::TransformationMatrix vb_pose = vb_transform_inv * arm_commanded_pose;
            message.pose.position.x = vb_pose.translation()[0];
            message.pose.position.y = vb_pose.translation()[1];
            message.pose.position.z = vb_pose.translation()[2];

            /** commanded tip position has no orientation */
            Vec4r quat = MathUtils::matToQuat(vb_pose.rotMat());
            message.pose.orientation.x = quat[0];
            message.pose.orientation.y = quat[1];
            message.pose.orientation.z = quat[2];
            message.pose.orientation.w = quat[3];
            
            publisher->publish(message);
        };

    _sim->addCallback(1.0/this->get_parameter("publish_rate_hz").as_double(), callback);
}


void VirtuosoSimBridge::_setupArmToolTipFramePublisher(const Sim::VirtuosoArm* arm, rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher)
{
    auto callback = 
        [this, arm, publisher]() -> void {

            auto message = geometry_msgs::msg::PoseStamped();
            message.header.stamp = this->now();
            message.header.frame_id = "ves/left/base";

            const Geometry::CoordinateFrame& vb_frame = this->_sim->virtuosoRobot()->VBFrame();
            const Geometry::TransformationMatrix vb_transform_inv = vb_frame.transform().inverse();
            const Geometry::TransformationMatrix& tip_transform = arm->actualTipPose();

            const Geometry::TransformationMatrix transform = vb_transform_inv * tip_transform;
            message.pose = this->_poseFromTransformationMatrix(transform);
            
            publisher->publish(message);
        };

    _sim->addCallback(1.0/this->get_parameter("publish_rate_hz").as_double(), callback);
}


void VirtuosoSimBridge::_setupArmNetForcePublisher(const Sim::VirtuosoArm* arm, rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr publisher)
{
    auto callback =
        [this, arm, publisher]() -> void {
            const Vec3r net_force = arm->filteredCollisionForce();  // get the force being used by the quasistatic model

            const Geometry::CoordinateFrame& vb_frame = this->_sim->virtuosoRobot()->VBFrame();
            const Geometry::TransformationMatrix vb_transform_inv = vb_frame.transform().inverse();

            const Vec3r vb_force = vb_transform_inv.rotMat()*net_force;

            auto message = geometry_msgs::msg::Vector3Stamped();
            message.header.stamp = this->now();
            message.header.frame_id = "ves/left/base";
            message.vector.x = vb_force[0];
            message.vector.y = vb_force[1];
            message.vector.z = vb_force[2];

            publisher->publish(message);
        };
    _sim->addCallback(1.0/this->get_parameter("publish_rate_hz").as_double(), callback);
}




/** === Subscribers === */

void VirtuosoSimBridge::_setupSubscribers()
{
    // set up subscriber callback to take in joint state for arm 1
    if (_sim->virtuosoRobot()->hasArm1())
    {
        auto arm1_state_callback = 
            [this](sensor_msgs::msg::JointState::UniquePtr msg) -> void {
                const auto [ot_rot, ot_trans, it_rot, it_trans, tool] = this->_jointMsgToJointState(msg.get());

                this->_sim->setArm1JointState(ot_rot, ot_trans, it_rot, it_trans, tool);
        };

        _arm1_joint_state_subscriber = this->create_subscription<sensor_msgs::msg::JointState>("/sim/input/arm1_joint_state", 10, arm1_state_callback);
    }
    
    // set up subscriber callback to take in joint state for arm 2
    if (_sim->virtuosoRobot()->hasArm2())
    {
        auto arm2_state_callback = 
            [this](sensor_msgs::msg::JointState::UniquePtr msg) -> void {
                const auto [ot_rot, ot_trans, it_rot, it_trans, tool] = this->_jointMsgToJointState(msg.get());

                this->_sim->setArm2JointState(ot_rot, ot_trans, it_rot, it_trans, tool);
        };

        _arm2_joint_state_subscriber = this->create_subscription<sensor_msgs::msg::JointState>("/sim/input/arm2_joint_state", 10, arm2_state_callback);
    }

    // set up subscriber callback to take in tip position for arm 1
    if (_sim->virtuosoRobot()->hasArm1())
    {
        auto arm1_tip_pos_callback = 
            [this](geometry_msgs::msg::PoseStamped::UniquePtr msg) -> void {
                std::string source_frame = msg->header.frame_id;
                std::string target_frame = "ves/left/base";

                Vec3r local_pos(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
                Mat3r local_rot = MathUtils::quatToMat(Vec4r(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w));
                
                // get the latest transform (if it exists)
                if (source_frame != target_frame)
                {
                    if (this->_tf_buffer->canTransform(target_frame, source_frame, tf2::TimePointZero))
                    {
                        try
                        {
                            // try getting the transform
                            auto transform_msg = this->_tf_buffer->lookupTransform(
                                target_frame, source_frame, tf2::TimePointZero);
                            
                            geometry_msgs::msg::PoseStamped transformed;
                            tf2::doTransform(*msg, transformed, transform_msg);
                            local_pos = Vec3r(transformed.pose.position.x, transformed.pose.position.y, transformed.pose.position.z);
                            local_rot = MathUtils::quatToMat(
                                Vec4r(transformed.pose.orientation.x, transformed.pose.orientation.y, transformed.pose.orientation.z, transformed.pose.orientation.w));
                                
                        }
                        catch (tf2::TransformException& ex)
                        {
                            RCLCPP_WARN(this->get_logger(), "Error getting transform: %s", ex.what());
                        }
                    }
                    else
                    {
                        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 100,
                            "Recieved left arm commanded position in the %s frame, but the transform from %s to %s does not exist yet!", 
                            source_frame.c_str(), source_frame.c_str(), target_frame.c_str());

                        return;
                    }
                }

                const Geometry::CoordinateFrame& frame = this->_sim->virtuosoRobot()->VBFrame();
                Geometry::TransformationMatrix global_transform = frame.transform() * Geometry::TransformationMatrix(local_rot, local_pos);
                // const Vec3r global_pos = frame.transform().rotMat()*local_pos + frame.origin();
                this->_sim->setArm1TipPose(global_transform);
        };

        _arm1_tip_position_subscriber = this->create_subscription<geometry_msgs::msg::PoseStamped>("/sim/input/arm1_tip_pos", 10, arm1_tip_pos_callback);
    }

    // set up subscriber callback to take in tip position for arm2
    if (_sim->virtuosoRobot()->hasArm2())
    {
        auto arm2_tip_pos_callback =
            [this](geometry_msgs::msg::PoseStamped::UniquePtr msg) -> void {
                std::string source_frame = msg->header.frame_id;
                std::string target_frame = "ves/left/base";

                Vec3r local_pos(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
                Mat3r local_rot = MathUtils::quatToMat(Vec4r(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w));

                // get the latest transform (if it exists)
                if (source_frame != target_frame)
                {
                    if (this->_tf_buffer->canTransform(target_frame, source_frame, tf2::TimePointZero))
                    {
                        try
                        {
                            // try getting the transform
                            auto transform_msg = this->_tf_buffer->lookupTransform(
                                target_frame, source_frame, tf2::TimePointZero);
                            
                            geometry_msgs::msg::PoseStamped transformed;
                            tf2::doTransform(*msg, transformed, transform_msg);
                            local_pos = Vec3r(transformed.pose.position.x, transformed.pose.position.y, transformed.pose.position.z);
                            local_rot = MathUtils::quatToMat(
                                Vec4r(transformed.pose.orientation.x, transformed.pose.orientation.y, transformed.pose.orientation.z, transformed.pose.orientation.w));
                                
                        }
                        catch (tf2::TransformException& ex)
                        {
                            RCLCPP_WARN(this->get_logger(), "Error getting transform: %s", ex.what());
                        }
                    }
                    else
                    {
                        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 100,
                            "Recieved right arm commanded position in the %s frame, but the transform from %s to %s does not exist yet!", 
                            source_frame.c_str(), source_frame.c_str(), target_frame.c_str());

                        return;
                    }
                }

                const Geometry::CoordinateFrame& frame = this->_sim->virtuosoRobot()->VBFrame();
                // const Vec3r global_pos = frame.transform().rotMat()*local_pos + frame.origin();
                Geometry::TransformationMatrix global_transform = frame.transform() * Geometry::TransformationMatrix(local_rot, local_pos);

                this->_sim->setArm2TipPose(global_transform);
        };

        _arm2_tip_position_subscriber = this->create_subscription<geometry_msgs::msg::PoseStamped>("/sim/input/arm2_tip_pos", 10, arm2_tip_pos_callback);
    }

    // set up subscriber callback to take in tip position (with covariance) for arm 1
    if (_sim->virtuosoRobot()->hasArm1())
    {
        auto arm1_tip_pos_callback = 
            [this](geometry_msgs::msg::PoseWithCovarianceStamped::UniquePtr msg) -> void {

                std::string source_frame = msg->header.frame_id;
                std::string target_frame = "ves/left/base";

                Vec3r local_pos(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);

                // get the latest transform (if it exists)
                if (source_frame != target_frame)
                {
                    if (this->_tf_buffer->canTransform(target_frame, source_frame, tf2::TimePointZero))
                    {
                        try
                        {
                            // try getting the transform
                            auto transform_msg = this->_tf_buffer->lookupTransform(
                                target_frame, source_frame, tf2::TimePointZero);
                            
                            geometry_msgs::msg::PoseWithCovarianceStamped transformed;
                            tf2::doTransform(*msg, transformed, transform_msg);
                            local_pos = Vec3r(transformed.pose.pose.position.x, transformed.pose.pose.position.y, transformed.pose.pose.position.z);
                                
                        }
                        catch (tf2::TransformException& ex)
                        {
                            RCLCPP_WARN(this->get_logger(), "Error getting transform: %s", ex.what());
                        }
                    }
                    else
                    {
                        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 100,
                            "Recieved left arm commanded position with covariance in the %s frame, but the transform from %s to %s does not exist yet!", 
                            source_frame.c_str(), source_frame.c_str(), target_frame.c_str());

                        return;
                    }
                }

                const Geometry::CoordinateFrame& frame = this->_sim->virtuosoRobot()->VBFrame();
                const Vec3r global_pos = frame.transform().rotMat()*local_pos + frame.origin();
                this->_sim->setArm1TipPosition(global_pos);
        };

        _arm1_tip_position_covariance_subscriber = 
            this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/sim/input/arm1_tip_pos_with_covar", 10, arm1_tip_pos_callback);
    }

    // set up subscriber callback to take in tip position (with covariance) for arm 2 
    if (_sim->virtuosoRobot()->hasArm2())
    {
        auto arm2_tip_pos_callback =
            [this](geometry_msgs::msg::PoseWithCovarianceStamped::UniquePtr msg) -> void {
                std::string source_frame = msg->header.frame_id;
                std::string target_frame = "ves/left/base";

                Vec3r local_pos(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);

                // get the latest transform (if it exists)
                if (source_frame != target_frame)
                {
                    if (this->_tf_buffer->canTransform(target_frame, source_frame, tf2::TimePointZero))
                    {
                        try
                        {
                            // try getting the transform
                            auto transform_msg = this->_tf_buffer->lookupTransform(
                                target_frame, source_frame, tf2::TimePointZero);
                            
                            geometry_msgs::msg::PoseWithCovarianceStamped transformed;
                            tf2::doTransform(*msg, transformed, transform_msg);
                            local_pos = Vec3r(transformed.pose.pose.position.x, transformed.pose.pose.position.y, transformed.pose.pose.position.z);
                                
                        }
                        catch (tf2::TransformException& ex)
                        {
                            RCLCPP_WARN(this->get_logger(), "Error getting transform: %s", ex.what());
                        }
                    }
                    else
                    {
                        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 100,
                            "Recieved right arm commanded position with covariance in the %s frame, but the transform from %s to %s does not exist yet!", 
                            source_frame.c_str(), source_frame.c_str(), target_frame.c_str());

                        return;
                    }
                }

                const Geometry::CoordinateFrame& frame = this->_sim->virtuosoRobot()->VBFrame();
                const Vec3r global_pos = frame.transform().rotMat()*local_pos + frame.origin();

                this->_sim->setArm2TipPosition(global_pos);
        };

        _arm2_tip_position_covariance_subscriber = 
            this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/sim/input/arm2_tip_pos_with_covar", 10, arm2_tip_pos_callback);
    }

    // set up subscriber callback to take in tool state for arm 1
    if (_sim->virtuosoRobot()->hasArm1())
    {
        auto arm1_tool_state_callback = 
            [this](std_msgs::msg::Float64::UniquePtr msg) -> void {
                int on = msg->data != 0.0;
                this->_sim->setArm1ToolState(on);
        };

        _arm1_tool_state_subscriber = this->create_subscription<std_msgs::msg::Float64>("/sim/input/arm1_tool_state", 10, arm1_tool_state_callback);
    }

    // set up subscriber callback to take in tool state for arm 2
    if (_sim->virtuosoRobot()->hasArm2())
    {
        auto arm2_tool_state_callback = 
            [this](std_msgs::msg::Float64::UniquePtr msg) -> void {
                int on = msg->data != 0.0;
                this->_sim->setArm2ToolState(on);
        };

        _arm2_tool_state_subscriber = this->create_subscription<std_msgs::msg::Float64>("/sim/input/arm2_tool_state", 10, arm2_tool_state_callback);
    }
    
}