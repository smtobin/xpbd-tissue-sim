#include "sim_bridge/VirtuosoSimBridge.hpp"

VirtuosoSimBridge::VirtuosoSimBridge(Sim::VirtuosoSimulation* sim)
    : SimBridge<Sim::VirtuosoSimulation>(sim)
{
    _setupTransformBroadcaster();

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




/** === Publishers === */


void VirtuosoSimBridge::_setupTransformBroadcaster()
{
    _tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    // set up callback to publish transforms
    std::cout << "Setting up callback for publishing transforms with tf..." << std::endl;

    auto tf_callback = 
        [this]() -> void {
            geometry_msgs::msg::TransformStamped t;
            t.header.stamp = this->now();
            t.header.frame_id = "/ves/left/base";
            t.child_frame_id = "/sim/world";

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

        _arm1_joint_state_publisher = this->create_publisher<sensor_msgs::msg::JointState>("/output/arm1_joint_state", 10);
        _arm1_frames_publisher = this->create_publisher<geometry_msgs::msg::PoseArray>("/output/arm1_frames", 10);
        _arm1_tip_frame_publisher = this->create_publisher<geometry_msgs::msg::PoseStamped>("/output/arm1_tip_frame", 10);
        _arm1_commanded_tip_frame_publisher = this->create_publisher<geometry_msgs::msg::PoseStamped>("/output/arm1_commanded_tip_frame", 10);
        _arm1_net_force_publisher = this->create_publisher<geometry_msgs::msg::Vector3Stamped>("/output/arm1_net_force", 10);


        _setupArmJointStatePublisher(arm1, _arm1_joint_state_publisher);
        _setupArmFramesPublisher(arm1, _arm1_frames_publisher);
        _setupArmTipFramePublisher(arm1, _arm1_tip_frame_publisher);
        _setupArmCommandedTipFramePublisher(arm1, _arm1_commanded_tip_frame_publisher);
        _setupArmNetForcePublisher(arm1, _arm1_net_force_publisher);

        if (arm1->hasTool())
        {
            _arm1_tool_tip_frame_publisher = this->create_publisher<geometry_msgs::msg::PoseStamped>("/output/arm1_tool_tip_frame", 10);
            _setupArmToolTipFramePublisher(arm1, _arm1_tool_tip_frame_publisher);
        }
    }

    // set up publishers for arm2 (if it exists)
    if (_sim->virtuosoRobot()->hasArm2())
    {
        const Sim::VirtuosoArm* arm2 = _sim->virtuosoRobot()->arm2();

        _arm2_joint_state_publisher = this->create_publisher<sensor_msgs::msg::JointState>("/output/arm2_joint_state", 10);
        _arm2_frames_publisher = this->create_publisher<geometry_msgs::msg::PoseArray>("/output/arm2_frames", 10);
        _arm2_tip_frame_publisher = this->create_publisher<geometry_msgs::msg::PoseStamped>("/output/arm2_tip_frame", 10);
        _arm2_commanded_tip_frame_publisher = this->create_publisher<geometry_msgs::msg::PoseStamped>("/output/arm2_commanded_tip_frame", 10);
        _arm2_net_force_publisher = this->create_publisher<geometry_msgs::msg::Vector3Stamped>("/output/arm2_net_force", 10);

        _setupArmJointStatePublisher(arm2, _arm2_joint_state_publisher);
        _setupArmFramesPublisher(arm2, _arm2_frames_publisher);
        _setupArmTipFramePublisher(arm2, _arm2_tip_frame_publisher);
        _setupArmCommandedTipFramePublisher(arm2, _arm2_commanded_tip_frame_publisher);
        _setupArmNetForcePublisher(arm2, _arm2_net_force_publisher);

        if (arm2->hasTool())
        {
            _arm2_tool_tip_frame_publisher = this->create_publisher<geometry_msgs::msg::PoseStamped>("/output/arm2_tool_tip_frame", 10);
            _setupArmToolTipFramePublisher(arm2, _arm2_tool_tip_frame_publisher);
        }
    }
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
            message.header.frame_id = "/ves/left/base";

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

            if (arm->hasTool())
            {
                const Sim::VirtuosoArm::ToolTubeFramesArray& tt_frames = arm->toolTubeFrames();
                for (const auto& frame : tt_frames)
                {
                    const Geometry::TransformationMatrix transform = vb_transform_inv * frame.transform();
                    message.poses.push_back(this->_poseFromTransformationMatrix(transform));
                }
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
            message.header.frame_id = "/ves/left/base";

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

            const Vec3r& origin = vb_transform_inv.rotMat() * arm_commanded_position + vb_transform_inv.translation();
            message.pose.position.x = origin[0];
            message.pose.position.y = origin[1];
            message.pose.position.z = origin[2];

            /** commanded tip position has no orientation */
            message.pose.orientation.x = 0;
            message.pose.orientation.y = 0;
            message.pose.orientation.z = 0;
            message.pose.orientation.w = 1;
            
            publisher->publish(message);
        };

    _sim->addCallback(1.0/this->get_parameter("publish_rate_hz").as_double(), callback);
}


void VirtuosoSimBridge::_setupArmToolTipFramePublisher(const Sim::VirtuosoArm* arm, rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher)
{
    auto callback = 
        [this, arm, publisher]() -> void {
            const Sim::VirtuosoArm::ToolTubeFramesArray& tt_frames = arm->toolTubeFrames();

            auto message = geometry_msgs::msg::PoseStamped();
            message.header.stamp = this->now();
            message.header.frame_id = "ves/left/base";

            const Geometry::CoordinateFrame& vb_frame = this->_sim->virtuosoRobot()->VBFrame();
            const Geometry::TransformationMatrix vb_transform_inv = vb_frame.transform().inverse();
            const Geometry::CoordinateFrame& tip_frame = tt_frames.back();

            const Geometry::TransformationMatrix transform = vb_transform_inv * tip_frame.transform();
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

        _arm1_joint_state_subscriber = this->create_subscription<sensor_msgs::msg::JointState>("/input/arm1_joint_state", 10, arm1_state_callback);
    }
    
    // set up subscriber callback to take in joint state for arm 2
    if (_sim->virtuosoRobot()->hasArm2())
    {
        auto arm2_state_callback = 
            [this](sensor_msgs::msg::JointState::UniquePtr msg) -> void {
                const auto [ot_rot, ot_trans, it_rot, it_trans, tool] = this->_jointMsgToJointState(msg.get());

                this->_sim->setArm2JointState(ot_rot, ot_trans, it_rot, it_trans, tool);
        };

        _arm2_joint_state_subscriber = this->create_subscription<sensor_msgs::msg::JointState>("/input/arm2_joint_state", 10, arm2_state_callback);
    }

    // set up subscriber callback to take in tip position for arm 1
    if (_sim->virtuosoRobot()->hasArm1())
    {
        auto arm1_tip_pos_callback = 
            [this](geometry_msgs::msg::PoseStamped::UniquePtr msg) -> void {
                const Vec3r local_pos(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);

                const Geometry::CoordinateFrame& frame = this->_sim->virtuosoRobot()->VBFrame();
                const Vec3r global_pos = frame.transform().rotMat()*local_pos + frame.origin();
                this->_sim->setArm1TipPosition(global_pos);
        };

        _arm1_tip_position_subscriber = this->create_subscription<geometry_msgs::msg::PoseStamped>("/input/arm1_tip_pos", 10, arm1_tip_pos_callback);
    }

    // set up subscriber callback to take in tip position for arm2
    if (_sim->virtuosoRobot()->hasArm2())
    {
        auto arm2_tip_pos_callback =
            [this](geometry_msgs::msg::PoseStamped::UniquePtr msg) -> void {
                const Vec3r local_pos(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);

                const Geometry::CoordinateFrame& frame = this->_sim->virtuosoRobot()->VBFrame();
                const Vec3r global_pos = frame.transform().rotMat()*local_pos + frame.origin();

                this->_sim->setArm2TipPosition(global_pos);
        };

        _arm2_tip_position_subscriber = this->create_subscription<geometry_msgs::msg::PoseStamped>("/input/arm2_tip_pos", 10, arm2_tip_pos_callback);
    }

    // set up subscriber callback to take in tool state for arm 1
    if (_sim->virtuosoRobot()->hasArm1())
    {
        auto arm1_tool_state_callback = 
            [this](std_msgs::msg::Int8::UniquePtr msg) -> void {
                this->_sim->setArm1ToolState(msg->data);
        };

        _arm1_tool_state_subscriber = this->create_subscription<std_msgs::msg::Int8>("/input/arm1_tool_state", 10, arm1_tool_state_callback);
    }

    // set up subscriber callback to take in tool state for arm 2
    if (_sim->virtuosoRobot()->hasArm2())
    {
        auto arm2_tool_state_callback = 
            [this](std_msgs::msg::Int8::UniquePtr msg) -> void {
                this->_sim->setArm2ToolState(msg->data);
        };

        _arm2_tool_state_subscriber = this->create_subscription<std_msgs::msg::Int8>("/intput/arm2_tool_state", 10, arm2_tool_state_callback);
    }
    
}