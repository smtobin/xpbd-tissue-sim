from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    config_file_arg = DeclareLaunchArgument(
        'config_filename',
        default_value=TextSubstitution(text='../config/demos/virtuoso_prostate/focal_lesion.yaml'),
        description='Path to the simulation config file.'
    )

    simulation_type_arg = DeclareLaunchArgument(
        'simulation_type',
        default_value=TextSubstitution(text='FocalLesionSimulation'),
        description='The type of simulation to create.'
    )

    prostate_mesh_filename_arg = DeclareLaunchArgument(
        'prostate_mesh_filename',
        default_value = TextSubstitution(text=''),
        description='The file path of the .msh file to use as the CT deformable geometry instead of the one specified in the config file. Assumes there is also a file with the same name with _fixed_faces.txt that specifies which vertices/faces are fixed. E.g. dir/mesh.msh would have dir/mesh_fixed_faces.txt.'
    )

    CTtoVBTranslation_arg = DeclareLaunchArgument(
        'CT_to_VB_translation',
        default_value = TextSubstitution(text='[0.1, 0.1, 0.15]'),
        description='Specifies the translation part of the (initial) CT -> VB transform, i.e. it transforms a point in the CT frame into the VB frame. Specified in meters.'
    )

    CTtoVBRotation_arg = DeclareLaunchArgument(
        'CT_to_VB_rotation',
        default_value = TextSubstitution(text='[0.6, 0, 0]'),
        description='Specifies the rotation part of the (initial) CT -> VB transform. Specified as XYZ Euler angles, in degrees.'
    )

    # the sim_bridge node
    sim_bridge_node = Node(
        package='sim_bridge',
        executable='sim_bridge_node',
        name='sim_bridge',
        output='screen',
        remappings=[
            # === Remappings for simulation inputs === 
            # ('/sim/input/arm1_joint_state', '/ves/left/joint/servo_jp'),      # Virtuoso arms can be controlled by joint position or Cartesian tip position
            # ('/sim/input/arm2_joint_state', '/ves/right/joint/servo_jp'),


            ### USE MEASURED_CP WHEN RUNNING WITH REAL ROBOT OR VES_KEYBOARD
            # these topics subscribe to a PoseStamped topic
            # and will automatically apply the transform from the frame they are expressed in to ves/left/base
            # (provided that a tf transform exists relating the two frames)
            # ('/sim/input/arm1_tip_pos', '/ves/left/joint/measured_cp'),
            # ('/sim/input/arm2_tip_pos', '/ves/right/joint/measured_cp'),

            ### OR, USE SMOOTHER OUTPUT
            # these topics subscribe to PoseWithCovarianceStamped topic
            # and will automatically apply the transform from the frame they are expressed in to ves/left/base
            # (provided that a tf transform exists relating the two frames)
            ('/sim/input/arm1_tip_pos_with_covar', '/ves_smoother/left/tip_pose'),
            ('/sim/input/arm2_tip_pos_with_covar', '/ves_smoother/right/tip_pose'),

            # USE SERVO_CP WHEN RUNNING WITH TRAINING
            # ('/sim/input/arm1_tip_pos', '/ves/left/joint/servo_cp'),
            # ('/sim/input/arm2_tip_pos', '/ves/right/joint/servo_cp'),
            
            ('/sim/input/arm1_tool_state', '/ves/left/set_tool'),
            ('/sim/input/arm2_tool_state', '/ves/right/set_tool'),


            # === Remappings for simulation outputs ===
            # ('/sim/output/arm1_tip_frame', '/ves/left/joint/measured_cp'),
            # ('/sim/output/arm2_tip_frame', '/ves/right/joint/mesaured_cp'),
            # ('/sim/output/arm1_commanded_tip_frame', '/ves/left/joint/setpoint_cp'),
            # ('/sim/output/arm2_commanded_tip_frame', '/ves/right/joint/setpoint_cp'),
            # ('/sim/output/arm1_frames', '/sim/arm1_frames'),
            # ('/sim/output/arm2_frames', '/sim/arm2_frames'),
            # ('/sim/output/tissue_mesh', '/sim/tissue_mesh'),
            # ('/sim/output/tissue_mesh_vertices', '/sim/tissue_mesh_vertices'),
            # ('/sim/output/partial_view_pc', '/sim/partial_view_pc')
        ],
        parameters=[
            {"use_wall_time_for_publishing": True},   # if true, the rate of publishing will be in terms of the wall time (which may not align with simulated time)
            {"publish_rate_hz": 10.0},      # publish rate of topics
            {"publish_matrices": False},     # whether or not to publish mesh matrices, i.e. stiffness matrix, vertices, faces, and elements
            {"image_publish_rate_hz": 5.0},    # publish rate of the image topic
            {"publish_images": False},       # whether or not to publish the rendered simulation image
            {"partial_view_pc": True},      # whether or not to publish partial-view point cloud
            {"partial_view_pc_hfov": 80.0},   # degrees
            {"partial_view_pc_vfov": 50.0},   # degrees
            {"partial_view_pc_sample_density": 1.0},   # rays per degree (i.e. higher = denser point cloud)

            {"CT_frame_name": "ct/base"},   # name of the CT origin frame in the tf tree
            {"cam_frame_name": "ves/camera"}    # name of the robot camera frame in the tf tree
        ],
        arguments=[
            '--config-filename', LaunchConfiguration('config_filename'),
            '--simulation-type', LaunchConfiguration('simulation_type'),
            '--prostate-mesh-filename', LaunchConfiguration('prostate_mesh_filename'),
            '--CT-to-VB-translation', LaunchConfiguration('CT_to_VB_translation'),
            '--CT-to-VB-rotation', LaunchConfiguration('CT_to_VB_rotation')
        ]
    )

    # Include rosbridge_server launch file
    rosbridge_launch_file = PathJoinSubstitution([
        FindPackageShare('rosbridge_server'),
        'launch',
        'rosbridge_websocket_launch.xml'
    ])
    
    # Create the include launch description action
    rosbridge_server = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(rosbridge_launch_file),
        launch_arguments={
            'delay_between_messages': '0.0',
        }.items()
    )

    ld = LaunchDescription([
        config_file_arg,
        simulation_type_arg,
        prostate_mesh_filename_arg,
        CTtoVBTranslation_arg,
        CTtoVBRotation_arg,
        sim_bridge_node,
        rosbridge_server
    ])

    return ld