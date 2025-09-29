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
        default_value=TextSubstitution(text='/workspace/config/demos/virtuoso_trachea/virtuoso_trachea.yaml'),
        description='Absolute path to the simulation config file.'
    )

    simulation_type_arg = DeclareLaunchArgument(
        'simulation_type',
        default_value=TextSubstitution(text='VirtuosoTissueGraspingSimulation'),
        description='The type of simulation to create.'
    )

    # the sim_bridge node
    sim_bridge_node = Node(
        package='sim_bridge',
        executable='sim_bridge_node',
        name='sim_bridge',
        output='screen',
        remappings=[
            ('/input/arm1_joint_state', '/ves/left/joint/servo_jp'),
            ('/input/arm2_joint_state', '/ves/right/joint/servo_jp'),
            ('/input/arm1_tip_pos', '/ves/left/joint/servo_cp'),
            ('/input/arm2_tip_pos', '/ves/right/joint/servo_cp'),
            ('/input/arm1_tool_state', '/ves/left/set_tool'),
            ('/input/arm2_tool_state', '/ves/right/set_tool'),
            ('/output/arm1_tip_frame', '/ves/left/joint/measured_cp'),
            ('/output/arm2_tip_frame', '/ves/right/joint/mesaured_cp'),
            ('/output/arm1_commanded_tip_frame', '/ves/left/joint/setpoint_cp'),
            ('/output/arm2_commanded_tip_frame', '/ves/right/joint/setpoint_cp'),
            ('/output/arm1_frames', '/sim/arm1_frames'),
            ('/output/arm2_frames', '/sim/arm2_frames'),
            ('/output/tissue_mesh', '/sim/tissue_mesh'),
            ('/output/tissue_mesh_vertices', '/sim/tissue_mesh_vertices'),
            ('/output/partial_view_pc', '/sim/partial_view_pc')
        ],
        parameters=[
            {"publish_rate_hz": 100.0},      # publish rate of topics
            {"publish_matrices": True},
            {"partial_view_pc": True},      # whether or not to publish partial-view point cloud
            {"partial_view_pc_hfov": 80.0},   # degrees
            {"partial_view_pc_vfov": 50.0},   # degrees
            {"partial_view_pc_sample_density": 1.0}   # rays per degree (i.e. higher = denser point cloud)
        ],
        arguments=[
            '--config-filename', LaunchConfiguration('config_filename'),
            '--simulation-type', LaunchConfiguration('simulation_type')
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
        AnyLaunchDescriptionSource(rosbridge_launch_file)
    )

    ld = LaunchDescription([
        config_file_arg,
        simulation_type_arg,
        sim_bridge_node,
        rosbridge_server
    ])

    return ld