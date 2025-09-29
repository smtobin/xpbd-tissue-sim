from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node

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
            {"use_wall_time_for_publishing": True},   # if true, the rate of publishing will be in terms of the wall time (which may not align with simulated time)
            {"publish_rate_hz": 10.0},      # publish rate of topics
            {"publish_matrices": True},     # whether or not to publish mesh matrices, i.e. stiffness matrix, vertices, faces, and elements
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

    ld = LaunchDescription([
        config_file_arg,
        simulation_type_arg,
        sim_bridge_node,
    ])

    return ld