from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description(vesselids = ['RAS_TN_DB','RAS_TN_OR','RAS_TN_GR']):
    ld = LaunchDescription()

    # Print: start formation control launch generation:
    print("Start formation control launch generation")
    foo_dir = get_package_share_directory('ras_ros_core_control_modules')
    included_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(foo_dir + '/fleet_geo_utils.launch.py'))
                
    ld.add_action(included_launch)

    # Bring up the control stack for each vessel
    for vesselid in vesselids:
        turtleboat_node = Node(
            package='turtleboat',
            executable='turtleboatmain',
            name='turtleboat_sim',
            namespace=vesselid,
            parameters=[{'rate_publish_position':20.0},
                        {'rate_publish_heading':17.0}],
            output='screen',
            emulate_tty=True,
        )
        ld.add_action(turtleboat_node)

        # Start vessel reference generator for each vessel
        ref_generator_node = Node(
            package='usv_formation_control_1',
            executable='vessel_reference_generator',
            name='vessel_reference_generator',
            namespace=vesselid,
            arguments=[vesselid,'1.8'],
            output='screen',
            emulate_tty=True,
        )
        ld.add_action(ref_generator_node)
        
        # Start vessel heading controller for each vessel
        heading_controller_node = Node(
            package=['ras_ros_core_control_modules'],
            executable='heading_controller',
            name='heading_controller',
            namespace=vesselid,
            arguments=[vesselid],
            output='screen',
            emulate_tty=True,
        )
        ld.add_action(heading_controller_node)
        
        # Start vessel surge velocity controller for each vessel
        surge_controller_node = Node(
            package=['ras_ros_core_control_modules'],
            executable='surge_vel_controller',
            name='surge_velocity_controller',
            namespace=vesselid,
            arguments=[vesselid],
            output='screen',
            emulate_tty=True,
        )
        ld.add_action(surge_controller_node)
        
        # Start thrust allocator for each vessel
        thrust_allocator_node = Node(
            package=['ras_ros_core_control_modules'],
            executable='TN_control_effort_allocator_nomoto',
            name='thrust_allocator',
            namespace=vesselid,
            arguments=[vesselid],
            output='screen',
            emulate_tty=True,
        )
        ld.add_action(thrust_allocator_node)

        # Start velocity differentiator for each vessel
        velocity_differentiator_node = Node(
            package=['ras_ros_core_control_modules'],
            executable='vel_differentiator',
            name='velocity_differentiator',
            namespace=vesselid,
            arguments=[vesselid],
            output='screen',
            emulate_tty=True,
        )
        ld.add_action(velocity_differentiator_node)

    # Start formation configuration broadcaster
    formation_config_broadcaster_node = Node(
        package='usv_formation_control_1',
        executable='formation_configuration_broadcaster_1',
        name='formation_config_broadcaster',
        namespace='formation1',
        arguments=['formation1'],
        output='screen',
        emulate_tty=True,
    )
    ld.add_action(formation_config_broadcaster_node)

    # start formation trajectory planner
    formation_trajectory_planner_node = Node(
        package='usv_formation_control_1',
        executable='formation_trajectory_planner',
        name='formation_trajectory_planner',
        namespace='formation1',
        arguments=['formation1'],
        output='screen',
        emulate_tty=True,
    )
    ld.add_action(formation_trajectory_planner_node)

    # Start Rviz2 launcher
    rviz2launchdescription = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('ras_urdf_common'),
            '/launch/rviz_bringup.launch.py'])

        )
    ld.add_action(rviz2launchdescription)
    

    return ld