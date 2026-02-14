import os
import sys
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction, SetEnvironmentVariable, LogInfo

def generate_launch_description():
    # Get package directory
    pkg_share = get_package_share_directory('diffRobot')
    
    # Get parent directory for package resolution
    import pathlib
    pkg_parent = str(pathlib.Path(pkg_share).parent)
    
    # Debug output
    print(f"Package share: {pkg_share}")
    print(f"Package parent: {pkg_parent}")
    
    # URDF path
    urdf_path = os.path.join(pkg_share, 'urdf', 'diffRobot.urdf')
    print(f"URDF path: {urdf_path}")
    
    # Read URDF
    with open(urdf_path, 'r') as f:
        robot_description = f.read()
    
    print("Converting mesh paths to absolute paths...")
    
    import re
    
    def replace_package_path(match):
        mesh_filename = match.group(1)
        # Convert package://diffRobot/meshes/... to absolute path
        absolute_path = os.path.join(pkg_share, 'meshes', os.path.basename(mesh_filename))
        print(f"Converting {mesh_filename} -> file://{absolute_path}")
        return f'filename="file://{absolute_path}"'
    
    # Replace all package:// URIs with file:// absolute paths
    robot_description = re.sub(
        r'filename="package://diffRobot/meshes/([^"]+)"',
        replace_package_path,
        robot_description
    )
    
    # Also replace $(find diffRobot) format if present
    robot_description = re.sub(
        r'filename="file://\$\(find diffRobot\)/meshes/([^"]+)"',
        replace_package_path,
        robot_description
    )
    
    # Note: Your URDF already has Gazebo extensions
    robot_description_with_gazebo = robot_description
    
    # Set world file path
    world_path = os.path.join(pkg_share, 'worlds', 'enclosed_obstacles.world')
    rviz_config_path = os.path.join(pkg_share, 'rviz', 'diffRobot.rviz')
    
    # Build resource paths
    current_gz_path = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    current_ign_path = os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '')

    # ros-gz bridge config path
    bridge_config_path = os.path.join(pkg_share, 'config', 'ros_gz_bridge_config.yaml')
    print(f"Bridge config path: {bridge_config_path}")
    
    # Add both the package share and its parent
    new_gz_path = f'{pkg_share}:{pkg_parent}'
    if current_gz_path:
        new_gz_path = f'{new_gz_path}:{current_gz_path}'
    
    new_ign_path = f'{pkg_share}:{pkg_parent}'
    if current_ign_path:
        new_ign_path = f'{new_ign_path}:{current_ign_path}'
    
    print(f"GZ_SIM_RESOURCE_PATH will be: {new_gz_path}")
    print(f"IGN_GAZEBO_RESOURCE_PATH will be: {new_ign_path}")
    
    # Debug: Check if meshes exist
    meshes_dir = os.path.join(pkg_share, 'meshes')
    if os.path.exists(meshes_dir):
        print(f"Meshes directory: {meshes_dir}")
        print("Files in meshes directory:")
        for f in os.listdir(meshes_dir):
            full_path = os.path.join(meshes_dir, f)
            print(f"  - {f} (exists: {os.path.exists(full_path)})")
    else:
        print(f"ERROR: Meshes directory not found: {meshes_dir}")
    
    return LaunchDescription([
        # Set environment variables for Gazebo - CRITICAL FIX
        SetEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH', 
            value=new_gz_path
        ),
        SetEnvironmentVariable(
            name='IGN_GAZEBO_RESOURCE_PATH',
            value=new_ign_path
        ),
        SetEnvironmentVariable(
            name='GAZEBO_MODEL_PATH',
            value=f'{pkg_share}/models:{os.environ.get("GAZEBO_MODEL_PATH", "")}'
        ),

          # Publish robot description FIRST
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description_with_gazebo,
                'use_sim_time': True
            }]
        ),

       
        # Start Gazebo with explicit environment
        # Added flags to prevent stepping/pausing issues
        ExecuteProcess(
            cmd=['gz', 'sim', '-v', '3', world_path], 
            output='screen',
            shell=True,
            additional_env={
                'GZ_SIM_RESOURCE_PATH': new_gz_path,
                'IGN_GAZEBO_RESOURCE_PATH': new_ign_path,
                'GAZEBO_MODEL_PATH': f'{pkg_share}/models:{os.environ.get("GAZEBO_MODEL_PATH", "")}'
            }
        ),

        # TimerAction(
        #     period=3.0,
        #     actions=[
        #         Node(
        #             package='ros_gz_sim',
        #             executable='create',
        #             arguments=[
        #                 '-topic', '/robot_description',
        #                 '-name', 'diffRobot',
        #                 '-z', '0.5'  # Higher spawn height
        #             ],
        #             output='screen'
        #         )
        #     ]
        # ),


                     
        # Spawn robot after delay
        TimerAction(
            period=3.0,
            actions=[
                ExecuteProcess(
                    cmd=[
                        'ros2', 'run', 'ros_gz_sim', 'create',
                        '-topic', '/robot_description',
                        '-name', 'diffRobot',
                        '-z', '0.1'
                    ],
                    output='screen'
                )
            ]
        ),
        
        TimerAction(
            period=5.0,  # Wait for Gazebo to start
            actions=[
                Node(
                    package='ros_gz_bridge',
                    executable='parameter_bridge',
                    name='ros_gz_bridge',
                    parameters=[{'config_file': bridge_config_path}],
                    output='screen'
                )
            ]
        ),

        # # Sensor fusion with ekf 
        TimerAction(
            period=10.0,  # Wait 8 seconds for bridge to be ready
            actions=[
                Node(
                    package='robot_localization',
                    executable='ekf_node',
                    name='ekf_filter_node',
                    output='screen',
                    parameters=[os.path.join(pkg_share, 'config', 'ekf.yaml')],
                    remappings=[
                        ('odometry/filtered', '/odometry/filtered')
                    ]
                )
            ]
        ),

       
    ])