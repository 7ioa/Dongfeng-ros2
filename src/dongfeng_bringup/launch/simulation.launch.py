"""Start the Dongfeng sandbox in Gazebo Sim with the differential-drive car.

Brings up, in one go:

  * Gazebo Sim running worlds/dongfeng.sdf (the static sandbox scene)
  * the dongfeng_car robot, spawned as a separate model
  * robot_state_publisher, from the URDF in dongfeng_description
  * the ros_gz bridge for /clock, /cmd_vel, /odom, /tf and /joint_states

The scene itself is never modified: the robot is a separate entity spawned at
run time through the Gazebo create service, so the world file stays exactly as
generate_scene.py wrote it.

Typical use:

    ros2 launch dongfeng_bringup simulation.launch.py

On a VMware guest where Ogre2 flickers, keep the previously validated Ogre 1
path (see README section 3):

    QT_QPA_PLATFORM=xcb ros2 launch dongfeng_bringup simulation.launch.py \\
        render_engine:=ogre

Headless, for scripted checks without a GUI:

    ros2 launch dongfeng_bringup simulation.launch.py headless:=true
"""

import os
import shlex
from pathlib import Path
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            OpaqueFunction, SetEnvironmentVariable,
                            RegisterEventHandler, EmitEvent, LogInfo)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _find_scene_root():
    """Return the project folder holding worlds/dongfeng.sdf, or None.

    Checked in order: $DONGFENG_SANDBOX_ROOT, then each parent of this file.
    Walking the parents works both from the source tree and from the installed
    copy under <workspace>/install/.../dongfeng_bringup/launch/.
    """
    candidates = []
    override = os.environ.get('DONGFENG_SANDBOX_ROOT')
    if override:
        candidates.append(Path(override))
    candidates.extend(Path(__file__).resolve().parents)
    for base in candidates:
        if (base / 'worlds' / 'dongfeng.sdf').is_file():
            return base
    return None


def _launch_setup(context, *args, **kwargs):
    def value(name):
        return LaunchConfiguration(name).perform(context)

    def flag(name):
        return value(name).strip().lower() in ('1', 'true', 'yes', 'on')

    world = value('world')
    if not Path(world).is_file():
        raise RuntimeError(f'World file not found: {world}')
    world_element = ET.parse(world).getroot().find('world')
    if world_element is None or not world_element.get('name'):
        raise RuntimeError(f'SDF has no named world: {world}')

    # Gazebo command line. Flags first, world file last.
    gz_args = []
    if flag('headless'):
        gz_args.append('-s')
    gz_args.append('-r')
    if value('render_engine').strip():
        gz_args += ['--render-engine', value('render_engine').strip()]
    if flag('verbose'):
        gz_args += ['-v', '4']
    gz_args.append(world)

    use_sim_time = flag('use_sim_time')
    robot_name = value('robot_name')

    description_share = get_package_share_directory('dongfeng_description')
    xacro_file = os.path.join(description_share, 'urdf', 'dongfeng_car.urdf.xacro')
    bridge_config = os.path.join(
        get_package_share_directory('dongfeng_bringup'), 'config', 'bridge.yaml')

    models_path = value('models_path')
    resource_path = os.pathsep.join(
        part for part in (models_path, os.path.dirname(description_share),
                          os.environ.get('GZ_SIM_RESOURCE_PATH', ''))
        if part)

    guard = Node(
        package='dongfeng_bringup', executable='command_guard',
        name='command_guard', output='screen',
        parameters=[{'use_sim_time': use_sim_time,
                     'command_timeout': float(value('command_timeout'))}])

    spawn = Node(
        package='ros_gz_sim', executable='create', name='spawn_dongfeng_car',
        output='screen',
        arguments=['-world', world_element.get('name'), '-name', robot_name,
                   '-allow_renaming', 'false', '-topic', 'robot_description',
                   '-x', value('x'), '-y', value('y'), '-z', value('z'),
                   '-Y', value('yaw')])

    def spawned(event, _context):
        if event.returncode != 0:
            return [EmitEvent(event=Shutdown(reason='Robot spawn failed; see create output.'))]
        return [LogInfo(msg='小车已加入地图。另开终端运行 bash keyboard_control.sh。')]

    return [
        # Must be set before gz sim starts, so model://dongfeng_sandbox resolves.
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', resource_path),
        SetEnvironmentVariable('QT_QPA_PLATFORM', os.environ.get('QT_QPA_PLATFORM', 'xcb')),

        RegisterEventHandler(OnProcessExit(target_action=guard,
            on_exit=[EmitEvent(event=Shutdown(reason='Command guard stopped.'))])),
        RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=spawned)),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('ros_gz_sim'), 'launch',
                'gz_sim.launch.py')),
            launch_arguments=[('gz_args', shlex.join(gz_args))]),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                # Wrapped so launch_ros keeps the whole URDF as one string
                # instead of trying to read it as YAML.
                'robot_description': ParameterValue(
                    Command(['xacro ', shlex.quote(xacro_file)]), value_type=str),
                'use_sim_time': use_sim_time,
            }]),

        # Waits for this world's create service and robot_description topic.
        spawn,
        guard,

        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='ros_gz_bridge',
            output='screen',
            parameters=[{
                'config_file': bridge_config,
                'use_sim_time': use_sim_time,
            }]),
    ]


def generate_launch_description():
    scene_root = _find_scene_root()
    if scene_root is None:
        raise RuntimeError(
            'Could not locate worlds/dongfeng.sdf. Run this from inside the '
            'dongfeng_sandbox workspace, or set DONGFENG_SANDBOX_ROOT to the '
            'project folder.')

    # Middle of the front straight of the perimeter road, facing +X. The road
    # deck is 0.30 m wide there and its surface sits at z = 0, so base_link
    # starts one wheel radius (0.028 m) up plus a small settling margin.
    spawn_defaults = {
        'x': '1.65',
        'y': '0.19',
        'z': '0.031',
        'yaw': '0.0',
    }

    return LaunchDescription([
        DeclareLaunchArgument(
            'world', default_value=str(scene_root / 'worlds' / 'dongfeng.sdf'),
            description='Sandbox world file to load.'),
        DeclareLaunchArgument(
            'models_path', default_value=str(scene_root / 'models'),
            description='Folder holding model://dongfeng_sandbox; exported as '
                        'GZ_SIM_RESOURCE_PATH.'),
        DeclareLaunchArgument(
            'robot_name', default_value='dongfeng_car',
            description='Name of the spawned Gazebo model.'),
        DeclareLaunchArgument(
            'x', default_value=spawn_defaults['x'],
            description='Spawn X on the front straight.'),
        DeclareLaunchArgument(
            'y', default_value=spawn_defaults['y'],
            description='Spawn Y, on the road centre line.'),
        DeclareLaunchArgument(
            'z', default_value=spawn_defaults['z'],
            description='Spawn Z, one wheel radius above the road deck.'),
        DeclareLaunchArgument(
            'yaw', default_value=spawn_defaults['yaw'],
            description='Spawn yaw in radians, 0 faces +X along the road.'),
        DeclareLaunchArgument(
            'render_engine', default_value='ogre',
            description="Gazebo render engine. Leave empty for the world's own "
                        "setting, or use 'ogre' on VMware guests."),
        DeclareLaunchArgument(
            'headless', default_value='false',
            description='Run the Gazebo server only, without a GUI (for '
                        'scripted tests).'),
        DeclareLaunchArgument(
            'verbose', default_value='false',
            description='Raise the Gazebo log level to 4.'),
        DeclareLaunchArgument(
            'command_timeout', default_value='0.4',
            description='Wall-time seconds without velocity input before stopping.'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Run the ROS 2 nodes on Gazebo sim time.'),
        OpaqueFunction(function=_launch_setup),
    ])
