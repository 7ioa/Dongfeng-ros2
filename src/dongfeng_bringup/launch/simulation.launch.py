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
import re
import shlex
import tempfile
from pathlib import Path
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            OpaqueFunction, SetEnvironmentVariable, GroupAction,
                            RegisterEventHandler, EmitEvent, LogInfo)
from launch.event_handlers import OnProcessExit, OnShutdown
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


def _prepare_gui_config(world_element, source):
    """Add the material fix without editing the world or a user's GUI file."""
    if source:
        # Gazebo GUI files are XML fragments, often with an XML declaration.
        text = Path(source).read_text()
        text = re.sub(r'^\s*<\?xml[^>]*\?>', '', text, count=1)
        gui = ET.fromstring('<gui>' + text + '</gui>')
    else:
        gui = world_element.find('gui')
        if gui is None:
            # Preserve Gazebo's default GUI selection for unrelated worlds.
            return None
    filename = 'TrafficLightMaterialSync'
    if not any(p.get('filename') == filename for p in gui.findall('plugin')):
        plugin = ET.SubElement(gui, 'plugin', filename=filename,
                               name='Traffic light material sync')
        properties = ET.SubElement(plugin, 'gz-gui')
        for key, kind, value in [('state', 'string', 'floating'),
                                 ('showTitleBar', 'bool', 'false'),
                                 ('width', 'double', '1'),
                                 ('height', 'double', '1')]:
            ET.SubElement(properties, 'property', key=key, type=kind).text = value
    with tempfile.NamedTemporaryFile(mode='w', prefix='dongfeng_gui_',
                                     suffix='.config', delete=False) as config:
        config.write(''.join(ET.tostring(child, encoding='unicode') for child in gui))
        return config.name


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

    gui_config = value('gui_config')
    gui_actions = []
    if not flag('headless'):
        generated_config = _prepare_gui_config(world_element, gui_config)
        if generated_config:
            gui_config = generated_config

            def cleanup_gui(_context):
                Path(generated_config).unlink(missing_ok=True)
                return []

            gui_actions.append(RegisterEventHandler(OnShutdown(
                on_shutdown=[OpaqueFunction(function=cleanup_gui)])))
        plugin_path = str(Path(get_package_prefix('dongfeng_bringup')) /
                          'lib' / 'dongfeng_bringup')
        gui_actions.append(SetEnvironmentVariable('GZ_GUI_PLUGIN_PATH',
            os.pathsep.join(filter(None, [plugin_path,
                os.environ.get('GZ_GUI_PLUGIN_PATH', '')]))))

    # Gazebo command line. Flags first, world file last.
    gz_args = []
    software_sensors=flag('sensor_software_rendering')
    if flag('headless') or software_sensors:
        gz_args.append('-s')
    if flag('headless_rendering'):
        if software_sensors or os.environ.get('LIBGL_ALWAYS_SOFTWARE') == '1':
            raise RuntimeError('软件渲染请使用 headless:=true，不要设置 headless_rendering:=true；EGL 会强制选择硬件设备。')
        gz_args.append('--headless-rendering')
    gz_args.append('-r')
    if value('render_engine').strip():
        gz_args += ['--render-engine', value('render_engine').strip()]
    gz_args += ['--render-engine-server', value('sensor_render_engine')]
    if gui_config:
        gz_args += ['--gui-config', gui_config]
    if flag('verbose'):
        gz_args += ['-v', '4']
    gz_args.append(world)

    gz_launch=PythonLaunchDescriptionSource(os.path.join(
        get_package_share_directory('ros_gz_sim'),'launch','gz_sim.launch.py'))
    server=IncludeLaunchDescription(gz_launch,
        launch_arguments={'gz_args':shlex.join(gz_args),'on_exit_shutdown':'true'}.items())
    gazebo_actions=[server]
    if software_sensors:
        # Mesa is needed for sensor images on VMware, but must not slow down
        # the visible scene. Keep the GUI environment identical to launch_car.sh.
        gazebo_actions=[GroupAction(actions=[
            SetEnvironmentVariable('LIBGL_ALWAYS_SOFTWARE','1'),server])]
        if not flag('headless'):
            gui_args=['-g','--render-engine',value('render_engine')]
            if gui_config:gui_args+=['--gui-config', gui_config]
            gazebo_actions.append(GroupAction(actions=[IncludeLaunchDescription(
                gz_launch,launch_arguments={'gz_args':shlex.join(gui_args),
                    'on_exit_shutdown':'true'}.items())]))

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
    arbiter = Node(package='dongfeng_autonomy', executable='arbiter_node',
                   name='command_arbiter', output='screen',
                   parameters=[{'use_sim_time': use_sim_time,
                                'start_enabled': flag('auto_mode')}])

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
        *gui_actions,

        RegisterEventHandler(OnProcessExit(target_action=guard,
            on_exit=[EmitEvent(event=Shutdown(reason='Command guard stopped.'))])),
        RegisterEventHandler(OnProcessExit(target_action=arbiter,
            on_exit=[EmitEvent(event=Shutdown(reason='Command arbiter stopped.'))])),
        RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=spawned)),

        *gazebo_actions,

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
        arbiter,

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
        DeclareLaunchArgument('gui_config',default_value='',description='Optional GUI configuration for validation.'),
        DeclareLaunchArgument('auto_mode', default_value='false',
                              description='Enable automatic input at startup.'),
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
            'sensor_software_rendering', default_value='false',
            description='Use Mesa for server sensors only; preserve GUI rendering.'),
        DeclareLaunchArgument(
            'sensor_render_engine', default_value='ogre2',
            description='Server rendering for camera and GPU lidar.'),
        DeclareLaunchArgument(
            'headless', default_value='false',
            description='Run the Gazebo server only, without a GUI (for '
                        'scripted tests).'),
        DeclareLaunchArgument(
            'headless_rendering', default_value='false',
            description='Use EGL rendering for sensors without a display (Ogre2).'),
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
