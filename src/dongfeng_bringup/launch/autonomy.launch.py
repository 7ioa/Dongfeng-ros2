"""Full sensor-based lap; GUI and sensor rendering are independently selectable."""
import json
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, EmitEvent, OpaqueFunction
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _simulation_launch(context):
    share=Path(get_package_share_directory('dongfeng_bringup'))
    arguments={k:LaunchConfiguration(k) for k in (
        'gui_config','headless','headless_rendering','render_engine',
        'sensor_render_engine','sensor_software_rendering','driving_mode')}
    arguments['auto_mode']='true'
    if LaunchConfiguration('mission').perform(context)=='full_demo':
        from dongfeng_autonomy.mission import Mission
        config=Path(get_package_share_directory('dongfeng_autonomy'))/'config'
        mission=Mission.load(config/'full_demo.json',json.loads((config/'signals.json').read_text()))
        x,y,yaw=mission.initial_pose
        z=.031+(.002 if mission.segments[0].surface=='yard' else 0.)
        arguments.update(x=str(x),y=str(y),z=str(z),yaw=str(yaw))
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(share/'launch/simulation.launch.py')),
        launch_arguments=arguments.items())]


def generate_launch_description():
    sim=OpaqueFunction(function=_simulation_launch)
    params={'use_sim_time':True}
    driver=Node(package='dongfeng_autonomy',executable='autonomy_node',output='screen',remappings=[('/camera/image_raw',LaunchConfiguration('image_topic')),('/scan',LaunchConfiguration('scan_topic'))],parameters=[params,{'speed':ParameterValue(LaunchConfiguration('speed'),value_type=float),'driving_mode':LaunchConfiguration('driving_mode'),'mission':LaunchConfiguration('mission'),'map_output':LaunchConfiguration('map_output')}])
    lights=Node(package='dongfeng_autonomy',executable='signal_node',output='screen',parameters=[params,{'phase_offset':ParameterValue(LaunchConfiguration('phase_offset'),value_type=float),'force_color':LaunchConfiguration('force_color')}])
    return LaunchDescription([
        DeclareLaunchArgument('gui_config',default_value=''),
        DeclareLaunchArgument('headless',default_value='false'),
        DeclareLaunchArgument('headless_rendering',default_value='false'),
        DeclareLaunchArgument('render_engine',default_value='ogre'),
        DeclareLaunchArgument('sensor_software_rendering',default_value='true'),
        DeclareLaunchArgument('sensor_render_engine',default_value='ogre2'),
        DeclareLaunchArgument('image_topic',default_value='/camera/image_raw'),
        DeclareLaunchArgument('scan_topic',default_value='/scan'),
        DeclareLaunchArgument('driving_mode',default_value='fast',choices=['fast']),
        DeclareLaunchArgument('speed',default_value='0.0',description='0 selects the optimized 0.40 m/s maximum; a lower positive cap is optional.'),
        DeclareLaunchArgument('mission',default_value='full_demo'),
        DeclareLaunchArgument('map_output',default_value='reports/autonomy/parking_map'),
        DeclareLaunchArgument('phase_offset',default_value='0.0'),
        DeclareLaunchArgument('force_color',default_value='cycle'),
        *[RegisterEventHandler(OnProcessExit(target_action=n,on_exit=[EmitEvent(event=Shutdown(reason='Autonomy node exited'))])) for n in (driver,lights)],
        sim,lights,driver])
