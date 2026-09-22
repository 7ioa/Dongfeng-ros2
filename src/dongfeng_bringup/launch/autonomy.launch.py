"""Full sensor-based lap; GUI and sensor rendering are independently selectable."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share=Path(get_package_share_directory('dongfeng_bringup'))
    sim=IncludeLaunchDescription(PythonLaunchDescriptionSource(str(share/'launch/simulation.launch.py')),
        launch_arguments={**{k:LaunchConfiguration(k) for k in ('gui_config','headless','headless_rendering','render_engine','sensor_render_engine','sensor_software_rendering')},'auto_mode':'true'}.items())
    params={'use_sim_time':True}
    driver=Node(package='dongfeng_autonomy',executable='autonomy_node',output='screen',remappings=[('/camera/image_raw',LaunchConfiguration('image_topic')),('/scan',LaunchConfiguration('scan_topic'))],parameters=[params,{'speed':ParameterValue(LaunchConfiguration('speed'),value_type=float)}])
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
        DeclareLaunchArgument('speed',default_value='0.15'),
        DeclareLaunchArgument('phase_offset',default_value='0.0'),
        DeclareLaunchArgument('force_color',default_value='cycle'),
        *[RegisterEventHandler(OnProcessExit(target_action=n,on_exit=[EmitEvent(event=Shutdown(reason='Autonomy node exited'))])) for n in (driver,lights)],
        sim,lights,driver])
