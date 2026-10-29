"""Painting cell.  mode:=plan  -> move_group + RViz (+ static joint states) for planning
                   mode:=play  -> move_group + RViz + player that replays a recorded plan."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory
import os


def launch_setup(context, *args, **kwargs):
    mode = LaunchConfiguration('mode').perform(context)
    plan = LaunchConfiguration('plan').perform(context)
    speed = LaunchConfiguration('speed').perform(context)
    use_rviz = LaunchConfiguration('rviz').perform(context) == 'true'
    rviz_name = LaunchConfiguration('rviz_config').perform(context)
    maps = {'rail_y': LaunchConfiguration('rail_y').perform(context),
            'pedestal_h': LaunchConfiguration('pedestal_h').perform(context)}
    mc = (MoveItConfigsBuilder('painting_cell', package_name='painting_cell')
          .robot_description(file_path='urdf/painting_cell.urdf.xacro', mappings=maps)
          .robot_description_semantic(file_path='config/painting_cell.srdf')
          .robot_description_kinematics(file_path='config/kinematics.yaml')
          .joint_limits(file_path='config/joint_limits.yaml')
          .trajectory_execution(file_path='config/moveit_controllers.yaml')
          .planning_pipelines(pipelines=['ompl'])
          .to_moveit_configs())
    nodes = [
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[mc.robot_description, {'use_sim_time': False}], output='log'),
        Node(package='moveit_ros_move_group', executable='move_group', output='screen',
             parameters=[mc.to_dict(), {'publish_robot_description_semantic': True,
                                        'publish_planning_scene': True,
                                        'publish_geometry_updates': True,
                                        'publish_state_updates': True,
                                        'publish_transforms_updates': True}]),
    ]
    if mode == 'plan':
        nodes.append(Node(package='joint_state_publisher', executable='joint_state_publisher',
                          output='log'))
    if use_rviz:
        rviz_cfg = os.path.join(get_package_share_directory('painting_cell'), 'rviz', rviz_name)
        nodes.append(Node(package='rviz2', executable='rviz2', arguments=['-d', rviz_cfg], output='log',
                          parameters=[mc.robot_description, mc.robot_description_semantic,
                                      mc.robot_description_kinematics, mc.planning_pipelines,
                                      mc.joint_limits]))
    if mode == 'play':
        nodes.append(Node(package='painting_cell', executable='play_paint', output='screen',
                          parameters=[{'plan': plan, 'speed': float(speed)}]))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='plan'),
        DeclareLaunchArgument('plan', default_value='/tmp/plan.json'),
        DeclareLaunchArgument('speed', default_value='1.0'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('rviz_config', default_value='painting_cell.rviz'),
        DeclareLaunchArgument('rail_y', default_value='-2.2'),
        DeclareLaunchArgument('pedestal_h', default_value='0.58'),
        OpaqueFunction(function=launch_setup),
    ])
