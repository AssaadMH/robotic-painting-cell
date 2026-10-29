"""Painting cell in Gazebo Harmonic: paint booth world + robot under ros2_control (gz_ros2_control).

  ros2 launch painting_cell gz_cell.launch.py [gui:=false]

The server runs headless (offscreen rendering for the cinema camera); once the controllers are up the gz_play
node pauses it and steps it frame by frame while the joint_trajectory_controller executes the recorded MoveIt plan."""
import os
import re
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, SetEnvironmentVariable, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


KUKA_COLORS = {'orange': '0.93 0.38 0.08 1', 'dark': '0.16 0.16 0.17 1'}


def gz_robot_description(urdf, share):
    """Adapt the MoveIt URDF for Gazebo.
    * the ROS-Industrial KUKA model has effort="0" limits (fine for MoveIt; Gazebo would apply no force)
    * the KUKA Collada colours do not survive the ogre2 import: each link is drawn from two OBJs
      (orange / dark, made by tools/make_gz_world.py) with explicit materials
    * no collision shapes: the motion was collision-checked by MoveIt, gravity is off, and DART mesh-mesh
      checks between the arm links would only cost time"""
    urdf = re.sub(r'effort="0"', 'effort="100000"', urdf)
    for link in ('base_link', 'link_1', 'link_2', 'link_3', 'link_4', 'link_5', 'link_6'):
        m = re.search(r'<link name="%s">(.*?)</link>' % link, urdf, re.S)
        if not m:
            continue
        body = m.group(1)
        origin = re.search(r'<collision>\s*(<origin[^>]*/>)', body)
        origin = origin.group(1) if origin else '<origin xyz="0 0 0" rpy="0 0 0"/>'
        vis = ''
        for k, rgba in KUKA_COLORS.items():
            if os.path.exists(os.path.join(share, 'gz', 'kuka', '%s_%s.obj' % (link, k))):
                vis += ('<visual>%s<geometry><mesh filename="package://painting_cell/gz/kuka/%s_%s.obj"/></geometry>'
                        '<material name="kuka_%s"><color rgba="%s"/></material></visual>') % (origin, link, k, k, rgba)
        body = re.sub(r'<visual>.*?</visual>', '', body, flags=re.S)
        urdf = urdf.replace(m.group(0), '<link name="%s">%s%s</link>' % (link, vis, body))
    return re.sub(r'<collision>.*?</collision>', '', urdf, flags=re.S)


def launch_setup(context, *args, **kwargs):
    share = get_package_share_directory('painting_cell')
    ctrl = os.path.join(share, 'config', 'gz_controllers.yaml')
    xacro = os.path.join(share, 'urdf', 'painting_cell.urdf.xacro')
    urdf = subprocess.run(['xacro', xacro, 'gz:=true', 'controllers:=' + ctrl], check=True,
                          capture_output=True, text=True).stdout
    urdf = gz_robot_description(urdf, share)
    world = os.path.join(share, 'gz', 'booth.sdf')
    gui = LaunchConfiguration('gui').perform(context) == 'true'
    server = ['gz', 'sim', '-r', '-v', '2', world] + ([] if gui else ['-s', '--headless-rendering'])
    return [
        ExecuteProcess(cmd=server, output='log'),
        Node(package='robot_state_publisher', executable='robot_state_publisher', output='log',
             parameters=[{'robot_description': urdf, 'use_sim_time': True}]),
        Node(package='ros_gz_bridge', executable='parameter_bridge', output='log',
             arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock']),
        TimerAction(period=4.0, actions=[
            Node(package='ros_gz_sim', executable='create', output='screen',
                 arguments=['-world', 'booth', '-name', 'painting_cell', '-string', urdf])]),
        TimerAction(period=8.0, actions=[
            Node(package='controller_manager', executable='spawner', output='screen',
                 arguments=['joint_state_broadcaster', 'painting_controller', '--controller-manager-timeout', '120',
                            '--switch-timeout', '120', '--service-call-timeout', '120'])]),
    ]


def generate_launch_description():
    share = get_package_share_directory('painting_cell')
    kuka = get_package_share_directory('kuka_kr210_support')
    res = ':'.join([os.path.dirname(share), os.path.dirname(kuka), os.environ.get('GZ_SIM_RESOURCE_PATH', '')])
    plug = ':'.join(['/opt/ros/jazzy/lib', os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', '')])
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='false'),
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', res),
        SetEnvironmentVariable('GZ_SIM_SYSTEM_PLUGIN_PATH', plug),
        OpaqueFunction(function=launch_setup),
    ])
