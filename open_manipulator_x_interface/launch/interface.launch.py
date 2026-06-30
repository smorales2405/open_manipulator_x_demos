#!/usr/bin/env python3
"""
interface.launch.py — Lanza la interfaz de taller completa:

  - robot_bridge          : puente Dynamixel (dueño del puerto) o eco en --sim.
  - robot_state_publisher : robot "preview" alimentado por /joint_states_preview.
  - rviz2                 : muestra el modelo (preview / espejo del robot real).
  - interface_gui         : la ventana PyQt5.

Argumentos:
  sim:=true|false        (def. false) — sin robot, eco de comandos. Útil para
                          preparar el taller sin hardware.
  port_name:=/dev/ttyUSB0
  rviz:=true|false       (def. true)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import (Command, FindExecutable, LaunchConfiguration,
                                   PathJoinSubstitution)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    sim = LaunchConfiguration('sim')
    port_name = LaunchConfiguration('port_name')
    use_rviz = LaunchConfiguration('rviz')
    robot_id = LaunchConfiguration('robot_id')

    robot_description = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]), ' ',
        PathJoinSubstitution([
            FindPackageShare('open_manipulator_x_description'),
            'urdf', 'open_manipulator_x_robot.urdf.xacro']),
        ' ', 'prefix:=', '""', ' ', 'use_fake_hardware:=', 'False',
    ])

    rviz_config = PathJoinSubstitution([
        FindPackageShare('open_manipulator_x_interface'),
        'rviz', 'interface_preview.rviz'])

    return LaunchDescription([
        DeclareLaunchArgument('sim', default_value='false',
                              description='Sin robot: eco de comandos.'),
        DeclareLaunchArgument('port_name', default_value='/dev/ttyUSB0',
                              description='Puerto del U2D2.'),
        DeclareLaunchArgument('rviz', default_value='true',
                              description='Lanzar RViz para el modelo preview.'),
        DeclareLaunchArgument('robot_id', default_value='1',
                              description='ID del robot (1-8). '
                                          'Define la calibración del gripper.'),

        Node(
            package='open_manipulator_x_interface',
            executable='robot_bridge',
            name='robot_bridge',
            output='screen',
            parameters=[{'sim': sim, 'port_name': port_name, 'robot_id': robot_id}],
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='preview_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
            remappings=[('joint_states', '/joint_states_preview')],
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen',
            condition=IfCondition(use_rviz),
        ),

        Node(
            package='open_manipulator_x_interface',
            executable='interface_gui',
            name='interface_gui',
            output='screen',
            parameters=[{'robot_id': robot_id}],
        ),
    ])
