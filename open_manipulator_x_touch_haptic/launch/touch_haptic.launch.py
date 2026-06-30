#!/usr/bin/env python3
"""
touch_haptic.launch.py — Teleoperación Geomagic Touch -> OpenMANIPULATOR-X.

  - touch_haptic_node  : dueño del puerto del OM-X; sigue al Touch (modos
                         articular / cartesiano). En --sim hace eco (sin robot).
  - touch_haptic_panel : panel PyQt5 (habilitar / E-STOP / modo / gripper).
  - (opcional) driver Geomagic real  : launch_driver:=true  -> omni_state.
  - (opcional) Touch VIRTUAL          : virtual_touch:=true  -> /phantom/* falso.

Argumentos:
  robot_port:=/dev/ttyUSB0     puerto U2D2 del OM-X
  robot_id:=1                  ID del robot (calibración del gripper)
  sim:=true|false              (def. false) sin robot; eco del comando
  mode:=joint|cartesian        (def. joint) modo inicial
  enable_on_start:=true|false  (def. false) arrancar con el espejo activo
  launch_driver:=true|false    (def. false) lanzar también el driver Geomagic real
  virtual_touch:=true|false    (def. false) lanzar un Touch virtual de prueba

Notas:
  - El driver Geomagic se lanza normalmente aparte:
        ros2 launch omni_common omni_state.launch.py
    Usa launch_driver:=true para incluirlo aquí.
  - NO uses launch_driver y virtual_touch a la vez (ambos publican /phantom/*).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_port = LaunchConfiguration('robot_port')
    robot_id = LaunchConfiguration('robot_id')
    sim = LaunchConfiguration('sim')
    mode = LaunchConfiguration('mode')
    enable_on_start = LaunchConfiguration('enable_on_start')
    launch_driver = LaunchConfiguration('launch_driver')
    virtual_touch = LaunchConfiguration('virtual_touch')

    return LaunchDescription([
        DeclareLaunchArgument('robot_port', default_value='/dev/ttyUSB0',
                              description='Puerto U2D2 del OpenMANIPULATOR-X.'),
        DeclareLaunchArgument('robot_id', default_value='1',
                              description='ID del robot (define la calibración del gripper).'),
        DeclareLaunchArgument('sim', default_value='false',
                              description='Sin robot: eco del comando.'),
        DeclareLaunchArgument('mode', default_value='joint',
                              description='Modo inicial: joint | cartesian.'),
        DeclareLaunchArgument('enable_on_start', default_value='false',
                              description='Arrancar con el espejo ya activo.'),
        DeclareLaunchArgument('launch_driver', default_value='false',
                              description='Lanzar también el driver Geomagic real.'),
        DeclareLaunchArgument('virtual_touch', default_value='false',
                              description='Lanzar un Touch virtual (pruebas sin hardware).'),

        # Driver Geomagic real (opcional).
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('omni_common'), 'launch', 'omni_state.launch.py'])),
            condition=IfCondition(launch_driver),
        ),

        # Touch virtual (opcional, para pruebas).
        Node(
            package='open_manipulator_x_touch_haptic',
            executable='virtual_touch',
            name='virtual_touch',
            output='screen',
            condition=IfCondition(virtual_touch),
        ),

        Node(
            package='open_manipulator_x_touch_haptic',
            executable='touch_haptic_node',
            name='touch_haptic_node',
            output='screen',
            parameters=[{
                'robot_port': robot_port,
                'robot_id': robot_id,
                'sim': sim,
                'mode': mode,
                'enable_on_start': enable_on_start,
            }],
        ),

        Node(
            package='open_manipulator_x_touch_haptic',
            executable='touch_haptic_panel',
            name='touch_haptic_panel',
            output='screen',
        ),
    ])
