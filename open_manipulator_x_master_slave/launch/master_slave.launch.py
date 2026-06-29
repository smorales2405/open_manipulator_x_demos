#!/usr/bin/env python3
"""
master_slave.launch.py — Lanza la teleoperación maestro-esclavo:

  - master_slave_node  : dueño de ambos puertos (maestro libre, esclavo sigue).
  - master_slave_panel : panel PyQt5 (habilitar/E-STOP/estado).

Argumentos:
  master_port:=/dev/ttyUSB1   puerto del brazo MAESTRO
  slave_port:=/dev/ttyUSB0    puerto del brazo ESCLAVO
  sim:=true|false             (def. false) sin robots; maestro virtual + eco
  enable_on_start:=true|false (def. false) arrancar con el espejo activo
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    master_port = LaunchConfiguration('master_port')
    slave_port = LaunchConfiguration('slave_port')
    sim = LaunchConfiguration('sim')
    enable_on_start = LaunchConfiguration('enable_on_start')

    return LaunchDescription([
        DeclareLaunchArgument('master_port', default_value='/dev/ttyUSB1',
                              description='Puerto U2D2 del brazo maestro.'),
        DeclareLaunchArgument('slave_port', default_value='/dev/ttyUSB0',
                              description='Puerto U2D2 del brazo esclavo.'),
        DeclareLaunchArgument('sim', default_value='false',
                              description='Sin robots: maestro virtual + eco.'),
        DeclareLaunchArgument('enable_on_start', default_value='false',
                              description='Arrancar con el espejo ya activo.'),

        Node(
            package='open_manipulator_x_master_slave',
            executable='master_slave_node',
            name='master_slave_node',
            output='screen',
            parameters=[{
                'master_port': master_port,
                'slave_port': slave_port,
                'sim': sim,
                'enable_on_start': enable_on_start,
            }],
        ),

        Node(
            package='open_manipulator_x_master_slave',
            executable='master_slave_panel',
            name='master_slave_panel',
            output='screen',
        ),
    ])
