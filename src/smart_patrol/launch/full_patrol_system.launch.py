#!/usr/bin/env python3
"""
Launch file completo para el sistema de patrullaje
Incluye: Autonomy Stack + Smart Patrol + Drift Monitor (opcional)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    
    # ==================== ARGUMENTOS ====================
    waypoints_file_arg = DeclareLaunchArgument(
        'waypoints_file',
        default_value='default_route.yaml',
        description='Archivo YAML con waypoints'
    )
    
    enable_drift_monitor_arg = DeclareLaunchArgument(
        'enable_drift_monitor',
        default_value='false',
        description='Activar monitoreo de drift'
    )
    
    enable_rviz_arg = DeclareLaunchArgument(
        'enable_rviz',
        default_value='false',
        description='Lanzar RViz para visualización'
    )
    
    auto_start_patrol_arg = DeclareLaunchArgument(
        'auto_start_patrol',
        default_value='true',
        description='Iniciar patrullaje automáticamente'
    )
    
    goal_tolerance_arg = DeclareLaunchArgument(
        'goal_tolerance',
        default_value='0.3',
        description='Tolerancia para considerar llegada (metros)'
    )
    
    wait_time_arg = DeclareLaunchArgument(
        'wait_time',
        default_value='10.0',
        description='Tiempo de espera en cada waypoint (segundos)'
    )
    ping_pong_mode_arg = DeclareLaunchArgument(
        'ping_pong_mode',
        default_value='true',
        description='Modo ida y vuelta (true) o loop directo (false)'
    )
    
    # ==================== CONFIGURACIONES ====================
    smart_patrol_share = get_package_share_directory('smart_patrol')
    
    waypoints_file = LaunchConfiguration('waypoints_file')
    enable_drift_monitor = LaunchConfiguration('enable_drift_monitor')
    enable_rviz = LaunchConfiguration('enable_rviz')
    goal_tolerance = LaunchConfiguration('goal_tolerance')
    wait_time = LaunchConfiguration('wait_time')
    ping_pong_mode = LaunchConfiguration('ping_pong_mode')
    
    # Ruta completa al archivo de waypoints
    waypoints_path = PythonExpression([
        '"', smart_patrol_share, '/waypoints/', waypoints_file, '"'
    ])
    
    # ==================== NODOS ====================
    
    # Smart Patrol Node
    patrol_node = Node(
        package='smart_patrol',
        executable='patrol_node',
        name='smart_patrol_node',
        output='screen',
        parameters=[{
            'goal_tolerance': goal_tolerance,
            'wait_time_at_waypoint': wait_time,
            'patrol_rate': 5.0,
            'loop_forever': True,
            'max_distance_from_path': 2.0,
            'drift_threshold': 0.20,
            'enable_auto_pause_on_drift': True,
            'odom_topic': '/state_estimation',
            'waypoint_topic': '/way_point',
            'waypoints_file': waypoints_path,
            'verbose_logging': True,
            'ping_pong_mode': ping_pong_mode
        }],
        emulate_tty=True
    )
    
    # Drift Monitor (opcional)
    drift_monitor_node = Node(
        package='drift_monitor',
        executable='drift_monitor_node',
        name='drift_monitor',
        output='screen',
        parameters=[{
            'drift_threshold': 0.15,
            'min_distance': 2.0,
            'window_size': 50,
            'enable_alerts': True
        }],
        condition=IfCondition(enable_drift_monitor),
        emulate_tty=True
    )
    
    # RViz (opcional)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(enable_rviz),
        arguments=['-d', os.path.join(smart_patrol_share, 'rviz', 'patrol.rviz')]
    )
    
    return LaunchDescription([
        # Argumentos
        waypoints_file_arg,
        enable_drift_monitor_arg,
        enable_rviz_arg,
        auto_start_patrol_arg,
        goal_tolerance_arg,
        wait_time_arg,
        
        # Nodos
        patrol_node,
        drift_monitor_node,
        rviz_node,
    ])
