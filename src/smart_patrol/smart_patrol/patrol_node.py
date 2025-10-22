#!/usr/bin/env python3
"""
Smart Patrol Node
Sistema avanzado de patrullaje para Unitree GO2 con Point-LIO
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Bool, Float32
from std_srvs.srv import Trigger, SetBool
import math
import time
import os
from .waypoint_manager import WaypointManager, Waypoint

class SmartPatrolNode(Node):
    """Nodo principal de patrullaje inteligente"""
    
    def __init__(self):
        super().__init__('smart_patrol_node')
        
        # ==================== PARÁMETROS ====================
        self.declare_parameter('goal_tolerance', 0.3)
        self.declare_parameter('wait_time_at_waypoint', 10.0)
        self.declare_parameter('patrol_rate', 5.0)
        self.declare_parameter('loop_forever', True)
        self.declare_parameter('max_distance_from_path', 1.0)
        self.declare_parameter('drift_threshold', 0.20)
        self.declare_parameter('enable_auto_pause_on_drift', True)
        self.declare_parameter('odom_topic', '/state_estimation')
        self.declare_parameter('waypoint_topic', '/way_point')
        self.declare_parameter('waypoints_file', '')
        self.declare_parameter('verbose_logging', True)
        self.declare_parameter('ping_pong_mode', True)
        
        # ==================== ESTADO ====================
        self.current_pose = None
        self.at_waypoint = False
        self.arrival_time = None
        self.is_paused = False
        self.drift_detected = False
        self.current_drift_value = 0.0
        self.just_left_waypoint = False
        
        # ==================== WAYPOINT MANAGER ====================
        self.waypoint_manager = WaypointManager(self.get_logger())
        
        # Cargar waypoints
        waypoints_file = self.get_parameter('waypoints_file').value
        if not waypoints_file:
            # Usar ruta por defecto
            pkg_share = os.path.join(
                os.path.expanduser('~'),
                'autonomy_ws/src/smart_patrol/waypoints/default_route.yaml'
            )
            waypoints_file = pkg_share
        
        if not self.waypoint_manager.load_from_yaml(waypoints_file):
            self.get_logger().error('❌ No se pudieron cargar waypoints - Abortando')
            raise RuntimeError('Failed to load waypoints')
        
        ping_pong = self.get_parameter('ping_pong_mode').value
        self.waypoint_manager.set_ping_pong_mode(ping_pong) 
        
        # ==================== PUBLISHERS ====================
        waypoint_topic = self.get_parameter('waypoint_topic').value
        self.waypoint_pub = self.create_publisher(
            PointStamped,
            waypoint_topic,
            10)
        
        # ==================== SUBSCRIBERS ====================
        odom_topic = self.get_parameter('odom_topic').value
        self.odom_sub = self.create_subscription(
            Odometry,
            odom_topic,
            self.odom_callback,
            10)
        
        # Subscriber para drift (si existe drift_monitor)
        self.drift_sub = self.create_subscription(
            Bool,
            '/drift_detected',
            self.drift_callback,
            10)
        
        self.drift_value_sub = self.create_subscription(
            Float32,
            '/drift_value',
            self.drift_value_callback,
            10)
        
        # ==================== SERVICIOS ====================
        self.pause_srv = self.create_service(
            Trigger,
            '/patrol/pause',
            self.pause_service)
        
        self.resume_srv = self.create_service(
            Trigger,
            '/patrol/resume',
            self.resume_service)
        
        self.goto_srv = self.create_service(
            SetBool,  # Usaremos el data field para el nombre
            '/patrol/goto_waypoint',
            self.goto_waypoint_service)
        
        self.status_srv = self.create_service(
            Trigger,
            '/patrol/status',
            self.status_service)
        
        # ==================== TIMER ====================
        patrol_rate = self.get_parameter('patrol_rate').value
        self.timer = self.create_timer(1.0 / patrol_rate, self.patrol_loop)
        
        # ==================== STARTUP ====================
        self.print_startup_banner()
        
        # Enviar primer waypoint
        time.sleep(1.0)  # Esperar a que todo esté listo
        self.send_current_waypoint()
    
    def print_startup_banner(self):
        """Imprimir banner de inicio"""
        self.get_logger().info('=' * 60)
        self.get_logger().info('   🤖 SMART PATROL SYSTEM INICIADO 🤖')
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'   Waypoints: {len(self.waypoint_manager.waypoints)}')
        self.get_logger().info(f'   Tolerancia: {self.get_parameter("goal_tolerance").value}m')
        self.get_logger().info(f'   Espera: {self.get_parameter("wait_time_at_waypoint").value}s')
        self.get_logger().info(f'   Loop infinito: {self.get_parameter("loop_forever").value}')
        
        mode = "IDA Y VUELTA 🔄" if self.get_parameter('ping_pong_mode').value else "LOOP DIRECTO ➰"
        self.get_logger().info(f'   Modo: {mode}')
    
        self.get_logger().info('=' * 60)
        self.get_logger().info('   RUTA DE PATRULLAJE:')
        for i, wp in enumerate(self.waypoint_manager.waypoints):
            self.get_logger().info(f'   {i+1}. {wp}')
        self.get_logger().info('=' * 60)
        self.get_logger().info('   Comandos disponibles:')
        self.get_logger().info('   • /patrol/pause     - Pausar patrullaje')
        self.get_logger().info('   • /patrol/resume    - Reanudar patrullaje')
        self.get_logger().info('   • /patrol/status    - Ver estado')
        self.get_logger().info('=' * 60)
    
    def odom_callback(self, msg):
        """Callback de odometría"""
        self.current_pose = msg.pose.pose
    
    def drift_callback(self, msg):
        """Callback de alerta de drift"""
        if msg.data and self.get_parameter('enable_auto_pause_on_drift').value:
            if not self.drift_detected:  # Solo alertar una vez
                self.get_logger().warn('⚠️  DRIFT DETECTADO - Pausando automáticamente')
                self.drift_detected = True
                self.pause()
    
    def drift_value_callback(self, msg):
        """Callback del valor de drift"""
        self.current_drift_value = msg.data
    
    def distance_to_waypoint(self, waypoint: Waypoint) -> float:
        """Calcular distancia al waypoint"""
        if self.current_pose is None:
            return float('inf')
        
        dx = self.current_pose.position.x - waypoint.x
        dy = self.current_pose.position.y - waypoint.y
        return math.sqrt(dx*dx + dy*dy)
    
    def send_current_waypoint(self):
        """Enviar waypoint actual al sistema de navegación"""
        waypoint = self.waypoint_manager.get_current_waypoint()
        if waypoint is None:
            return
        
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.point = waypoint.to_point()
        
        self.waypoint_pub.publish(msg)
        
        if self.get_parameter('verbose_logging').value:
            self.get_logger().info(f'🎯 Navegando a: {waypoint}')
    
    def patrol_loop(self):
        """Loop principal de patrullaje"""
        # Verificar que haya waypoints
        if len(self.waypoint_manager.waypoints) == 0:
            return
    
        # Si está pausado, no hacer nada
        if self.is_paused:
            return
    
        # Obtener waypoint actual
        waypoint = self.waypoint_manager.get_current_waypoint()
        if waypoint is None:
            return
    
        # Calcular distancia
        distance = self.distance_to_waypoint(waypoint)
        goal_tolerance = self.get_parameter('goal_tolerance').value
    
        # Verificar si llegamos
        if distance < goal_tolerance:
            # Si acabamos de salir de un waypoint, ignorar hasta que salgamos del radio
            if self.just_left_waypoint:
                return
        
            if not self.at_waypoint:
                # Acabamos de llegar
                self.at_waypoint = True
                self.arrival_time = time.time()
                self.get_logger().info('=' * 60)
                self.get_logger().info(f'✅ LLEGADA: {waypoint}')
                self.get_logger().info(f'   Distancia: {distance:.2f}m')
                self.get_logger().info(f'   Esperando {self.get_parameter("wait_time_at_waypoint").value}s...')
                self.get_logger().info('=' * 60)
        
            # Verificar tiempo de espera
            wait_time = self.get_parameter('wait_time_at_waypoint').value
            if time.time() - self.arrival_time >= wait_time:
                # Avanzar al siguiente waypoint
                next_wp = self.waypoint_manager.get_next_waypoint()
                self.at_waypoint = False
                self.just_left_waypoint = True  # ← NUEVO: Marcar que acabamos de salir
            
                # Verificar si completamos todos los loops
                if not self.get_parameter('loop_forever').value and \
                   self.waypoint_manager.total_loops > 0:
                    self.get_logger().info('🏁 Patrullaje completado - Pausando')
                    self.pause()
                    return
            
                # Enviar siguiente waypoint
                if next_wp:
                    self.send_current_waypoint()
    
        else:
            # Aún no llegamos (fuera del radio)
            if self.at_waypoint:
                self.at_waypoint = False
        
            # Si salimos del radio, permitir detectar siguiente llegada
            if self.just_left_waypoint:
                # Verificar que estamos suficientemente lejos
                if distance > goal_tolerance * 1.5:  # 1.5x para evitar rebotes
                    self.just_left_waypoint = False
                    self.get_logger().info(f'📍 Salió del radio - Distancia: {distance:.2f}m')
    
    def pause(self):
        """Pausar patrullaje"""
        self.is_paused = True
        self.get_logger().warn('⏸️  PATRULLAJE PAUSADO')
    
    def resume(self):
        """Reanudar patrullaje"""
        self.is_paused = False
        self.drift_detected = False
        self.get_logger().info('▶️  PATRULLAJE REANUDADO')
        self.send_current_waypoint()
    
    # ==================== SERVICIOS ====================
    
    def pause_service(self, request, response):
        """Servicio para pausar"""
        self.pause()
        response.success = True
        response.message = 'Patrullaje pausado'
        return response
    
    def resume_service(self, request, response):
        """Servicio para reanudar"""
        self.resume()
        response.success = True
        response.message = 'Patrullaje reanudado'
        return response
    
    def goto_waypoint_service(self, request, response):
        """Servicio para ir a waypoint específico"""
        # Nota: SetBool no es ideal, pero funciona
        # En producción usarías un servicio custom
        waypoint_name = "waypoint_1"  # Placeholder
        
        if self.waypoint_manager.set_current_by_name(waypoint_name):
            self.at_waypoint = False
            self.send_current_waypoint()
            response.success = True
            response.message = f'Navegando a {waypoint_name}'
        else:
            response.success = False
            response.message = f'Waypoint {waypoint_name} no encontrado'
        
        return response
    
    def status_service(self, request, response):
        """Servicio para obtener estado"""
        current_wp = self.waypoint_manager.get_current_waypoint()
        status = f"Patrullaje: {'PAUSADO' if self.is_paused else 'ACTIVO'}\n"
        status += f"Waypoint actual: {current_wp}\n"
        status += f"Loops completados: {self.waypoint_manager.total_loops}\n"
        status += f"Drift actual: {self.current_drift_value*100:.1f}%"
        
        response.success = True
        response.message = status
        return response

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = SmartPatrolNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
