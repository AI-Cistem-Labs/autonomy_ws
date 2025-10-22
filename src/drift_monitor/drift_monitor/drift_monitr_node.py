#!/usr/bin/env python3
"""
Nodo de Monitoreo de Drift para Point-LIO
Detecta drift acumulado y publica alertas
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float32
from geometry_msgs.msg import Point
import math

class DriftMonitor(Node):
    def __init__(self):
        super().__init__('drift_monitor')
        
        # Parámetros configurables
        self.declare_parameter('drift_threshold', 0.15)      # 15% drift
        self.declare_parameter('min_distance', 2.0)          # Mínimo 2m para calcular
        self.declare_parameter('window_size', 50)            # Ventana de muestras
        self.declare_parameter('enable_alerts', True)
        
        # Estado interno
        self.positions = []
        self.last_position = None
        self.total_distance = 0.0
        self.start_position = None
        self.loop_count = 0
        
        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry,
            '/state_estimation',
            self.odom_callback,
            10)
        
        # Publishers
        self.drift_detected_pub = self.create_publisher(
            Bool,
            '/drift_detected',
            10)
        
        self.drift_value_pub = self.create_publisher(
            Float32,
            '/drift_value',
            10)
        
        self.drift_distance_pub = self.create_publisher(
            Float32,
            '/drift_distance',
            10)
        
        # Timer para logging periódico
        self.timer = self.create_timer(10.0, self.log_status)
        
        self.get_logger().info('=' * 50)
        self.get_logger().info('  🔍 Drift Monitor Iniciado')
        self.get_logger().info('=' * 50)
        self.get_logger().info(f'  Umbral de drift: {self.get_parameter("drift_threshold").value * 100:.1f}%')
        self.get_logger().info(f'  Distancia mínima: {self.get_parameter("min_distance").value}m')
        self.get_logger().info('=' * 50)
    
    def odom_callback(self, msg):
        current_pos = [
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        ]
        
        # Guardar posición inicial
        if self.start_position is None:
            self.start_position = current_pos
            self.get_logger().info(f'📍 Posición inicial: ({current_pos[0]:.2f}, {current_pos[1]:.2f}, {current_pos[2]:.2f})')
        
        # Calcular distancia incremental
        if self.last_position is not None:
            dx = current_pos[0] - self.last_position[0]
            dy = current_pos[1] - self.last_position[1]
            dz = current_pos[2] - self.last_position[2]
            dist = math.sqrt(dx*dx + dy*dy + dz*dz)
            
            self.total_distance += dist
            
            # Guardar en ventana deslizante
            self.positions.append(current_pos)
            window_size = self.get_parameter('window_size').value
            if len(self.positions) > window_size:
                self.positions.pop(0)
            
            # Calcular drift solo después de mínima distancia
            min_dist = self.get_parameter('min_distance').value
            if self.total_distance > min_dist:
                self.calculate_drift(current_pos)
        
        self.last_position = current_pos
    
    def calculate_drift(self, current_pos):
        """Calcula drift basado en desviación de trayectoria ideal"""
        # Distancia al origen
        dx_origin = current_pos[0] - self.start_position[0]
        dy_origin = current_pos[1] - self.start_position[1]
        dz_origin = current_pos[2] - self.start_position[2]
        distance_from_origin = math.sqrt(dx_origin*dx_origin + dy_origin*dy_origin + dz_origin*dz_origin)
        
        # Drift ratio = distancia recorrida vs distancia lineal
        if self.total_distance > 0.1:
            # En un loop perfecto, deberías volver al origen
            drift_distance = distance_from_origin
            drift_ratio = drift_distance / self.total_distance
            
            # Publicar métricas
            drift_msg = Float32()
            drift_msg.data = drift_ratio
            self.drift_value_pub.publish(drift_msg)
            
            drift_dist_msg = Float32()
            drift_dist_msg.data = drift_distance
            self.drift_distance_pub.publish(drift_dist_msg)
            
            # Detectar drift excesivo
            threshold = self.get_parameter('drift_threshold').value
            if drift_ratio > threshold and self.get_parameter('enable_alerts').value:
                alert_msg = Bool()
                alert_msg.data = True
                self.drift_detected_pub.publish(alert_msg)
                
                self.get_logger().warn('=' * 50)
                self.get_logger().warn(f'⚠️  DRIFT DETECTADO: {drift_ratio*100:.1f}%')
                self.get_logger().warn(f'   Distancia recorrida: {self.total_distance:.2f}m')
                self.get_logger().warn(f'   Desviación: {drift_distance:.2f}m')
                self.get_logger().warn('=' * 50)
    
    def log_status(self):
        """Log periódico del estado"""
        if self.total_distance > 0.1:
            if self.start_position and self.last_position:
                dx = self.last_position[0] - self.start_position[0]
                dy = self.last_position[1] - self.start_position[1]
                dist_from_origin = math.sqrt(dx*dx + dy*dy)
                
                self.get_logger().info(f'📊 Distancia recorrida: {self.total_distance:.2f}m | '
                                      f'Desviación origen: {dist_from_origin:.2f}m')

def main(args=None):
    rclpy.init(args=args)
    node = DriftMonitor()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Drift Monitor detenido')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
