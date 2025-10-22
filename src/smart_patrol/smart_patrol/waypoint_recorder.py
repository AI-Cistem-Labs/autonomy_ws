#!/usr/bin/env python3
"""
Grabador de Waypoints Interactivo
Teleopera el robot y graba posiciones para crear rutas de patrullaje
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import yaml
import os
import sys
from datetime import datetime

class WaypointRecorder(Node):
    """Nodo para grabar waypoints mientras teleoperas"""
    
    def __init__(self):
        super().__init__('waypoint_recorder')
        
        # Parámetros
        self.declare_parameter('output_file', 'recorded_route.yaml')
        self.declare_parameter('odom_topic', '/state_estimation')
        
        # Estado
        self.current_pose = None
        self.waypoints = []
        self.recording = True
        
        # Subscriber
        odom_topic = self.get_parameter('odom_topic').value
        self.odom_sub = self.create_subscription(
            Odometry,
            odom_topic,
            self.odom_callback,
            10)
        
        # Esperar a recibir primera posición
        self.get_logger().info('⏳ Esperando datos de odometría...')
        while self.current_pose is None and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
        
        # Iniciar interfaz interactiva
        self.print_banner()
        self.interactive_loop()
    
    def print_banner(self):
        """Banner de bienvenida"""
        print('\n' + '='*70)
        print('   📍 GRABADOR DE WAYPOINTS INTERACTIVO')
        print('='*70)
        print('   Teleopera el robot y graba waypoints para patrullaje')
        print('='*70)
        print('\n🎮 CONTROLES:')
        print('   • Teleopera el robot con el joystick/teclado')
        print('   • Presiona ENTER para grabar posición actual')
        print('   • Escribe nombre del waypoint (o deja en blanco para auto-nombre)')
        print('   • Escribe "done" para terminar y guardar')
        print('   • Escribe "list" para ver waypoints grabados')
        print('   • Escribe "undo" para borrar último waypoint')
        print('   • Escribe "quit" para salir sin guardar')
        print('='*70 + '\n')
    
    def odom_callback(self, msg):
        """Callback de odometría"""
        self.current_pose = msg.pose.pose
    
    def get_current_position(self):
        """Obtener posición actual"""
        if self.current_pose is None:
            return None
        return {
            'x': round(self.current_pose.position.x, 3),
            'y': round(self.current_pose.position.y, 3),
            'z': round(self.current_pose.position.z, 3)
        }
    
    def add_waypoint(self, name: str = None):
        """Agregar waypoint en posición actual"""
        pos = self.get_current_position()
        if pos is None:
            print('❌ Error: No hay datos de odometría')
            return False
        
        # Auto-generar nombre si no se proporciona
        if not name or name.strip() == '':
            name = f'waypoint_{len(self.waypoints) + 1}'
        
        # Verificar que el nombre no exista
        if any(wp['name'] == name for wp in self.waypoints):
            print(f'⚠️  Nombre "{name}" ya existe, usa otro nombre')
            return False
        
        waypoint = {
            'name': name,
            'x': pos['x'],
            'y': pos['y'],
            'z': pos['z']
        }
        
        self.waypoints.append(waypoint)
        print(f'✅ Waypoint grabado: {name} → ({pos["x"]:.2f}, {pos["y"]:.2f}, {pos["z"]:.2f})')
        return True
    
    def list_waypoints(self):
        """Listar waypoints grabados"""
        if not self.waypoints:
            print('📋 No hay waypoints grabados aún')
            return
        
        print(f'\n📋 WAYPOINTS GRABADOS ({len(self.waypoints)}):')
        print('='*70)
        for i, wp in enumerate(self.waypoints, 1):
            print(f'   {i}. {wp["name"]:20s} → ({wp["x"]:7.2f}, {wp["y"]:7.2f}, {wp["z"]:7.2f})')
        print('='*70 + '\n')
    
    def undo_last(self):
        """Borrar último waypoint"""
        if not self.waypoints:
            print('⚠️  No hay waypoints para borrar')
            return
        
        removed = self.waypoints.pop()
        print(f'🗑️  Waypoint borrado: {removed["name"]}')
    
    def save_to_file(self):
        """Guardar waypoints a archivo YAML"""
        if not self.waypoints:
            print('⚠️  No hay waypoints para guardar')
            return False
        
        output_file = self.get_parameter('output_file').value
        
        # Crear directorio si no existe
        output_dir = os.path.join(
            os.path.expanduser('~'),
            'autonomy_ws/src/smart_patrol/waypoints'
        )
        os.makedirs(output_dir, exist_ok=True)
        
        output_path = os.path.join(output_dir, output_file)
        
        # Convertir a formato YAML
        yaml_data = {'waypoints': {}}
        for wp in self.waypoints:
            yaml_data['waypoints'][wp['name']] = {
                'x': float(wp['x']),
                'y': float(wp['y']),
                'z': float(wp['z'])
            }
        
        # Guardar archivo
        try:
            with open(output_path, 'w') as f:
                yaml.dump(yaml_data, f, default_flow_style=False, sort_keys=False)
            
            print('\n' + '='*70)
            print(f'💾 Waypoints guardados exitosamente!')
            print(f'📁 Archivo: {output_path}')
            print(f'📊 Total: {len(self.waypoints)} waypoints')
            print('='*70)
            
            # Mostrar cómo usar el archivo
            print('\n📖 CÓMO USAR ESTOS WAYPOINTS:')
            print(f'   ros2 launch smart_patrol full_patrol_system.launch.py \\')
            print(f'     waypoints_file:={output_file}')
            print('='*70 + '\n')
            
            return True
            
        except Exception as e:
            print(f'❌ Error guardando archivo: {e}')
            return False
    
    def interactive_loop(self):
        """Loop interactivo principal con actualización continua de odometría"""
        import threading
        import queue
    
        # Cola para comunicación entre threads
        command_queue = queue.Queue()
    
        def input_thread():
            """Thread separado para leer input sin bloquear ROS"""
            try:
                while rclpy.ok():
                    try:
                        cmd = input('>>> ')
                        command_queue.put(cmd)
                    except EOFError:
                        break
            except KeyboardInterrupt:
                pass
    
        # Iniciar thread de input
        input_thread_obj = threading.Thread(target=input_thread, daemon=True)
        input_thread_obj.start()
    
        try:
            while rclpy.ok():
                # Procesar callbacks de ROS (actualiza odometría)
                rclpy.spin_once(self, timeout_sec=0.1)
            
                # Verificar si hay comandos en la cola
                try:
                    command = command_queue.get_nowait().strip().lower()
                
                    # Procesar comando
                    if command == '':
                        # Enter sin texto = grabar con auto-nombre
                        self.add_waypoint()
                
                    elif command == 'done':
                        # Terminar y guardar
                        if self.save_to_file():
                            break
                
                    elif command == 'list':
                        self.list_waypoints()
                
                    elif command == 'undo':
                        self.undo_last()
                
                    elif command == 'quit':
                        print('🚪 Saliendo sin guardar...')
                        break
                
                    else:
                        # Grabar con nombre personalizado
                        self.add_waypoint(command)
                
                    # Mostrar posición actual después de cada comando
                    pos = self.get_current_position()
                    if pos:
                        print(f'\n📍 Posición actual: ({pos["x"]:.2f}, {pos["y"]:.2f}, {pos["z"]:.2f})')
                
                except queue.Empty:
                    # No hay comandos, continuar
                    pass
    
        except KeyboardInterrupt:
            print('\n\n⚠️  Grabación interrumpida')
        
            # Preguntar si quiere guardar
            try:
                save = input('¿Guardar waypoints antes de salir? (s/n): ').strip().lower()
                if save == 's' or save == 'y':
                    self.save_to_file()
            except:
                pass
                
def main(args=None):
    rclpy.init(args=args)
    
    try:
        recorder = WaypointRecorder()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
