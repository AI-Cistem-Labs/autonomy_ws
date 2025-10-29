#!/usr/bin/env python3
"""
Unitree Go2 Web Bridge
Publica datos del robot a una interfaz web mediante REST API y WebSocket
Tópico principal: /lf/lowstate
"""

import rclpy
from rclpy.node import Node
from flask import Flask, jsonify
from flask_cors import CORS
from flask_socketio import SocketIO, emit
import threading
from datetime import datetime
from geometry_msgs.msg import PointStamped
import os
from ament_index_python.packages import get_package_share_directory

# ============================================================
# Configuración inicial
# ============================================================
node = None
WEB_DIR = os.path.join(get_package_share_directory('go2_web_bridge'), 'web')

# Estado compartido
robot_state = {
    'battery': {'soc': 0, 'voltage': 0.0, 'current': 0.0},
    'imu': {'accelerometer': [0, 0, 0]},
    'motors': [],
    'foot_force': [0, 0, 0, 0],
    'timestamp': datetime.now().isoformat(),
    'connected': False
}
state_lock = threading.Lock()

# ============================================================
# Serialización simple
# ============================================================
def convert_to_serializable(obj):
    if isinstance(obj, dict):
        return {k: convert_to_serializable(v) for k, v in obj.items()}
    elif isinstance(obj, list):
        return [convert_to_serializable(i) for i in obj]
    elif hasattr(obj, 'tolist'):
        return obj.tolist()
    elif isinstance(obj, (int, float)):
        return obj
    else:
        return obj

# ============================================================
# Importar mensaje LowState
# ============================================================
try:
    from unitree_go.msg import LowState
except ImportError:
    print("⚠️  No se encontró unitree_go.msg, usando estructura simulada")
    LowState = None

# ============================================================
# Nodo principal
# ============================================================
class Go2Bridge(Node):
    def __init__(self):
        super().__init__('go2_web_bridge')

        # Suscripción a LowState
        self.subscription = self.create_subscription(
            LowState if LowState else type('LowState', (), {}),
            '/lf/lowstate',
            self.lowstate_callback,
            10
        )

        # Publisher de waypoints
        self.waypoint_pub = self.create_publisher(PointStamped, '/way_point', 10)

        # Timer de conexión
        self.last_msg_time = self.get_clock().now()
        self.create_timer(2.0, self.check_connection)

        self.get_logger().info('🚀 Go2 Web Bridge iniciado')
        self.get_logger().info('📡 Suscrito a /lf/lowstate')

    # ============================================================
    # Verificación de conexión
    # ============================================================
    def check_connection(self):
        time_diff = (self.get_clock().now() - self.last_msg_time).nanoseconds / 1e9
        with state_lock:
            robot_state['connected'] = time_diff < 3.0
        if time_diff > 5.0:
            self.get_logger().warn('⚠️  No se han recibido mensajes en 5 segundos')

    # ============================================================
    # Callback LowState
    # ============================================================
    def lowstate_callback(self, msg):
        self.last_msg_time = self.get_clock().now()
        with state_lock:
            # Batería
            if hasattr(msg, 'bms_state'):
                bms = msg.bms_state
                robot_state['battery']['soc'] = getattr(bms, 'soc', 0)
                robot_state['battery']['voltage'] = sum(getattr(bms, 'cell_vol', [0]*8)[:8])/1000.0
                robot_state['battery']['current'] = getattr(bms, 'current', 0)/1000.0

            # IMU
            if hasattr(msg, 'imu_state'):
                imu = msg.imu_state
                robot_state['imu']['accelerometer'] = list(getattr(imu, 'accelerometer', [0,0,0]))

            # Motores
            if hasattr(msg, 'motor_state'):
                robot_state['motors'] = [{'name': f'Motor {i+1}', 
                                          'temperature': getattr(m,'temperature',0)}
                                         for i, m in enumerate(msg.motor_state)]

            # Fuerza patas
            if hasattr(msg, 'foot_force'):
                robot_state['foot_force'] = list(msg.foot_force)

            robot_state['timestamp'] = datetime.now().isoformat()
            robot_state['connected'] = True

        # Emitir a WebSocket
        socketio.emit('robot_update', convert_to_serializable(robot_state))

# ============================================================
# Flask + SocketIO
# ============================================================
app = Flask(__name__)
CORS(app)
socketio = SocketIO(app, cors_allowed_origins="*", async_mode='threading')

@app.route('/api/status')
def get_status():
    with state_lock:
        return jsonify(robot_state)

@app.route('/')
def serve_index():
    return open(os.path.join(WEB_DIR, 'index.html')).read()

# ============================================================
# WebSocket eventos
# ============================================================
@socketio.on('connect')
def ws_connect():
    print("✅ Cliente web conectado")
    emit('robot_update', convert_to_serializable(robot_state))

@socketio.on('disconnect')
def ws_disconnect():
    print("❌ Cliente web desconectado")

@socketio.on('home_command')
def ws_home_command():
    print("[WEB] 🏠 Comando HOME recibido")
    msg = PointStamped()
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.header.frame_id = 'map'
    msg.point.x = 0.0
    msg.point.y = 0.0
    msg.point.z = 0.0
    node.waypoint_pub.publish(msg)
    print("✅ Waypoint HOME publicado en /way_point")

# ============================================================
# Ejecutar Flask en hilo paralelo
# ============================================================
def run_flask():
    socketio.run(app, host='0.0.0.0', port=5000, allow_unsafe_werkzeug=True)

# ============================================================
# Main
# ============================================================
def main(args=None):
    global node
    rclpy.init(args=args)
    node = Go2Bridge()

    flask_thread = threading.Thread(target=run_flask, daemon=True)
    flask_thread.start()

    print("="*60)
    print("🤖 Unitree Go2 Web Bridge corriendo")
    print("🌐 http://<IP_DEL_ROBOT>:5000/")
    print("="*60)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n👋 Cerrando Go2 Web Bridge...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
