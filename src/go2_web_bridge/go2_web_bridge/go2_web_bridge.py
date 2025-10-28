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
import numpy as np
from geometry_msgs.msg import PointStamped
import os
from ament_index_python.packages import get_package_share_directory

# ============================================================
# Configuración inicial
# ============================================================
node = None
WEB_DIR = os.path.join(get_package_share_directory('go2_web_bridge'), 'web')


# ============================================================
# Utilidad para serializar datos NumPy
# ============================================================
def convert_to_serializable(obj):
    if isinstance(obj, dict):
        return {k: convert_to_serializable(v) for k, v in obj.items()}
    elif isinstance(obj, list):
        return [convert_to_serializable(i) for i in obj]
    elif hasattr(obj, 'tolist'):
        return obj.tolist()
    elif isinstance(obj, (np.integer, np.int32, np.int64, np.uint16, np.uint8)):
        return int(obj)
    elif isinstance(obj, (np.floating, np.float32, np.float64)):
        return float(obj)
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
# Configuración Flask + SocketIO
# ============================================================
app = Flask(__name__)
CORS(app)
socketio = SocketIO(app, cors_allowed_origins="*", async_mode='threading')

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
# Nodo ROS2 principal
# ============================================================
class Go2Bridge(Node):
    def __init__(self):
        super().__init__('go2_web_bridge')

        # Suscriptor al estado del robot
        self.subscription = self.create_subscription(
            LowState if LowState else type('LowState', (), {}),
            '/lf/lowstate',
            self.lowstate_callback,
            10
        )

        # Publisher de waypoints
        self.waypoint_pub = self.create_publisher(PointStamped, '/way_point', 10)

        # Timer para verificar conexión
        self.last_msg_time = self.get_clock().now()
        self.create_timer(2.0, self.check_connection)

        self.get_logger().info('🚀 Go2 Web Bridge iniciado')
        self.get_logger().info('📡 Suscrito a /lf/lowstate')

    def check_connection(self):
        """Verifica si seguimos recibiendo mensajes"""
        time_diff = (self.get_clock().now() - self.last_msg_time).nanoseconds / 1e9
        with state_lock:
            robot_state['connected'] = time_diff < 3.0
        if time_diff > 5.0:
            self.get_logger().warn('⚠️  No se han recibido mensajes en 5 segundos')

    def lowstate_callback(self, msg):
        """Callback para LowState"""
        self.last_msg_time = self.get_clock().now()
        with state_lock:
            # Datos mínimos para el dashboard
            if hasattr(msg, 'bms_state'):
                bms = msg.bms_state
                robot_state['battery']['soc'] = getattr(bms, 'soc', 0)
                robot_state['battery']['voltage'] = sum(getattr(bms, 'cell_vol', [0]*8)[:8]) / 1000.0
                robot_state['battery']['current'] = getattr(bms, 'current', 0) / 1000.0

            if hasattr(msg, 'imu_state'):
                imu = msg.imu_state
                robot_state['imu']['accelerometer'] = list(getattr(imu, 'accelerometer', [0, 0, 0]))

            if hasattr(msg, 'foot_force'):
                robot_state['foot_force'] = list(msg.foot_force)

            robot_state['timestamp'] = datetime.now().isoformat()
            robot_state['connected'] = True

        try:
            serializable_state = convert_to_serializable(robot_state)
            socketio.emit('robot_update', serializable_state)
        except Exception as e:
            self.get_logger().error(f"Error serializando robot_state: {e}")


# ============================================================
# API REST
# ============================================================
@app.route('/api/status')
def get_status():
    with state_lock:
        return jsonify(robot_state)


@app.route('/')
def serve_index():
    """Sirve el dashboard principal"""
    return open(os.path.join(WEB_DIR, 'index.html')).read()


# ============================================================
# WebSocket: eventos desde el dashboard
# ============================================================
@socketio.on('connect')
def handle_connect(auth=None):
    print("✅ Cliente web conectado")
    emit('robot_update', convert_to_serializable(robot_state))


@socketio.on('disconnect')
def handle_disconnect():
    print('❌ Cliente web desconectado')


@socketio.on('home_command')
def handle_home_command():
    """Comando enviado desde el botón 🏠"""
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
# Ejecución
# ============================================================
def run_flask():
    """Ejecuta Flask en hilo paralelo"""
    socketio.run(app, host='0.0.0.0', port=5000, allow_unsafe_werkzeug=True)


def main(args=None):
    global node
    rclpy.init(args=args)
    node = Go2Bridge()

    flask_thread = threading.Thread(target=run_flask, daemon=True)
    flask_thread.start()

    print("=" * 60)
    print("🤖 Unitree Go2 Web Bridge corriendo")
    print("🌐 http://<IP_DEL_ROBOT>:5000/")
    print("=" * 60)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n👋 Cerrando Go2 Web Bridge...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

