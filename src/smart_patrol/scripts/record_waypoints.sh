#!/bin/bash

# Script para facilitar grabación de waypoints
# Uso: ./record_waypoints.sh [nombre_archivo.yaml]

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
WORKSPACE_DIR="$(dirname "$(dirname "$SCRIPT_DIR")")"

# Nombre del archivo de salida
OUTPUT_FILE="${1:-my_route_$(date +%Y%m%d_%H%M%S).yaml}"

echo "======================================================================"
echo "   🎮 SISTEMA DE GRABACIÓN DE WAYPOINTS"
echo "======================================================================"
echo ""
echo "📁 Archivo de salida: $OUTPUT_FILE"
echo ""
echo "INSTRUCCIONES:"
echo "1. Asegúrate de que el autonomy_stack esté corriendo"
echo "2. Teleopera el robot con el joystick/teclado"
echo "3. En esta ventana, presiona ENTER para grabar cada waypoint"
echo "4. Escribe 'done' para terminar y guardar"
echo ""
echo "======================================================================"
echo ""

# Source workspace
cd "$WORKSPACE_DIR"
source install/setup.bash

# Lanzar recorder
ros2 run smart_patrol waypoint_recorder --ros-args \
    -p output_file:="$OUTPUT_FILE" \
    -p odom_topic:="/state_estimation"
