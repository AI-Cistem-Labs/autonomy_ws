#!/usr/bin/env python3
"""
Gestor de Waypoints para Smart Patrol
Maneja carga, validación y tracking de waypoints
"""

import yaml
import os
from typing import List, Dict, Optional
from geometry_msgs.msg import Point

class Waypoint:
    """Clase para representar un waypoint individual"""
    def __init__(self, name: str, x: float, y: float, z: float = 0.0):
        self.name = name
        self.x = x
        self.y = y
        self.z = z
        self.visited = False
        self.visit_count = 0
    
    def to_point(self) -> Point:
        """Convertir a geometry_msgs/Point"""
        point = Point()
        point.x = self.x
        point.y = self.y
        point.z = self.z
        return point
    
    def __str__(self):
        return f"{self.name} ({self.x:.2f}, {self.y:.2f}, {self.z:.2f})"

class WaypointManager:
    """Gestor de waypoints para el sistema de patrullaje"""
    
    def __init__(self, logger):
        self.logger = logger
        self.waypoints: List[Waypoint] = []
        self.current_index = 0
        self.total_loops = 0
        self.direction = 1  # ← NUEVO: 1 = forward, -1 = backward
        self.ping_pong_mode = True  # ← NUEVO: True = ida/vuelta, False = loop directo
    
    def load_from_yaml(self, filepath: str) -> bool:
        """Cargar waypoints desde archivo YAML"""
        if not os.path.exists(filepath):
            self.logger.error(f'❌ Archivo no encontrado: {filepath}')
            return False
        
        try:
            with open(filepath, 'r') as f:
                data = yaml.safe_load(f)
            
            if not data or 'waypoints' not in data:
                self.logger.error('❌ Archivo YAML mal formado')
                return False
            
            # Cargar waypoints
            self.waypoints.clear()
            waypoints_dict = data['waypoints']
            
            for name, wp_data in waypoints_dict.items():
                waypoint = Waypoint(
                    name=name,
                    x=wp_data['x'],
                    y=wp_data['y'],
                    z=wp_data.get('z', 0.0)
                )
                self.waypoints.append(waypoint)
            
            self.logger.info(f'✅ Cargados {len(self.waypoints)} waypoints desde {filepath}')
            return True
            
        except Exception as e:
            self.logger.error(f'❌ Error cargando waypoints: {e}')
            return False
    
    def set_ping_pong_mode(self, enabled: bool):
        """Activar/desactivar modo ping-pong (ida y vuelta)"""
        self.ping_pong_mode = enabled
        mode_str = "IDA Y VUELTA" if enabled else "LOOP DIRECTO"
        self.logger.info(f'🔄 Modo de patrullaje: {mode_str}')
    
    def get_current_waypoint(self) -> Optional[Waypoint]:
        """Obtener waypoint actual"""
        if not self.waypoints:
            return None
        return self.waypoints[self.current_index]
    
    def get_next_waypoint(self) -> Optional[Waypoint]:
        """Avanzar al siguiente waypoint (con soporte ping-pong)"""
        if not self.waypoints:
            return None
        
        # Marcar actual como visitado
        current = self.waypoints[self.current_index]
        current.visited = True
        current.visit_count += 1
        
        # MODO PING-PONG (ida y vuelta)
        if self.ping_pong_mode:
            # Avanzar en la dirección actual
            self.current_index += self.direction
            
            # Verificar límites y cambiar dirección si es necesario
            if self.current_index >= len(self.waypoints):
                # Llegamos al final, ir hacia atrás
                self.current_index = len(self.waypoints) - 2  # Penúltimo
                self.direction = -1
                self.logger.info('🔄 Llegó al final - Regresando')
            
            elif self.current_index < 0:
                # Llegamos al inicio, ir hacia adelante
                self.current_index = 1  # Segundo
                self.direction = 1
                self.total_loops += 1
                self.logger.info(f'🔄 Loop {self.total_loops} completado (ida y vuelta)')
        
        # MODO LOOP DIRECTO (comportamiento original)
        else:
            self.current_index += 1
            
            # Verificar si completamos un loop
            if self.current_index >= len(self.waypoints):
                self.current_index = 0
                self.total_loops += 1
                self.logger.info(f'🔄 Loop {self.total_loops} completado')
        
        return self.waypoints[self.current_index]
    
    def get_waypoint_by_name(self, name: str) -> Optional[Waypoint]:
        """Buscar waypoint por nombre"""
        for wp in self.waypoints:
            if wp.name == name:
                return wp
        return None
    
    def set_current_by_name(self, name: str) -> bool:
        """Saltar a un waypoint específico por nombre"""
        for i, wp in enumerate(self.waypoints):
            if wp.name == name:
                self.current_index = i
                self.logger.info(f'🎯 Saltando a waypoint: {name}')
                return True
        return False
    
    def reset(self):
        """Reiniciar al primer waypoint"""
        self.current_index = 0
        self.direction = 1  # ← NUEVO: Resetear dirección
        for wp in self.waypoints:
            wp.visited = False
    
    def get_route_summary(self) -> str:
        """Obtener resumen de la ruta"""
        if not self.waypoints:
            return "Sin waypoints cargados"
        
        mode = "IDA Y VUELTA" if self.ping_pong_mode else "LOOP DIRECTO"
        direction_arrow = "→" if self.direction == 1 else "←"
        
        summary = f"Ruta: {len(self.waypoints)} waypoints [{mode}]\n"
        summary += f"Dirección actual: {direction_arrow}\n"
        
        for i, wp in enumerate(self.waypoints):
            marker = "→" if i == self.current_index else " "
            summary += f"{marker} {i+1}. {wp.name}\n"
        return summary
