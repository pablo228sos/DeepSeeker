"""
swarm.py — управление роем БПЛА
"""
from __future__ import annotations
import math
import threading
from typing import List, Dict, Optional
from .drone import Drone, Waypoint, DroneStatus


class Formation:
    @staticmethod
    def line(center_lat, center_lon, count, spacing=20.0, heading=0.0):
        """Линейная формация"""
        points = []
        angle = math.radians(heading + 90)
        for i in range(count):
            offset = (i - (count - 1) / 2) * spacing
            dlat = offset * math.cos(angle) / 111320
            dlon = offset * math.sin(angle) / (111320 * math.cos(math.radians(center_lat)))
            points.append((center_lat + dlat, center_lon + dlon))
        return points

    @staticmethod
    def grid(center_lat, center_lon, count, spacing=20.0):
        """Сетка NxN"""
        cols = math.ceil(math.sqrt(count))
        points = []
        for i in range(count):
            row = i // cols
            col = i % cols
            rows = math.ceil(count / cols)
            dlat = (row - (rows - 1) / 2) * spacing / 111320
            dlon = (col - (cols - 1) / 2) * spacing / (111320 * math.cos(math.radians(center_lat)))
            points.append((center_lat + dlat, center_lon + dlon))
        return points

    @staticmethod
    def circle(center_lat, center_lon, count, radius=50.0):
        """Круговая формация"""
        points = []
        for i in range(count):
            angle = 2 * math.pi * i / count
            dlat = radius * math.cos(angle) / 111320
            dlon = radius * math.sin(angle) / (111320 * math.cos(math.radians(center_lat)))
            points.append((center_lat + dlat, center_lon + dlon))
        return points

    @staticmethod
    def v_shape(center_lat, center_lon, count, spacing=25.0, heading=0.0):
        """V-образная формация"""
        points = [(center_lat, center_lon)]
        angle = math.radians(heading)
        left_angle  = math.radians(heading + 135)
        right_angle = math.radians(heading - 135)
        for i in range(1, count):
            side = i % 2
            idx  = (i + 1) // 2
            a = left_angle if side == 0 else right_angle
            dlat = idx * spacing * math.cos(a) / 111320
            dlon = idx * spacing * math.sin(a) / (111320 * math.cos(math.radians(center_lat)))
            points.append((center_lat + dlat, center_lon + dlon))
        return points


class Swarm:
    """
    Менеджер роя. Хранит список дронов, предоставляет групповые команды.
    """
    def __init__(self):
        self.drones: List[Drone] = []
        self._lock = threading.Lock()
        self._callbacks = []

    # ── Fleet management ──────────────────────────────────────────

    def add_drone(self, drone: Drone):
        with self._lock:
            self.drones.append(drone)
        drone.on('telemetry', lambda t: self._on_telem(drone, t))
        drone.on('connected', lambda n: self._emit('fleet_changed', None))
        drone.on('error',     lambda e: self._emit('drone_error', (drone, e)))
        self._emit('fleet_changed', None)

    def remove_drone(self, drone_id: str):
        with self._lock:
            d = self._get(drone_id)
            if d:
                d.disconnect()
                self.drones.remove(d)
        self._emit('fleet_changed', None)

    def get_drone(self, drone_id: str) -> Optional[Drone]:
        return self._get(drone_id)

    def online_drones(self) -> List[Drone]:
        return [d for d in self.drones
                if d.status not in (DroneStatus.DISCONNECTED, DroneStatus.ERROR)]

    def armed_drones(self) -> List[Drone]:
        return [d for d in self.drones if d.telem.armed]

    # ── Group commands ────────────────────────────────────────────

    def arm_all(self):
        for d in self.online_drones():
            d.arm()

    def disarm_all(self):
        for d in self.drones:
            d.disarm()

    def rtl_all(self):
        for d in self.online_drones():
            d.rtl()

    def land_all(self):
        for d in self.online_drones():
            d.land()

    def set_mode_all(self, mode: str):
        for d in self.online_drones():
            d.set_mode(mode)

    def upload_mission_all(self, waypoints: List[Waypoint]):
        for d in self.online_drones():
            threading.Thread(target=d.upload_mission,
                             args=(waypoints,), daemon=True).start()

    def start_mission_all(self):
        for d in self.online_drones():
            d.start_mission()

    # ── Formations ────────────────────────────────────────────────

    def apply_formation(self, formation: str, center_lat: float,
                         center_lon: float, alt: float,
                         spacing: float = 25.0, heading: float = 0.0):
        dlist = self.online_drones()
        n = len(dlist)
        if n == 0:
            return

        if formation == 'line':
            pts = Formation.line(center_lat, center_lon, n, spacing, heading)
        elif formation == 'grid':
            pts = Formation.grid(center_lat, center_lon, n, spacing)
        elif formation == 'circle':
            pts = Formation.circle(center_lat, center_lon, n, spacing * 2)
        elif formation == 'v':
            pts = Formation.v_shape(center_lat, center_lon, n, spacing, heading)
        else:
            return

        for drone, (lat, lon) in zip(dlist, pts):
            drone.goto(lat, lon, alt)

        self._emit('formation_applied', {'formation': formation, 'count': n})

    # ── Stats ─────────────────────────────────────────────────────

    def stats(self) -> dict:
        return {
            'total':      len(self.drones),
            'online':     len(self.online_drones()),
            'armed':      len(self.armed_drones()),
            'mission':    len([d for d in self.drones if d.telem.mode == 'AUTO']),
            'error':      len([d for d in self.drones if d.status == DroneStatus.ERROR]),
        }

    def center_of_mass(self) -> tuple:
        dlist = [d for d in self.online_drones() if d.telem.lat != 0]
        if not dlist:
            return (0.0, 0.0)
        return (
            sum(d.telem.lat for d in dlist) / len(dlist),
            sum(d.telem.lon for d in dlist) / len(dlist),
        )

    # ── Events ────────────────────────────────────────────────────

    def on(self, event: str, cb):
        self._callbacks.append((event, cb))

    def _emit(self, event, data):
        for ev, cb in self._callbacks:
            if ev == event:
                try:
                    cb(data)
                except Exception:
                    pass

    def _on_telem(self, drone: Drone, telem):
        self._emit('telemetry', (drone, telem))

    def _get(self, drone_id: str) -> Optional[Drone]:
        for d in self.drones:
            if d.id == drone_id:
                return d
        return None
