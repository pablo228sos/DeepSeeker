"""
simulation.py — программная симуляция дронов через MAVLink UDP (без SITL)
Запускает виртуальных дронов, которые реально общаются по MAVLink протоколу.
"""
from __future__ import annotations
import math
import time
import threading
import socket
import struct
import random
from typing import List


class SimDrone:
    """Виртуальный дрон — генерирует MAVLink пакеты в UDP-сокет."""

    HEARTBEAT_ID = 0
    SYS_STATUS_ID = 1
    GPS_RAW_INT_ID = 24
    ATTITUDE_ID = 30
    GLOBAL_POS_ID = 33
    VFR_HUD_ID = 74
    RADIO_STATUS_ID = 109
    MISSION_CURRENT_ID = 42
    NAV_CTRL_ID = 62

    def __init__(self, sysid: int, lat: float, lon: float, alt: float, port: int):
        self.sysid   = sysid
        self.lat     = lat
        self.lon     = lon
        self.alt     = alt
        self.port    = port
        self.heading = random.uniform(0, 360)
        self.speed   = random.uniform(5, 15)
        self.roll    = 0.0
        self.pitch   = 0.0
        self.yaw     = math.radians(self.heading)
        self.battery = random.uniform(60, 100)
        self.armed   = False
        self.mode    = 0   # STABILIZE
        self.mode_name = 'STABILIZE'
        self.wp_idx  = 0
        self.seq     = 0
        self._running = False
        self._sock   = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._thread: threading.Thread | None = None
        self._waypoints = []

    def start(self):
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def stop(self):
        self._running = False
        self._sock.close()

    def arm(self):
        self.armed = True
        self.mode = 3
        self.mode_name = 'AUTO'

    def set_waypoints(self, wps):
        self._waypoints = wps

    def _loop(self):
        hz4  = 0
        hz10 = 0
        hz1  = 0
        while self._running:
            now = time.time()
            self._move()
            if now - hz4 >= 0.25:
                hz4 = now
                self._send_heartbeat()
                self._send_gps()
                self._send_global_pos()
                self._send_vfr()
            if now - hz10 >= 0.1:
                hz10 = now
                self._send_attitude()
            if now - hz1 >= 1.0:
                hz1 = now
                self._send_sys_status()
                self._send_mission_current()
            time.sleep(0.05)

    def _move(self):
        """Плавное движение по маршруту или random walk."""
        if self._waypoints and self.armed:
            wp = self._waypoints[self.wp_idx % len(self._waypoints)]
            dlat = wp[0] - self.lat
            dlon = wp[1] - self.lon
            dist = math.hypot(dlat * 111320,
                              dlon * 111320 * math.cos(math.radians(self.lat)))
            if dist < 3:
                self.wp_idx = (self.wp_idx + 1) % len(self._waypoints)
            else:
                ang = math.atan2(dlon, dlat)
                step = self.speed * 0.05
                self.lat += step * math.cos(ang) / 111320
                self.lon += step * math.sin(ang) / (111320 * math.cos(math.radians(self.lat)))
                self.yaw  = ang
        else:
            # random walk
            self.heading += random.uniform(-2, 2)
            self.yaw = math.radians(self.heading)
            step = self.speed * 0.05
            self.lat += step * math.cos(self.yaw) / 111320
            self.lon += step * math.sin(self.yaw) / (111320 * math.cos(math.radians(self.lat)))

        self.alt += random.uniform(-0.2, 0.2)
        self.alt  = max(10, min(200, self.alt))
        self.roll  = random.uniform(-3, 3)
        self.pitch = random.uniform(-2, 2)
        self.battery -= 0.0002

    # ── MAVLink packet builders (minimal, no external lib needed) ─

    def _pack(self, msg_id: int, payload: bytes) -> bytes:
        length = len(payload)
        self.seq = (self.seq + 1) & 0xFF
        hdr = struct.pack('BBBBBB', 0xFE, length, self.seq,
                          self.sysid, 1, msg_id)
        crc = self._crc(hdr[1:] + payload, msg_id)
        return hdr + payload + struct.pack('<H', crc)

    def _send(self, data: bytes):
        try:
            self._sock.sendto(data, ('127.0.0.1', self.port))
        except Exception:
            pass

    def _send_heartbeat(self):
        base_mode = 0b10000001 if self.armed else 0b00000001
        payload = struct.pack('<IBBBBB',
            0,           # custom_mode
            6,           # MAV_TYPE_HELICOPTER
            8,           # MAV_AUTOPILOT_ARDUPILOTMEGA
            base_mode,
            0,           # system_status MAV_STATE_ACTIVE
            3            # mavlink_version
        )
        self._send(self._pack(0, payload))

    def _send_global_pos(self):
        payload = struct.pack('<qiiiiHH',
            int(time.time() * 1000) & 0x7FFFFFFFFFFFFFFF,
            int(self.lat * 1e7),
            int(self.lon * 1e7),
            int(self.alt * 1000),
            int(self.alt * 1000),  # relative_alt
            0, 0,                  # vx, vy
            int(self.yaw * 100)    # hdg
        )
        # pad to correct length
        self._send(self._pack(33, payload))

    def _send_gps(self):
        payload = struct.pack('<QiiiHHHHBB',
            int(time.time() * 1000) & 0x7FFFFFFFFFFFFFFF,
            int(self.lat * 1e7),
            int(self.lon * 1e7),
            int(self.alt * 1000),
            80,   # eph (HDOP *100)
            65535,  # epv
            int(self.speed * 100),
            int(self.speed * 100),
            int(self.heading * 100) % 36000,
            14,   # satellites
            3,    # fix_type 3D_FIX
        )
        self._send(self._pack(24, payload[:30]))

    def _send_attitude(self):
        payload = struct.pack('<Iffffff',
            int(time.time() * 1000) & 0x7FFFFFFF,
            math.radians(self.roll),
            math.radians(self.pitch),
            self.yaw,
            0.0, 0.0, 0.0
        )
        self._send(self._pack(30, payload))

    def _send_vfr(self):
        payload = struct.pack('<ffffHf',
            self.speed,
            self.speed,
            0.0,
            math.degrees(self.yaw) % 360,
            int(self.alt),
            random.uniform(-0.3, 0.3)
        )
        self._send(self._pack(74, payload))

    def _send_sys_status(self):
        payload = struct.pack('<IIIHhHhHHHHHH',
            0, 0, 0,
            500,                    # load
            int(self.battery * 168),# voltage_battery (mV)
            -1,                     # current_battery
            int(self.battery),      # battery_remaining %
            0, 0, 0, 0, 0, 0
        )
        self._send(self._pack(1, payload))

    def _send_mission_current(self):
        payload = struct.pack('<H', self.wp_idx)
        self._send(self._pack(42, payload))

    @staticmethod
    def _crc(data: bytes, msg_id: int) -> int:
        CRC_EXTRA = {
            0: 50, 1: 124, 24: 24, 30: 39, 33: 104,
            42: 28, 62: 183, 74: 20, 109: 21
        }
        crc = 0xFFFF
        for b in data:
            tmp = b ^ (crc & 0xFF)
            tmp = (tmp ^ (tmp << 4)) & 0xFF
            crc = ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF
        extra = CRC_EXTRA.get(msg_id, 0)
        tmp = extra ^ (crc & 0xFF)
        tmp = (tmp ^ (tmp << 4)) & 0xFF
        crc = ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF
        return crc


class SimulationEngine:
    """
    Запускает N виртуальных дронов. Каждый слушает на своём UDP-порту.
    GCS коннектится к udp:127.0.0.1:<port>.
    """
    BASE_PORT   = 14550
    CENTER_LAT  = 41.0082
    CENTER_LON  = 28.9784
    BASE_ALT    = 50.0

    def __init__(self):
        self._sims: List[SimDrone] = []
        self._running = False

    def start(self, count: int = 6) -> List[dict]:
        """Запускает count симулируемых дронов. Возвращает список параметров подключения."""
        self._running = True
        connections = []
        for i in range(count):
            port = self.BASE_PORT + i
            angle = 2 * math.pi * i / count
            lat = self.CENTER_LAT + 0.005 * math.cos(angle)
            lon = self.CENTER_LON + 0.005 * math.sin(angle)
            # Create receiving socket for this sim drone
            recv_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            try:
                recv_sock.bind(('127.0.0.1', port))
            except OSError:
                port = self.BASE_PORT + 100 + i
                recv_sock.bind(('127.0.0.1', port))
            recv_sock.close()

            sim = SimDrone(sysid=i+1, lat=lat, lon=lon,
                           alt=self.BASE_ALT + i*10, port=port)
            sim.start()
            self._sims.append(sim)
            connections.append({
                'id':   f'SIM-{i+1:03d}',
                'name': f'SIM-{i+1:03d}',
                'host': '127.0.0.1',
                'port': port,
                'type': 'udp',
            })
        return connections

    def stop(self):
        self._running = False
        for s in self._sims:
            s.stop()
        self._sims.clear()

    def set_waypoints(self, wps):
        for s in self._sims:
            s.set_waypoints(wps)

    def arm_all(self):
        for s in self._sims:
            s.arm()

    @property
    def sim_drones(self):
        return self._sims
