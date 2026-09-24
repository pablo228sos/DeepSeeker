"""
drone.py — модель одного дрона (MAVLink state machine)
"""
from __future__ import annotations
import time
import threading
from dataclasses import dataclass, field
from typing import Optional, List, Dict, Any
from enum import Enum


class DroneStatus(Enum):
    DISCONNECTED = "disconnected"
    CONNECTING   = "connecting"
    CONNECTED    = "connected"
    ARMED        = "armed"
    MISSION      = "mission"
    ERROR        = "error"


class ConnectionType(Enum):
    UDP    = "udp"
    SERIAL = "serial"
    SIM    = "sim"


@dataclass
class Telemetry:
    # Position
    lat: float = 0.0
    lon: float = 0.0
    alt_rel: float = 0.0      # relative altitude, m
    alt_abs: float = 0.0      # absolute altitude, m
    # Velocity
    vx: float = 0.0
    vy: float = 0.0
    vz: float = 0.0
    groundspeed: float = 0.0
    airspeed: float = 0.0
    # Attitude
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0
    rollspeed: float = 0.0
    pitchspeed: float = 0.0
    yawspeed: float = 0.0
    # Power
    battery_pct: int = 100
    voltage: float = 0.0
    current: float = 0.0
    # GPS
    satellites: int = 0
    hdop: float = 99.9
    fix_type: int = 0
    # System
    mode: str = "UNKNOWN"
    armed: bool = False
    heading: float = 0.0
    climb: float = 0.0
    wp_index: int = 0
    wp_dist: float = 0.0
    wp_count: int = 0
    rssi: int = 0
    # EKF
    ekf_ok: bool = False
    # Vibration
    vib_x: float = 0.0
    vib_y: float = 0.0
    vib_z: float = 0.0
    # Timestamps
    last_heartbeat: float = 0.0
    flight_start: float = 0.0


@dataclass
class Waypoint:
    index: int
    command: int          # MAV_CMD
    lat: float
    lon: float
    alt: float
    param1: float = 0.0
    param2: float = 0.0
    param3: float = 0.0
    param4: float = 0.0
    frame: int = 3        # MAV_FRAME_GLOBAL_RELATIVE_ALT
    autocontinue: int = 1


class Drone:
    """
    Представляет один БПЛА. Управляет MAVLink-соединением в фоновом потоке.
    Все данные хранятся в self.telem и обновляются из потока.
    """
    def __init__(self, drone_id: str, name: str,
                 conn_type: ConnectionType,
                 host: str = "127.0.0.1",
                 port: int = 14550,
                 serial_port: str = "COM3",
                 baud: int = 57600,
                 sysid: int = 1):
        self.id          = drone_id
        self.name        = name
        self.conn_type   = conn_type
        self.host        = host
        self.port        = port
        self.serial_port = serial_port
        self.baud        = baud
        self.sysid       = sysid

        self.status      = DroneStatus.DISCONNECTED
        self.telem       = Telemetry()
        self.waypoints: List[Waypoint] = []
        self.params: Dict[str, Any] = {}
        self.track: List[tuple] = []          # [(lat,lon), ...]

        self._mav        = None               # MAVLink connection
        self._thread: Optional[threading.Thread] = None
        self._lock       = threading.Lock()
        self._running    = False
        self._callbacks  = []                 # (event, callable)

        # Mode map (ArduCopter)
        self._mode_map = {
            0:'STABILIZE', 1:'ACRO', 2:'ALT_HOLD', 3:'AUTO',
            4:'GUIDED', 5:'LOITER', 6:'RTL', 7:'CIRCLE',
            9:'LAND', 11:'DRIFT', 13:'SPORT', 14:'FLIP',
            15:'AUTOTUNE', 16:'POSHOLD', 17:'BRAKE',
            18:'THROW', 19:'AVOID_ADSB', 20:'GUIDED_NOGPS',
            21:'SMART_RTL', 22:'FLOWHOLD', 23:'FOLLOW',
            24:'ZIGZAG',
        }

    # ── Public API ────────────────────────────────────────────────

    def connect(self):
        if self._running:
            return
        self._running = True
        self.status = DroneStatus.CONNECTING
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def disconnect(self):
        self._running = False
        if self._mav:
            try:
                self._mav.close()
            except Exception:
                pass
        self.status = DroneStatus.DISCONNECTED

    def arm(self):
        self._send_command(400, param1=1)

    def disarm(self):
        self._send_command(400, param1=0)

    def set_mode(self, mode_name: str):
        if not self._mav:
            return
        inv = {v: k for k, v in self._mode_map.items()}
        mode_id = inv.get(mode_name.upper())
        if mode_id is None:
            return
        try:
            from pymavlink import mavutil
            self._mav.set_mode(mode_id)
        except Exception as e:
            self._emit('error', str(e))

    def takeoff(self, alt: float = 20.0):
        self._send_command(22, param7=alt)   # MAV_CMD_NAV_TAKEOFF

    def land(self):
        self.set_mode('LAND')

    def rtl(self):
        self.set_mode('RTL')

    def goto(self, lat: float, lon: float, alt: float):
        if not self._mav:
            return
        try:
            from pymavlink import mavutil
            self.set_mode('GUIDED')
            self._mav.mav.mission_item_int_send(
                self._mav.target_system,
                self._mav.target_component,
                0, 3,   # seq, frame GLOBAL_RELATIVE_ALT
                16,     # MAV_CMD_NAV_WAYPOINT
                2, 1,   # current=guided, autocontinue
                0, 0, 0, 0,
                int(lat * 1e7), int(lon * 1e7), alt
            )
        except Exception as e:
            self._emit('error', str(e))

    def upload_mission(self, wps: List[Waypoint]):
        if not self._mav:
            return
        try:
            from pymavlink import mavutil
            self._mav.waypoint_clear_all_send()
            time.sleep(0.3)
            self._mav.waypoint_count_send(len(wps))
            for i, wp in enumerate(wps):
                msg = self._mav.recv_match(type='MISSION_REQUEST', blocking=True, timeout=3)
                if msg:
                    self._mav.mav.mission_item_send(
                        self._mav.target_system,
                        self._mav.target_component,
                        wp.index, wp.frame, wp.command,
                        1 if i == 0 else 0,
                        wp.autocontinue,
                        wp.param1, wp.param2, wp.param3, wp.param4,
                        wp.lat, wp.lon, wp.alt
                    )
            self._emit('mission_uploaded', len(wps))
        except Exception as e:
            self._emit('error', str(e))

    def download_mission(self):
        if not self._mav:
            return
        try:
            self._mav.waypoint_request_list_send()
            count_msg = self._mav.recv_match(type='MISSION_COUNT', blocking=True, timeout=5)
            if not count_msg:
                return
            wps = []
            for i in range(count_msg.count):
                self._mav.waypoint_request_send(i)
                msg = self._mav.recv_match(type='MISSION_ITEM', blocking=True, timeout=5)
                if msg:
                    wps.append(Waypoint(
                        index=msg.seq, command=msg.command,
                        lat=msg.x, lon=msg.y, alt=msg.z,
                        param1=msg.param1, param2=msg.param2,
                        param3=msg.param3, param4=msg.param4,
                        frame=msg.frame, autocontinue=msg.autocontinue
                    ))
            self.waypoints = wps
            self._emit('mission_downloaded', wps)
        except Exception as e:
            self._emit('error', str(e))

    def start_mission(self):
        self.set_mode('AUTO')
        self._send_command(300, param1=0)    # MAV_CMD_MISSION_START

    def request_params(self):
        if not self._mav:
            return
        try:
            self._mav.param_fetch_all()
        except Exception:
            pass

    def set_param(self, name: str, value: float):
        if not self._mav:
            return
        try:
            self._mav.param_set_send(name.encode(), value)
        except Exception as e:
            self._emit('error', str(e))

    def on(self, event: str, cb):
        self._callbacks.append((event, cb))

    def flight_time(self) -> str:
        if self.telem.flight_start == 0:
            return "00:00"
        secs = int(time.time() - self.telem.flight_start)
        return f"{secs//60:02d}:{secs%60:02d}"

    def signal_strength(self) -> int:
        """0-4 based on rssi"""
        r = abs(self.telem.rssi)
        if r < 60: return 4
        if r < 70: return 3
        if r < 80: return 2
        if r < 90: return 1
        return 0

    # ── Internal ──────────────────────────────────────────────────

    def _run(self):
        try:
            from pymavlink import mavutil
            cs = self._connection_string()
            self._mav = mavutil.mavlink_connection(cs, baud=self.baud,
                                                   source_system=255)
            self._mav.wait_heartbeat(timeout=10)
            self.status = DroneStatus.CONNECTED
            self._mav.mav.request_data_stream_send(
                self._mav.target_system,
                self._mav.target_component,
                mavutil.mavlink.MAV_DATA_STREAM_ALL, 4, 1
            )
            self._emit('connected', self.name)
            while self._running:
                msg = self._mav.recv_match(blocking=True, timeout=1.0)
                if msg:
                    self._handle_msg(msg)
                self._check_heartbeat()
        except Exception as e:
            self.status = DroneStatus.ERROR
            self._emit('error', str(e))

    def _connection_string(self) -> str:
        if self.conn_type == ConnectionType.UDP:
            return f"udp:{self.host}:{self.port}"
        elif self.conn_type == ConnectionType.SERIAL:
            return self.serial_port
        else:  # SIM
            return f"udp:127.0.0.1:{self.port}"

    def _handle_msg(self, msg):
        t = msg.get_type()
        with self._lock:
            if t == 'HEARTBEAT':
                self.telem.last_heartbeat = time.time()
                self.telem.armed = bool(msg.base_mode & 128)
                mode_id = msg.custom_mode
                self.telem.mode = self._mode_map.get(mode_id, str(mode_id))
                if self.telem.armed and self.telem.flight_start == 0:
                    self.telem.flight_start = time.time()
                if not self.telem.armed:
                    self.telem.flight_start = 0
                if self.telem.armed:
                    self.status = DroneStatus.ARMED
                else:
                    self.status = DroneStatus.CONNECTED

            elif t == 'GLOBAL_POSITION_INT':
                self.telem.lat     = msg.lat / 1e7
                self.telem.lon     = msg.lon / 1e7
                self.telem.alt_rel = msg.relative_alt / 1000.0
                self.telem.alt_abs = msg.alt / 1000.0
                self.telem.vx      = msg.vx / 100.0
                self.telem.vy      = msg.vy / 100.0
                self.telem.vz      = msg.vz / 100.0
                self.telem.heading = msg.hdg / 100.0
                if self.telem.lat != 0 and self.telem.lon != 0:
                    self.track.append((self.telem.lat, self.telem.lon))
                    if len(self.track) > 500:
                        self.track.pop(0)

            elif t == 'VFR_HUD':
                self.telem.groundspeed = msg.groundspeed
                self.telem.airspeed    = msg.airspeed
                self.telem.climb       = msg.climb
                self.telem.alt_rel     = msg.alt

            elif t == 'ATTITUDE':
                import math
                self.telem.roll       = math.degrees(msg.roll)
                self.telem.pitch      = math.degrees(msg.pitch)
                self.telem.yaw        = math.degrees(msg.yaw)
                self.telem.rollspeed  = math.degrees(msg.rollspeed)
                self.telem.pitchspeed = math.degrees(msg.pitchspeed)
                self.telem.yawspeed   = math.degrees(msg.yawspeed)

            elif t == 'SYS_STATUS':
                self.telem.battery_pct = msg.battery_remaining
                self.telem.voltage     = msg.voltage_battery / 1000.0
                self.telem.current     = msg.current_battery / 100.0

            elif t == 'GPS_RAW_INT':
                self.telem.satellites = msg.satellites_visible
                self.telem.hdop       = msg.eph / 100.0
                self.telem.fix_type   = msg.fix_type

            elif t == 'MISSION_CURRENT':
                self.telem.wp_index = msg.seq

            elif t == 'NAV_CONTROLLER_OUTPUT':
                self.telem.wp_dist = msg.wp_dist

            elif t == 'MISSION_COUNT':
                self.telem.wp_count = msg.count

            elif t == 'PARAM_VALUE':
                self.params[msg.param_id.rstrip('\x00')] = msg.param_value

            elif t == 'RADIO_STATUS':
                self.telem.rssi = msg.rssi - 256 if msg.rssi > 127 else msg.rssi

            elif t == 'VIBRATION':
                self.telem.vib_x = msg.vibration_x
                self.telem.vib_y = msg.vibration_y
                self.telem.vib_z = msg.vibration_z

            elif t == 'EKF_STATUS_REPORT':
                flags = msg.flags
                self.telem.ekf_ok = bool(flags & 0x1F == 0x1F)

        self._emit('telemetry', self.telem)

    def _check_heartbeat(self):
        if (self.telem.last_heartbeat > 0 and
                time.time() - self.telem.last_heartbeat > 5):
            self.status = DroneStatus.ERROR
            self._emit('timeout', self.name)

    def _send_command(self, cmd, param1=0, param2=0, param3=0,
                       param4=0, param5=0, param6=0, param7=0):
        if not self._mav:
            return
        try:
            self._mav.mav.command_long_send(
                self._mav.target_system,
                self._mav.target_component,
                cmd, 0,
                param1, param2, param3, param4, param5, param6, param7
            )
        except Exception as e:
            self._emit('error', str(e))

    def _emit(self, event: str, data=None):
        for ev, cb in self._callbacks:
            if ev == event:
                try:
                    cb(data)
                except Exception:
                    pass
