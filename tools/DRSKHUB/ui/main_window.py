from __future__ import annotations
import sys, os, time, threading
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from PyQt6.QtWidgets import (QMainWindow, QWidget, QHBoxLayout, QVBoxLayout,
                              QLabel, QPushButton, QTabWidget, QSplitter,
                              QStatusBar, QDialog, QDialogButtonBox,
                              QFormLayout, QComboBox, QLineEdit,
                              QSpinBox, QMessageBox, QApplication, QFrame)
from PyQt6.QtCore import Qt, QTimer, pyqtSignal, QObject
from PyQt6.QtGui import QFont, QIcon

from core.drone import Drone, DroneStatus, ConnectionType, Waypoint
from core.swarm import Swarm
from core.simulation import SimulationEngine
from ui.styles import STYLESHEET, DRONE_COLORS
from ui.fleet_panel import FleetPanel
from ui.map_panel import MapPanel
from ui.telemetry_panel import TelemetryPanel
from ui.mission_panel import MissionPanel
from ui.params_panel import ParamsPanel

DRONE_COLORS_LIST = [
    "#3b82f6","#22c55e","#a855f7","#f97316",
    "#eab308","#ef4444","#06b6d4","#ec4899",
    "#10b981","#f43f5e","#8b5cf6","#14b8a6",
]


class LogEntry(QLabel):
    def __init__(self, text, level="info", parent=None):
        super().__init__(text, parent)
        colors = {"info":"#94a3b8","ok":"#22c55e","warn":"#eab308","error":"#ef4444"}
        self.setStyleSheet(f"color:{colors.get(level,'#94a3b8')};font-size:11px;"
                            "font-family:Consolas,monospace;")


class ConnectDroneDialog(QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Connect Drone")
        self.setMinimumWidth(360)
        self.setStyleSheet("background:#111318;color:#e2e8f0;")

        layout = QVBoxLayout(self)
        form = QFormLayout()
        form.setSpacing(10)

        self.name_edit = QLineEdit()
        self.name_edit.setPlaceholderText("UAV-001")
        self.conn_type = QComboBox()
        self.conn_type.addItems(["UDP (MAVLink)", "Serial / 3DR Radio"])
        self.host_edit = QLineEdit("192.168.1.100")
        self.port_spin = QSpinBox(); self.port_spin.setRange(1, 65535); self.port_spin.setValue(14550)
        self.serial_edit = QLineEdit("COM3")
        self.baud_combo = QComboBox(); self.baud_combo.addItems(["57600","115200","9600","38400"])

        form.addRow("Name:", self.name_edit)
        form.addRow("Connection:", self.conn_type)
        form.addRow("Host/IP:", self.host_edit)
        form.addRow("Port:", self.port_spin)
        form.addRow("Serial Port:", self.serial_edit)
        form.addRow("Baud Rate:", self.baud_combo)
        layout.addLayout(form)

        btns = QDialogButtonBox(QDialogButtonBox.StandardButton.Ok |
                                 QDialogButtonBox.StandardButton.Cancel)
        btns.accepted.connect(self.accept)
        btns.rejected.connect(self.reject)
        layout.addWidget(btns)

        self.conn_type.currentIndexChanged.connect(self._update_form)
        self._update_form(0)

    def _update_form(self, idx):
        is_udp = idx == 0
        self.host_edit.setVisible(is_udp)
        self.port_spin.setVisible(is_udp)
        self.serial_edit.setVisible(not is_udp)
        self.baud_combo.setVisible(not is_udp)

    def get_params(self):
        is_udp = self.conn_type.currentIndex() == 0
        return {
            "name": self.name_edit.text() or "UAV-001",
            "type": "udp" if is_udp else "serial",
            "host": self.host_edit.text(),
            "port": self.port_spin.value(),
            "serial": self.serial_edit.text(),
            "baud": int(self.baud_combo.currentText()),
        }


class SimSetupDialog(QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Start Simulation")
        self.setMinimumWidth(300)
        self.setStyleSheet("background:#111318;color:#e2e8f0;")
        v = QVBoxLayout(self)
        form = QFormLayout()
        self.count_spin = QSpinBox(); self.count_spin.setRange(1, 12); self.count_spin.setValue(6)
        form.addRow("Number of drones:", self.count_spin)
        v.addLayout(form)
        info = QLabel("Simulation uses internal MAVLink over UDP.\n"
                       "No real hardware needed. Each drone gets\n"
                       "its own UDP port starting at 14550.")
        info.setStyleSheet("color:#475569;font-size:11px;margin:8px 0;")
        v.addWidget(info)
        btns = QDialogButtonBox(QDialogButtonBox.StandardButton.Ok |
                                 QDialogButtonBox.StandardButton.Cancel)
        btns.accepted.connect(self.accept)
        btns.rejected.connect(self.reject)
        v.addWidget(btns)


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("DRSKHUB — Swarm Ground Control Station")
        self.resize(1600, 950)
        self.setMinimumSize(1200, 700)

        self._swarm  = Swarm()
        self._sim    = SimulationEngine()
        self._sim_running = False
        self._selected_idx = 0
        self._log_entries  = []

        self.setStyleSheet(STYLESHEET)
        self._build_ui()
        self._setup_timer()
        self._swarm.on("fleet_changed", lambda _: self._on_fleet_changed())
        self._swarm.on("drone_error",   lambda d: self._on_drone_error(d))

    # ── UI Build ──────────────────────────────────────────────────

    def _build_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        root = QVBoxLayout(central)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(0)

        root.addWidget(self._build_topbar())
        root.addWidget(self._build_body(), 1)
        root.addWidget(self._build_logbar())

    def _build_topbar(self):
        tb = QWidget()
        tb.setFixedHeight(44)
        tb.setStyleSheet("background:#111318;border-bottom:1px solid #21262f;")
        h = QHBoxLayout(tb)
        h.setContentsMargins(12, 0, 12, 0)
        h.setSpacing(16)

        # Logo
        logo = QLabel("DRSKHUB")
        logo.setStyleSheet("font-size:17px;font-weight:800;letter-spacing:3px;color:#e2e8f0;")
        h.addWidget(logo)

        dot = QFrame()
        dot.setFixedSize(8, 8)
        dot.setStyleSheet("background:#3b82f6;border-radius:4px;")
        h.addWidget(dot)

        sep = lambda: self._vsep()

        h.addWidget(sep())

        # Stats
        self._tb_total   = self._topbar_stat("FLEET",   "0",  "#e2e8f0")
        self._tb_online  = self._topbar_stat("ONLINE",  "0",  "#22c55e")
        self._tb_armed   = self._topbar_stat("ARMED",   "0",  "#eab308")
        self._tb_mission = self._topbar_stat("MISSION", "0",  "#3b82f6")
        for w in [self._tb_total, sep(), self._tb_online, sep(),
                  self._tb_armed, sep(), self._tb_mission, sep()]:
            h.addWidget(w)

        # Mode indicator
        self._mode_pill = QLabel("● STANDBY")
        self._mode_pill.setStyleSheet("font-size:11px;font-weight:700;color:#475569;"
                                       "background:#1a1f28;border:1px solid #2d3340;"
                                       "border-radius:10px;padding:3px 10px;")
        h.addWidget(self._mode_pill)

        h.addStretch()

        # Right buttons
        self._sim_btn = QPushButton("⟳ Start Simulation")
        self._sim_btn.setStyleSheet("font-weight:600;padding:5px 14px;font-size:12px;")
        self._sim_btn.clicked.connect(self._toggle_simulation)
        h.addWidget(self._sim_btn)

        connect_btn = QPushButton("⊕ Connect Drone")
        connect_btn.setStyleSheet("font-weight:600;padding:5px 14px;font-size:12px;")
        connect_btn.clicked.connect(self._open_connect_dialog)
        h.addWidget(connect_btn)

        rtl_btn = QPushButton("⚠ RTL ALL")
        rtl_btn.setStyleSheet("background:rgba(239,68,68,.12);border:1px solid #ef4444;"
                               "color:#ef4444;font-weight:700;padding:5px 14px;font-size:12px;")
        rtl_btn.clicked.connect(self._swarm.rtl_all)
        h.addWidget(rtl_btn)

        arm_btn = QPushButton("▶ ARM ALL")
        arm_btn.setStyleSheet("background:#3b82f6;border-color:#3b82f6;color:#fff;"
                               "font-weight:700;padding:5px 14px;font-size:12px;")
        arm_btn.clicked.connect(self._swarm.arm_all)
        h.addWidget(arm_btn)

        return tb

    def _topbar_stat(self, label, val, color):
        w = QWidget()
        h = QHBoxLayout(w)
        h.setContentsMargins(0, 0, 0, 0)
        h.setSpacing(5)
        lbl = QLabel(label)
        lbl.setStyleSheet("font-size:9px;font-weight:700;letter-spacing:1px;color:#475569;")
        val_lbl = QLabel(val)
        val_lbl.setStyleSheet(f"font-size:16px;font-weight:700;color:{color};"
                               "font-family:Consolas,monospace;")
        setattr(self, f"_tbv_{label.lower()}", val_lbl)
        h.addWidget(lbl)
        h.addWidget(val_lbl)
        return w

    def _vsep(self):
        sep = QFrame()
        sep.setFixedWidth(1)
        sep.setFixedHeight(22)
        sep.setStyleSheet("background:#21262f;")
        return sep

    def _build_body(self):
        splitter = QSplitter(Qt.Orientation.Horizontal)
        splitter.setHandleWidth(1)
        splitter.setStyleSheet("QSplitter::handle{background:#21262f;}")

        # Fleet panel
        self._fleet = FleetPanel()
        self._fleet.drone_selected.connect(self._on_drone_selected)
        self._fleet.command_all.connect(self._on_command_all)
        splitter.addWidget(self._fleet)

        # Center: tabs (Map / Mission / Params)
        center = QWidget()
        cv = QVBoxLayout(center)
        cv.setContentsMargins(0, 0, 0, 0)
        cv.setSpacing(0)

        self._center_tabs = QTabWidget()
        self._center_tabs.setTabPosition(QTabWidget.TabPosition.North)
        self._map    = MapPanel()
        self._map.map_clicked.connect(self._on_map_click)
        self._mission = MissionPanel()
        self._mission.mission_changed.connect(self._on_mission_changed)
        self._mission.upload_requested.connect(self._on_mission_action)
        self._params = ParamsPanel()
        self._params.param_write.connect(self._on_param_write)
        self._center_tabs.addTab(self._map,     "🗺  Map")
        self._center_tabs.addTab(self._mission, "📍  Mission")
        self._center_tabs.addTab(self._params,  "⚙  Parameters")
        cv.addWidget(self._center_tabs, 1)
        splitter.addWidget(center)

        # Telemetry panel
        self._telem = TelemetryPanel()
        self._telem.command_issued.connect(self._on_command)
        splitter.addWidget(self._telem)

        splitter.setSizes([240, 980, 320])
        return splitter

    def _build_logbar(self):
        bar = QWidget()
        bar.setFixedHeight(26)
        bar.setStyleSheet("background:#060709;border-top:1px solid #21262f;")
        h = QHBoxLayout(bar)
        h.setContentsMargins(8, 0, 8, 0)
        h.setSpacing(16)
        tag = QLabel("LOG ›")
        tag.setStyleSheet("font-size:10px;color:#2d3340;font-family:Consolas,monospace;")
        h.addWidget(tag)
        self._log_bar_lbl = QLabel("DRSKHUB ready.")
        self._log_bar_lbl.setStyleSheet("font-size:11px;color:#475569;font-family:Consolas,monospace;")
        h.addWidget(self._log_bar_lbl)
        h.addStretch()
        return bar

    # ── Timer ─────────────────────────────────────────────────────

    def _setup_timer(self):
        self._timer = QTimer()
        self._timer.timeout.connect(self._tick)
        self._timer.start(200)  # 5 Hz UI refresh

    def _tick(self):
        # Update fleet list
        self._fleet.refresh_all(self._swarm)

        # Update topbar stats
        s = self._swarm.stats()
        for attr, key in [("fleet","total"),("online","online"),
                           ("armed","armed"),("mission","mission")]:
            try:
                getattr(self, f"_tbv_{attr}").setText(str(s[key]))
            except Exception:
                pass

        # Update selected drone telemetry
        if self._swarm.drones:
            idx = min(self._selected_idx, len(self._swarm.drones)-1)
            drone = self._swarm.drones[idx]
            self._telem.refresh(drone)

        # Update map markers
        for i, drone in enumerate(self._swarm.drones):
            color = DRONE_COLORS_LIST[i % len(DRONE_COLORS_LIST)]
            t = drone.telem
            self._map.update_drone(
                drone.id, t.lat, t.lon, t.heading,
                color, drone.name, t.battery_pct,
                t.alt_rel, selected=(i == self._selected_idx)
            )

    # ── Event handlers ────────────────────────────────────────────

    def _on_drone_selected(self, idx: int):
        self._selected_idx = idx
        if idx < len(self._swarm.drones):
            drone = self._swarm.drones[idx]
            self._telem.set_drone(drone)
            self._params.set_drone(drone)
            self._mission.set_drone_list([d.name for d in self._swarm.drones])

    def _on_command(self, cmd: str, data):
        if not self._swarm.drones:
            return
        idx = min(self._selected_idx, len(self._swarm.drones)-1)
        drone = self._swarm.drones[idx]
        self._dispatch_cmd(drone, cmd, data)

    def _dispatch_cmd(self, drone: Drone, cmd: str, data):
        cmds = {
            "arm":           lambda: drone.arm(),
            "disarm":        lambda: drone.disarm(),
            "takeoff":       lambda: drone.takeoff(data or 20.0),
            "land":          lambda: drone.land(),
            "rtl":           lambda: drone.rtl(),
            "loiter":        lambda: drone.set_mode("LOITER"),
            "set_mode":      lambda: drone.set_mode(data),
            "goto":          lambda: drone.goto(*data),
            "mission_start": lambda: drone.start_mission(),
            "mission_pause": lambda: drone.set_mode("LOITER"),
            "mission_resume":lambda: drone.start_mission(),
            "mission_abort": lambda: drone.set_mode("LOITER"),
        }
        fn = cmds.get(cmd)
        if fn:
            threading.Thread(target=fn, daemon=True).start()
            self._log(f"{drone.name}: {cmd.upper()}", "info")

    def _on_command_all(self, cmd: str):
        fn = {
            "arm_all":    self._swarm.arm_all,
            "disarm_all": self._swarm.disarm_all,
            "rtl_all":    self._swarm.rtl_all,
        }.get(cmd)
        if fn:
            threading.Thread(target=fn, daemon=True).start()
            self._log(f"SWARM: {cmd.upper()}", "warn")

    def _on_map_click(self, lat: float, lon: float):
        self._mission.set_coords_from_map(lat, lon)
        self._telem.set_goto_coords(lat, lon)
        self._log(f"Map click: {lat:.5f}, {lon:.5f}", "info")

    def _on_mission_changed(self, wps: list):
        self._map.set_waypoints(wps)

    def _on_mission_action(self, action: str):
        if action.startswith("formation:"):
            parts = action.split(":")
            formation = parts[1]
            spacing   = float(parts[2]) if len(parts)>2 else 25.0
            clat, clon = self._swarm.center_of_mass()
            if clat == 0: clat, clon = 41.0082, 28.9784
            self._swarm.apply_formation(formation, clat, clon, 50.0, spacing)
            self._log(f"Formation: {formation} applied to swarm", "ok")
            return

        wps = self._mission.get_waypoints()
        if action == "all":
            self._swarm.upload_mission_all(wps)
            self._log(f"Mission uploaded to all drones ({len(wps)} WPs)", "ok")
        elif action == "download":
            if self._swarm.drones:
                drone = self._swarm.drones[self._selected_idx]
                threading.Thread(target=drone.download_mission, daemon=True).start()
                self._log(f"Downloading mission from {drone.name}", "info")
        else:
            for d in self._swarm.drones:
                if d.name == action:
                    threading.Thread(target=d.upload_mission,args=(wps,),daemon=True).start()
                    self._log(f"Mission uploaded to {d.name} ({len(wps)} WPs)", "ok")

    def _on_param_write(self, name: str, val: float):
        if self._swarm.drones:
            drone = self._swarm.drones[self._selected_idx]
            drone.set_param(name, val)
            self._log(f"{drone.name}: SET {name}={val}", "info")

    def _on_fleet_changed(self):
        self._fleet.rebuild(self._swarm)
        self._mission.set_drone_list([d.name for d in self._swarm.drones])

    def _on_drone_error(self, data):
        drone, err = data
        self._log(f"ERROR {drone.name}: {err}", "error")

    # ── Simulation ────────────────────────────────────────────────

    def _toggle_simulation(self):
        if not self._sim_running:
            dlg = SimSetupDialog(self)
            if dlg.exec() == QDialog.DialogCode.Accepted:
                count = dlg.count_spin.value()
                self._start_simulation(count)
        else:
            self._stop_simulation()

    def _start_simulation(self, count: int):
        self._log(f"Starting simulation with {count} drones…", "info")
        conns = self._sim.start(count)
        self._sim_running = True
        self._sim_btn.setText("■ Stop Simulation")
        self._sim_btn.setStyleSheet("background:rgba(239,68,68,.12);border:1px solid #ef4444;"
                                     "color:#ef4444;font-weight:600;padding:5px 14px;font-size:12px;")
        self._mode_pill.setText("● SIMULATION")
        self._mode_pill.setStyleSheet("font-size:11px;font-weight:700;color:#a855f7;"
                                       "background:rgba(168,85,247,.1);border:1px solid #a855f7;"
                                       "border-radius:10px;padding:3px 10px;")

        # Connect to each sim drone (delayed slightly to allow sockets to open)
        def connect_all():
            time.sleep(0.5)
            for i, conn in enumerate(conns):
                d = Drone(
                    drone_id=conn["id"], name=conn["name"],
                    conn_type=ConnectionType.UDP,
                    host=conn["host"], port=conn["port"],
                )
                d.on("connected", lambda n: self._log(f"{n} connected", "ok"))
                d.on("error", lambda e: self._log(f"Sim error: {e}", "warn"))
                self._swarm.add_drone(d)
                d.connect()
                time.sleep(0.1)
        threading.Thread(target=connect_all, daemon=True).start()
        self._log(f"Simulation started. Connecting to {count} virtual drones…", "ok")

    def _stop_simulation(self):
        for d in list(self._swarm.drones):
            if d.id.startswith("SIM-"):
                self._swarm.remove_drone(d.id)
                self._map.remove_drone(d.id)
        self._sim.stop()
        self._sim_running = False
        self._sim_btn.setText("⟳ Start Simulation")
        self._sim_btn.setStyleSheet("font-weight:600;padding:5px 14px;font-size:12px;")
        self._mode_pill.setText("● STANDBY")
        self._mode_pill.setStyleSheet("font-size:11px;font-weight:700;color:#475569;"
                                       "background:#1a1f28;border:1px solid #2d3340;"
                                       "border-radius:10px;padding:3px 10px;")
        self._log("Simulation stopped.", "warn")

    # ── Connect dialog ────────────────────────────────────────────

    def _open_connect_dialog(self):
        dlg = ConnectDroneDialog(self)
        if dlg.exec() == QDialog.DialogCode.Accepted:
            p = dlg.get_params()
            ctype = ConnectionType.UDP if p["type"] == "udp" else ConnectionType.SERIAL
            uid = f"drone_{int(time.time()*1000) % 100000}"
            d = Drone(drone_id=uid, name=p["name"],
                      conn_type=ctype,
                      host=p["host"], port=p["port"],
                      serial_port=p["serial"], baud=p["baud"])
            d.on("connected", lambda n: self._log(f"{n} connected", "ok"))
            d.on("error",     lambda e: self._log(f"Connection error: {e}", "error"))
            self._swarm.add_drone(d)
            d.connect()
            self._log(f"Connecting to {p['name']} ({p['type'].upper()})…", "info")

    # ── Logging ───────────────────────────────────────────────────

    def _log(self, msg: str, level: str = "info"):
        import time
        ts = time.strftime("%H:%M:%S")
        full = f"[{ts}] {msg}"
        colors = {"info":"#94a3b8","ok":"#22c55e","warn":"#eab308","error":"#ef4444"}
        c = colors.get(level, "#94a3b8")
        self._log_bar_lbl.setText(full)
        self._log_bar_lbl.setStyleSheet(f"font-size:11px;color:{c};font-family:Consolas,monospace;")

    def closeEvent(self, event):
        self._sim.stop()
        for d in self._swarm.drones:
            d.disconnect()
        event.accept()