from __future__ import annotations
from PyQt6.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QLabel,
                              QTabWidget, QPushButton, QGridLayout, QGroupBox,
                              QDoubleSpinBox, QComboBox, QScrollArea, QFrame, QSizePolicy)
from PyQt6.QtCore import Qt, pyqtSignal, QTimer
from PyQt6.QtGui import QFont
from .widgets import (AttitudeIndicator, CompassWidget, BatteryWidget,
                       TelemetryCard, SignalWidget, StatusDot)


class TelemetryPanel(QWidget):
    command_issued = pyqtSignal(str, object)   # (cmd_name, data)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumWidth(280)
        self.setMaximumWidth(340)
        self._drone = None
        self._build_ui()

    def _build_ui(self):
        v = QVBoxLayout(self)
        v.setContentsMargins(0, 0, 0, 0)
        v.setSpacing(0)

        # Header
        hdr = QWidget()
        hdr.setStyleSheet("background:#111318;border-bottom:1px solid #21262f;")
        hdr.setFixedHeight(72)
        hl = QVBoxLayout(hdr)
        hl.setContentsMargins(12, 8, 12, 8)
        hl.setSpacing(4)

        top_row = QHBoxLayout()
        self._dot = StatusDot("disconnected")
        self._name_lbl = QLabel("No drone selected")
        self._name_lbl.setStyleSheet("font-size:15px;font-weight:700;color:#e2e8f0;")
        self._signal = SignalWidget()
        top_row.addWidget(self._dot)
        top_row.addWidget(self._name_lbl)
        top_row.addStretch()
        top_row.addWidget(self._signal)
        hl.addLayout(top_row)

        self._sub_lbl = QLabel("–")
        self._sub_lbl.setStyleSheet("font-size:11px;color:#475569;font-family:Consolas,monospace;")
        hl.addWidget(self._sub_lbl)
        v.addWidget(hdr)

        # Mode + RSSI bar
        mode_bar = QWidget()
        mode_bar.setStyleSheet("background:#0f1116;border-bottom:1px solid #21262f;")
        mode_bar.setFixedHeight(32)
        mb = QHBoxLayout(mode_bar)
        mb.setContentsMargins(12, 4, 12, 4)
        self._mode_lbl = QLabel("DISARMED")
        self._mode_lbl.setStyleSheet("font-size:12px;font-weight:700;color:#475569;"
                                      "background:#1a1f28;border:1px solid #2d3340;"
                                      "border-radius:5px;padding:1px 8px;")
        self._rssi_lbl = QLabel("RSSI: --")
        self._rssi_lbl.setStyleSheet("font-size:11px;color:#475569;font-family:Consolas,monospace;")
        mb.addWidget(self._mode_lbl)
        mb.addStretch()
        mb.addWidget(self._rssi_lbl)
        v.addWidget(mode_bar)

        # Tabs
        self._tabs = QTabWidget()
        self._tabs.addTab(self._build_data_tab(), "Data")
        self._tabs.addTab(self._build_attitude_tab(), "Attitude")
        self._tabs.addTab(self._build_cmd_tab(), "Commands")
        v.addWidget(self._tabs, 1)

    # ── Data Tab ─────────────────────────────────────────────────

    def _build_data_tab(self):
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.Shape.NoFrame)
        w = QWidget()
        v = QVBoxLayout(w)
        v.setContentsMargins(8, 8, 8, 8)
        v.setSpacing(6)

        def sec(title):
            lbl = QLabel(title)
            lbl.setStyleSheet("font-size:9px;font-weight:700;letter-spacing:1px;"
                               "color:#475569;border-bottom:1px solid #21262f;"
                               "padding-bottom:4px;margin-top:4px;")
            return lbl

        v.addWidget(sec("POSITION"))
        g1 = QGridLayout(); g1.setSpacing(6)
        self._c_alt  = TelemetryCard("Altitude",   "--", "m",  "#3b82f6")
        self._c_spd  = TelemetryCard("Gnd Speed",  "--", "m/s","#e2e8f0")
        self._c_hdg  = TelemetryCard("Heading",    "--", "°",  "#e2e8f0")
        self._c_dwp  = TelemetryCard("Dist to WP", "--", "m",  "#e2e8f0")
        g1.addWidget(self._c_alt, 0,0); g1.addWidget(self._c_spd, 0,1)
        g1.addWidget(self._c_hdg, 1,0); g1.addWidget(self._c_dwp, 1,1)
        v.addLayout(g1)

        self._gps_lbl = QLabel("GPS: --")
        self._gps_lbl.setStyleSheet("font-size:11px;color:#94a3b8;font-family:Consolas,monospace;"
                                     "background:#111318;border:1px solid #21262f;border-radius:6px;"
                                     "padding:4px 8px;")
        v.addWidget(self._gps_lbl)

        v.addWidget(sec("POWER"))
        self._bat_widget = BatteryWidget()
        v.addWidget(self._bat_widget)

        g2 = QGridLayout(); g2.setSpacing(6)
        self._c_volt = TelemetryCard("Voltage", "--", "V", "#e2e8f0")
        self._c_curr = TelemetryCard("Current", "--", "A", "#e2e8f0")
        g2.addWidget(self._c_volt, 0,0); g2.addWidget(self._c_curr, 0,1)
        v.addLayout(g2)

        v.addWidget(sec("FLIGHT"))
        g3 = QGridLayout(); g3.setSpacing(6)
        self._c_time = TelemetryCard("Flight Time", "--:--", "", "#e2e8f0")
        self._c_wp   = TelemetryCard("Waypoint",    "--",    "",  "#e2e8f0")
        self._c_sats = TelemetryCard("Satellites",  "--",    "",  "#22c55e")
        self._c_hdop = TelemetryCard("HDOP",        "--",    "",  "#e2e8f0")
        self._c_clmb = TelemetryCard("Climb",       "--",    "m/s","#e2e8f0")
        self._c_air  = TelemetryCard("Airspeed",    "--",    "m/s","#e2e8f0")
        g3.addWidget(self._c_time, 0,0); g3.addWidget(self._c_wp, 0,1)
        g3.addWidget(self._c_sats, 1,0); g3.addWidget(self._c_hdop,1,1)
        g3.addWidget(self._c_clmb, 2,0); g3.addWidget(self._c_air, 2,1)
        v.addLayout(g3)

        v.addStretch()
        scroll.setWidget(w)
        return scroll

    # ── Attitude Tab ─────────────────────────────────────────────

    def _build_attitude_tab(self):
        w = QWidget()
        v = QVBoxLayout(w)
        v.setContentsMargins(8, 8, 8, 8)
        v.setSpacing(8)
        v.setAlignment(Qt.AlignmentFlag.AlignTop)

        adi_row = QHBoxLayout()
        adi_row.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._adi = AttitudeIndicator(160)
        adi_row.addWidget(self._adi)
        self._compass = CompassWidget(120)
        adi_row.addWidget(self._compass)
        v.addLayout(adi_row)

        g = QGridLayout(); g.setSpacing(6)
        self._c_roll  = TelemetryCard("Roll",    "--", "°", "#e2e8f0")
        self._c_pitch = TelemetryCard("Pitch",   "--", "°", "#e2e8f0")
        self._c_yaw   = TelemetryCard("Yaw",     "--", "°", "#e2e8f0")
        self._c_vspd  = TelemetryCard("Vrt Spd", "--", "m/s","#e2e8f0")
        g.addWidget(self._c_roll, 0,0); g.addWidget(self._c_pitch,0,1)
        g.addWidget(self._c_yaw,  1,0); g.addWidget(self._c_vspd, 1,1)
        v.addLayout(g)

        self._vib_lbl = QLabel("Vibration X/Y/Z: --")
        self._vib_lbl.setStyleSheet("font-size:11px;color:#94a3b8;font-family:Consolas,monospace;"
                                     "background:#111318;border:1px solid #21262f;border-radius:6px;"
                                     "padding:4px 8px;")
        v.addWidget(self._vib_lbl)

        ekf_row = QHBoxLayout()
        self._ekf_lbl = QLabel("EKF: UNKNOWN")
        self._ekf_lbl.setStyleSheet("font-size:12px;font-weight:700;color:#475569;")
        self._armed_lbl = QLabel("DISARMED")
        self._armed_lbl.setStyleSheet("font-size:12px;font-weight:700;color:#475569;")
        ekf_row.addWidget(self._ekf_lbl)
        ekf_row.addStretch()
        ekf_row.addWidget(self._armed_lbl)
        v.addLayout(ekf_row)
        v.addStretch()
        return w

    # ── Commands Tab ─────────────────────────────────────────────

    def _build_cmd_tab(self):
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.Shape.NoFrame)
        w = QWidget()
        v = QVBoxLayout(w)
        v.setContentsMargins(8, 8, 8, 8)
        v.setSpacing(10)

        def sec(title):
            lbl = QLabel(title)
            lbl.setStyleSheet("font-size:9px;font-weight:700;letter-spacing:1px;"
                               "color:#475569;border-bottom:1px solid #21262f;padding-bottom:4px;")
            return lbl

        # ARM / DISARM
        v.addWidget(sec("ARMING"))
        arm_row = QHBoxLayout()
        arm_btn = QPushButton("ARM")
        arm_btn.setProperty("class","warning")
        arm_btn.setStyleSheet("background:rgba(234,179,8,.12);border:1px solid #eab308;"
                               "color:#eab308;font-weight:700;padding:8px;")
        arm_btn.clicked.connect(lambda: self.command_issued.emit("arm", None))
        dis_btn = QPushButton("DISARM")
        dis_btn.setStyleSheet("background:rgba(239,68,68,.1);border:1px solid #ef4444;"
                               "color:#ef4444;font-weight:700;padding:8px;")
        dis_btn.clicked.connect(lambda: self.command_issued.emit("disarm", None))
        arm_row.addWidget(arm_btn); arm_row.addWidget(dis_btn)
        v.addLayout(arm_row)

        # Takeoff / Land / RTL
        v.addWidget(sec("QUICK ACTIONS"))
        q_grid = QGridLayout(); q_grid.setSpacing(6)
        for i, (label, cmd, style) in enumerate([
            ("Takeoff", "takeoff", "background:rgba(34,197,94,.12);border:1px solid #22c55e;color:#22c55e;font-weight:600;padding:7px;"),
            ("Land",    "land",    "font-weight:600;padding:7px;"),
            ("RTL",     "rtl",     "background:rgba(234,179,8,.1);border:1px solid #eab308;color:#eab308;font-weight:600;padding:7px;"),
            ("Loiter",  "loiter",  "font-weight:600;padding:7px;"),
        ]):
            b = QPushButton(label)
            b.setStyleSheet(style)
            b.clicked.connect(lambda checked, c=cmd: self.command_issued.emit(c, None))
            q_grid.addWidget(b, i//2, i%2)
        v.addLayout(q_grid)

        # Mode
        v.addWidget(sec("FLIGHT MODE"))
        modes = ["STABILIZE","ALT_HOLD","LOITER","AUTO","GUIDED","RTL","LAND","CIRCLE","POSHOLD","BRAKE"]
        self._mode_combo = QComboBox()
        self._mode_combo.addItems(modes)
        set_mode_btn = QPushButton("Set Mode")
        set_mode_btn.setStyleSheet("background:#3b82f6;border-color:#3b82f6;color:#fff;font-weight:600;padding:6px;")
        set_mode_btn.clicked.connect(lambda: self.command_issued.emit("set_mode", self._mode_combo.currentText()))
        v.addWidget(self._mode_combo)
        v.addWidget(set_mode_btn)

        # Goto
        v.addWidget(sec("GUIDED GOTO"))
        goto_grid = QGridLayout(); goto_grid.setSpacing(4)
        self._goto_lat = QDoubleSpinBox(); self._goto_lat.setRange(-90,90); self._goto_lat.setDecimals(6); self._goto_lat.setValue(41.0082)
        self._goto_lon = QDoubleSpinBox(); self._goto_lon.setRange(-180,180); self._goto_lon.setDecimals(6); self._goto_lon.setValue(28.9784)
        self._goto_alt = QDoubleSpinBox(); self._goto_alt.setRange(0,500); self._goto_alt.setValue(50)
        goto_grid.addWidget(QLabel("Lat"), 0,0); goto_grid.addWidget(self._goto_lat, 0,1)
        goto_grid.addWidget(QLabel("Lon"), 1,0); goto_grid.addWidget(self._goto_lon, 1,1)
        goto_grid.addWidget(QLabel("Alt"), 2,0); goto_grid.addWidget(self._goto_alt, 2,1)
        v.addLayout(goto_grid)
        goto_btn = QPushButton("Go To Position")
        goto_btn.setStyleSheet("background:#3b82f6;border-color:#3b82f6;color:#fff;font-weight:600;padding:7px;")
        goto_btn.clicked.connect(lambda: self.command_issued.emit("goto", (self._goto_lat.value(), self._goto_lon.value(), self._goto_alt.value())))
        v.addWidget(goto_btn)

        # Mission
        v.addWidget(sec("MISSION CONTROL"))
        m_grid = QGridLayout(); m_grid.setSpacing(6)
        for i, (label, cmd) in enumerate([
            ("Start Mission","mission_start"),("Pause","mission_pause"),
            ("Resume","mission_resume"),("Abort","mission_abort"),
        ]):
            b = QPushButton(label)
            b.setStyleSheet("font-weight:600;padding:7px;")
            b.clicked.connect(lambda checked, c=cmd: self.command_issued.emit(c, None))
            m_grid.addWidget(b, i//2, i%2)
        v.addLayout(m_grid)
        v.addStretch()
        scroll.setWidget(w)
        return scroll

    # ── Public API ────────────────────────────────────────────────

    def set_drone(self, drone):
        self._drone = drone

    def set_goto_coords(self, lat: float, lon: float):
        self._goto_lat.setValue(lat)
        self._goto_lon.setValue(lon)

    def refresh(self, drone):
        if drone is None:
            return
        t = drone.telem
        s = drone.status.value

        self._dot.set_status(s)
        self._signal.set_strength(drone.signal_strength())
        self._name_lbl.setText(drone.name)
        self._sub_lbl.setText(f"{s.upper()}  ·  {drone.conn_type.value.upper()}")

        mode_colors = {
            "AUTO":"color:#3b82f6;background:rgba(59,130,246,.12);border:1px solid #3b82f6;",
            "LOITER":"color:#a855f7;background:rgba(168,85,247,.1);border:1px solid #a855f7;",
            "RTL":"color:#eab308;background:rgba(234,179,8,.1);border:1px solid #eab308;",
            "LAND":"color:#f97316;background:rgba(249,115,22,.1);border:1px solid #f97316;",
            "STABILIZE":"color:#22c55e;background:rgba(34,197,94,.1);border:1px solid #22c55e;",
        }
        mc = mode_colors.get(t.mode, "color:#475569;background:#1a1f28;border:1px solid #2d3340;")
        self._mode_lbl.setStyleSheet(f"font-size:12px;font-weight:700;{mc};border-radius:5px;padding:1px 8px;")
        self._mode_lbl.setText(t.mode)
        self._rssi_lbl.setText(f"RSSI: {t.rssi} dBm")

        # Data
        self._c_alt.update_value(f"{t.alt_rel:.1f}", "#3b82f6" if t.alt_rel > 5 else "#e2e8f0")
        self._c_spd.update_value(f"{t.groundspeed:.1f}")
        self._c_hdg.update_value(f"{t.heading:.0f}")
        self._c_dwp.update_value(f"{t.wp_dist:.0f}")
        self._gps_lbl.setText(f"GPS  {t.lat:.6f}N  {t.lon:.6f}E   Sats:{t.satellites}  Fix:{t.fix_type}")
        self._bat_widget.update(t.battery_pct, t.voltage)
        self._c_volt.update_value(f"{t.voltage:.2f}")
        self._c_curr.update_value(f"{t.current:.1f}")
        self._c_time.update_value(drone.flight_time())
        self._c_wp.update_value(f"{t.wp_index}/{t.wp_count}")

        sat_col = "#22c55e" if t.satellites >= 6 else "#eab308" if t.satellites >= 4 else "#ef4444"
        self._c_sats.update_value(str(t.satellites), sat_col)
        self._c_hdop.update_value(f"{t.hdop:.1f}")
        self._c_clmb.update_value(f"{t.climb:.2f}", "#22c55e" if t.climb > 0 else "#ef4444")
        self._c_air.update_value(f"{t.airspeed:.1f}")

        # Attitude
        self._adi.set_attitude(t.roll, t.pitch)
        self._compass.set_heading(t.heading)
        self._c_roll.update_value(f"{t.roll:.1f}")
        self._c_pitch.update_value(f"{t.pitch:.1f}")
        self._c_yaw.update_value(f"{t.yaw:.0f}")
        self._c_vspd.update_value(f"{t.vz:.2f}")
        self._vib_lbl.setText(f"Vibration  X:{t.vib_x:.3f}  Y:{t.vib_y:.3f}  Z:{t.vib_z:.3f}")
        self._ekf_lbl.setText("EKF: OK" if t.ekf_ok else "EKF: --")
        self._ekf_lbl.setStyleSheet(f"font-size:12px;font-weight:700;"
                                     f"color:{'#22c55e' if t.ekf_ok else '#475569'};")
        armed_txt = "ARMED" if t.armed else "DISARMED"
        armed_col = "#eab308" if t.armed else "#475569"
        self._armed_lbl.setText(armed_txt)
        self._armed_lbl.setStyleSheet(f"font-size:12px;font-weight:700;color:{armed_col};")
