from __future__ import annotations
from PyQt6.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QLabel,
                              QPushButton, QTableWidget, QTableWidgetItem,
                              QComboBox, QDoubleSpinBox, QGroupBox, QGridLayout,
                              QScrollArea, QFrame, QMessageBox)
from PyQt6.QtCore import Qt, pyqtSignal
from PyQt6.QtGui import QColor
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from core.drone import Waypoint


MAV_CMDS = {
    "TAKEOFF":     22,
    "WAYPOINT":    16,
    "LOITER_UNLIM":17,
    "LOITER_TIME": 19,
    "LOITER_TURNS":18,
    "RETURN_HOME": 20,
    "RTL":         20,
    "LAND":        21,
    "DO_CHANGE_SPEED": 178,
    "DO_SET_CAM_TRIGG_DIST": 206,
    "CONDITION_DELAY": 112,
}


class MissionPanel(QWidget):
    mission_changed = pyqtSignal(list)
    upload_requested = pyqtSignal(str)    # target drone id or "all"

    def __init__(self, parent=None):
        super().__init__(parent)
        self._waypoints = []
        self._drone_list = []
        self._build_ui()

    def _build_ui(self):
        v = QVBoxLayout(self)
        v.setContentsMargins(0, 0, 0, 0)
        v.setSpacing(0)

        # Toolbar
        tb = QWidget()
        tb.setStyleSheet("background:#111318;border-bottom:1px solid #21262f;")
        tb.setFixedHeight(40)
        tbl = QHBoxLayout(tb)
        tbl.setContentsMargins(8, 4, 8, 4)
        tbl.setSpacing(8)

        self._target_combo = QComboBox()
        self._target_combo.setFixedWidth(160)
        self._target_combo.addItem("All Drones (Swarm)")
        tbl.addWidget(QLabel("Target:"))
        tbl.addWidget(self._target_combo)
        tbl.addStretch()

        for label, cb, style in [
            ("Upload",   self._upload,   "background:#3b82f6;border-color:#3b82f6;color:#fff;font-weight:600;"),
            ("Download", self._download, ""),
            ("Clear",    self._clear,    "background:rgba(239,68,68,.1);border:1px solid #ef4444;color:#ef4444;"),
        ]:
            b = QPushButton(label)
            b.setStyleSheet(style + "padding:4px 12px;")
            b.clicked.connect(cb)
            tbl.addWidget(b)
        v.addWidget(tb)

        # Main area split
        main = QHBoxLayout()
        main.setSpacing(0)

        # WP Table
        left = QVBoxLayout()
        left.setContentsMargins(8, 8, 8, 8)
        left.setSpacing(6)

        self._table = QTableWidget(0, 6)
        self._table.setHorizontalHeaderLabels(["#","Type","Lat","Lon","Alt (m)",""])
        self._table.setColumnWidth(0, 30); self._table.setColumnWidth(1, 120)
        self._table.setColumnWidth(2, 100); self._table.setColumnWidth(3, 100)
        self._table.setColumnWidth(4, 70); self._table.setColumnWidth(5, 30)
        self._table.setStyleSheet("font-size:12px;")
        self._table.verticalHeader().setVisible(False)
        self._table.setSelectionBehavior(QTableWidget.SelectionBehavior.SelectRows)
        self._table.setEditTriggers(QTableWidget.EditTrigger.NoEditTriggers)
        left.addWidget(self._table, 1)

        # Add WP form
        form_grp = QGroupBox("Add Waypoint")
        form_grp.setStyleSheet("font-size:10px;font-weight:700;color:#475569;")
        fg = QGridLayout(form_grp)
        fg.setSpacing(6)

        self._wp_type = QComboBox()
        self._wp_type.addItems(list(MAV_CMDS.keys()))
        self._wp_lat = QDoubleSpinBox(); self._wp_lat.setRange(-90, 90); self._wp_lat.setDecimals(6); self._wp_lat.setValue(41.0082)
        self._wp_lon = QDoubleSpinBox(); self._wp_lon.setRange(-180,180); self._wp_lon.setDecimals(6); self._wp_lon.setValue(28.9784)
        self._wp_alt = QDoubleSpinBox(); self._wp_alt.setRange(0, 1000); self._wp_alt.setValue(50)
        fg.addWidget(QLabel("Type"), 0,0); fg.addWidget(self._wp_type, 0,1,1,3)
        fg.addWidget(QLabel("Lat"),  1,0); fg.addWidget(self._wp_lat,  1,1)
        fg.addWidget(QLabel("Lon"),  1,2); fg.addWidget(self._wp_lon,  1,3)
        fg.addWidget(QLabel("Alt"),  2,0); fg.addWidget(self._wp_alt,  2,1)

        add_btn = QPushButton("+ Add Waypoint")
        add_btn.setStyleSheet("background:#3b82f6;border-color:#3b82f6;color:#fff;font-weight:700;padding:7px;")
        add_btn.clicked.connect(self._add_wp)
        fg.addWidget(add_btn, 2,2,1,2)
        left.addWidget(form_grp)
        main.addLayout(left, 1)

        # Right: mission settings
        right = QWidget()
        right.setFixedWidth(200)
        right.setStyleSheet("background:#0f1116;border-left:1px solid #21262f;")
        rv = QVBoxLayout(right)
        rv.setContentsMargins(12, 12, 12, 12)
        rv.setSpacing(8)
        rv.addWidget(QLabel("Mission Settings"))

        for label, attr, default in [
            ("Speed (m/s)",    "_ms_speed", 10.0),
            ("Loiter R (m)",   "_ms_loitr", 50.0),
            ("RTL Alt (m)",    "_ms_rtlalt",100.0),
            ("Takeoff Alt (m)","_ms_tkalt",  30.0),
        ]:
            lbl = QLabel(label)
            lbl.setStyleSheet("font-size:10px;color:#475569;")
            rv.addWidget(lbl)
            spin = QDoubleSpinBox()
            spin.setRange(0, 5000)
            spin.setValue(default)
            spin.setStyleSheet("font-size:12px;")
            setattr(self, attr, spin)
            rv.addWidget(spin)

        rv.addStretch()

        # Swarm formation
        rv.addWidget(self._sep())
        rv.addWidget(QLabel("Swarm Formation"))
        self._formation_combo = QComboBox()
        self._formation_combo.addItems(["Line","Grid","Circle","V-Shape"])
        rv.addWidget(self._formation_combo)
        self._spacing_spin = QDoubleSpinBox(); self._spacing_spin.setRange(5,200); self._spacing_spin.setValue(25)
        rv.addWidget(QLabel("Spacing (m)"))
        rv.addWidget(self._spacing_spin)
        form_btn = QPushButton("Apply Formation")
        form_btn.setStyleSheet("background:rgba(59,130,246,.1);border:1px solid #3b82f6;color:#3b82f6;font-weight:600;padding:6px;")
        form_btn.clicked.connect(self._apply_formation)
        rv.addWidget(form_btn)

        self._formation_result = None
        main.addWidget(right)

        wrapper = QWidget()
        wrapper.setLayout(main)
        v.addWidget(wrapper, 1)

    def _sep(self):
        sep = QFrame()
        sep.setFrameShape(QFrame.Shape.HLine)
        sep.setStyleSheet("background:#21262f;max-height:1px;border:none;")
        return sep

    # ── Public API ────────────────────────────────────────────────

    def set_drone_list(self, names: list):
        self._drone_list = names
        cur = self._target_combo.currentText()
        self._target_combo.clear()
        self._target_combo.addItem("All Drones (Swarm)")
        for n in names:
            self._target_combo.addItem(n)

    def set_coords_from_map(self, lat: float, lon: float):
        self._wp_lat.setValue(lat)
        self._wp_lon.setValue(lon)

    def get_waypoints(self) -> list:
        return self._waypoints

    # ── Slots ─────────────────────────────────────────────────────

    def _add_wp(self):
        t = self._wp_type.currentText()
        lat = self._wp_lat.value()
        lon = self._wp_lon.value()
        alt = self._wp_alt.value()
        cmd = MAV_CMDS.get(t, 16)
        wp = Waypoint(index=len(self._waypoints), command=cmd,
                      lat=lat, lon=lon, alt=alt)
        wp.type_name = t
        self._waypoints.append(wp)
        self._refresh_table()
        self.mission_changed.emit(self._wp_to_dicts())

    def _refresh_table(self):
        self._table.setRowCount(0)
        for i, wp in enumerate(self._waypoints):
            self._table.insertRow(i)
            self._table.setItem(i, 0, QTableWidgetItem(str(i+1)))
            self._table.setItem(i, 1, QTableWidgetItem(getattr(wp, "type_name", str(wp.command))))
            self._table.setItem(i, 2, QTableWidgetItem(f"{wp.lat:.6f}"))
            self._table.setItem(i, 3, QTableWidgetItem(f"{wp.lon:.6f}"))
            self._table.setItem(i, 4, QTableWidgetItem(f"{wp.alt:.1f}"))
            del_btn = QPushButton("✕")
            del_btn.setStyleSheet("color:#ef4444;border:none;background:transparent;font-size:13px;")
            del_btn.clicked.connect(lambda checked, idx=i: self._del_wp(idx))
            self._table.setCellWidget(i, 5, del_btn)
        self._table.resizeRowsToContents()

    def _del_wp(self, idx):
        if 0 <= idx < len(self._waypoints):
            self._waypoints.pop(idx)
            for i, wp in enumerate(self._waypoints):
                wp.index = i
            self._refresh_table()
            self.mission_changed.emit(self._wp_to_dicts())

    def _clear(self):
        self._waypoints.clear()
        self._refresh_table()
        self.mission_changed.emit([])

    def _upload(self):
        if not self._waypoints:
            QMessageBox.warning(self, "No Waypoints", "Add waypoints first.")
            return
        target = self._target_combo.currentText()
        tid = "all" if target.startswith("All") else target
        self.upload_requested.emit(tid)

    def _download(self):
        self.upload_requested.emit("download")

    def _apply_formation(self):
        f = self._formation_combo.currentText().lower().replace("-","")
        s = self._spacing_spin.value()
        # Emit as special command
        self.upload_requested.emit(f"formation:{f}:{s}")

    def _wp_to_dicts(self):
        return [{"lat": wp.lat, "lon": wp.lon, "alt": wp.alt,
                 "type": getattr(wp, "type_name", str(wp.command))}
                for wp in self._waypoints]