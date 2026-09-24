from __future__ import annotations
from PyQt6.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QLabel,
                              QPushButton, QTableWidget, QTableWidgetItem,
                              QLineEdit, QComboBox, QMessageBox)
from PyQt6.QtCore import Qt, pyqtSignal
from PyQt6.QtGui import QColor


COMMON_PARAMS = [
    ("ARMING_CHECK",          "1",      "Pre-arm safety check bitmask"),
    ("FENCE_ENABLE",          "0",      "Enable geofence (0=disable,1=enable)"),
    ("FENCE_RADIUS",          "300",    "Circular fence radius (m)"),
    ("RTL_ALT",               "10000",  "RTL altitude in cm"),
    ("WPNAV_SPEED",           "1000",   "Waypoint nav speed cm/s"),
    ("WPNAV_ACCEL",           "250",    "Waypoint acceleration cm/s2"),
    ("WPNAV_RADIUS",          "200",    "Waypoint radius cm"),
    ("PILOT_SPEED_UP",        "250",    "Max climb rate cm/s"),
    ("PILOT_SPEED_DN",        "150",    "Max descent rate cm/s"),
    ("BATT_LOW_VOLT",         "22.0",   "Low voltage warning V"),
    ("BATT_CRT_VOLT",         "21.0",   "Critical voltage V"),
    ("BATT_FS_LOW_ACT",       "2",      "Low battery failsafe action"),
    ("FS_GCS_ENABLE",         "1",      "GCS heartbeat failsafe"),
    ("FS_THR_ENABLE",         "2",      "Throttle failsafe"),
    ("LOG_BITMASK",           "65535",  "Dataflash log bitmask"),
    ("SERIAL1_BAUD",          "57",     "Telemetry port baud x1000"),
    ("SR1_POSITION",          "3",      "Position stream rate Hz"),
    ("SR1_ATTITUDE",          "10",     "Attitude stream rate Hz"),
    ("SR1_EXTRA1",            "4",      "Extra1 stream rate Hz"),
    ("SR1_EXT_STAT",          "2",      "Extended status stream rate Hz"),
    ("MOT_SPIN_ARM",          "0.10",   "Motor spin when armed"),
    ("MOT_SPIN_MIN",          "0.15",   "Minimum motor spin"),
    ("INS_GYRO_FILTER",       "20",     "Gyro low-pass filter Hz"),
    ("ATC_RAT_RLL_P",         "0.135",  "Roll rate P gain"),
    ("ATC_RAT_PIT_P",         "0.135",  "Pitch rate P gain"),
    ("ATC_RAT_YAW_P",         "0.18",   "Yaw rate P gain"),
]


class ParamsPanel(QWidget):
    param_write = pyqtSignal(str, float)

    def __init__(self, parent=None):
        super().__init__(parent)
        self._drone = None
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

        self._search = QLineEdit()
        self._search.setPlaceholderText("Search parameter…")
        self._search.setFixedWidth(180)
        self._search.textChanged.connect(self._filter)
        tbl.addWidget(self._search)
        tbl.addStretch()

        for label, cb, style in [
            ("Refresh All",  self._refresh_params, ""),
            ("Write All",    self._write_all, "background:#3b82f6;border-color:#3b82f6;color:#fff;font-weight:600;"),
        ]:
            b = QPushButton(label)
            b.setStyleSheet(style + "padding:4px 12px;")
            b.clicked.connect(cb)
            tbl.addWidget(b)
        v.addWidget(tb)

        # Table
        self._table = QTableWidget(0, 3)
        self._table.setHorizontalHeaderLabels(["Parameter", "Value", "Description"])
        self._table.setColumnWidth(0, 200)
        self._table.setColumnWidth(1, 100)
        self._table.setColumnWidth(2, 350)
        self._table.verticalHeader().setVisible(False)
        self._table.setStyleSheet("font-size:12px;")
        self._table.setSelectionBehavior(QTableWidget.SelectionBehavior.SelectRows)
        v.addWidget(self._table, 1)
        self._populate_defaults()

    def _populate_defaults(self):
        self._table.setRowCount(0)
        for name, val, desc in COMMON_PARAMS:
            row = self._table.rowCount()
            self._table.insertRow(row)
            name_item = QTableWidgetItem(name)
            name_item.setForeground(QColor("#3b82f6"))
            name_item.setFlags(Qt.ItemFlag.ItemIsSelectable | Qt.ItemFlag.ItemIsEnabled)
            self._table.setItem(row, 0, name_item)
            val_item = QTableWidgetItem(val)
            self._table.setItem(row, 1, val_item)
            desc_item = QTableWidgetItem(desc)
            desc_item.setForeground(QColor("#475569"))
            desc_item.setFlags(Qt.ItemFlag.ItemIsSelectable | Qt.ItemFlag.ItemIsEnabled)
            self._table.setItem(row, 2, desc_item)

    def set_drone(self, drone):
        self._drone = drone
        if drone and drone.params:
            self._load_drone_params(drone.params)

    def _load_drone_params(self, params: dict):
        self._table.setRowCount(0)
        for name, val in sorted(params.items()):
            row = self._table.rowCount()
            self._table.insertRow(row)
            ni = QTableWidgetItem(name)
            ni.setForeground(QColor("#3b82f6"))
            ni.setFlags(Qt.ItemFlag.ItemIsSelectable | Qt.ItemFlag.ItemIsEnabled)
            self._table.setItem(row, 0, ni)
            vi = QTableWidgetItem(str(val))
            self._table.setItem(row, 1, vi)
            self._table.setItem(row, 2, QTableWidgetItem(""))

    def _filter(self, text):
        for row in range(self._table.rowCount()):
            item = self._table.item(row, 0)
            visible = text.upper() in item.text().upper() if text else True
            self._table.setRowHidden(row, not visible)

    def _refresh_params(self):
        if self._drone:
            self._drone.request_params()
            QMessageBox.information(self, "Parameters", "Requesting parameters from drone…")

    def _write_all(self):
        if not self._drone:
            QMessageBox.warning(self, "No Drone", "Select a drone first.")
            return
        count = 0
        for row in range(self._table.rowCount()):
            name = self._table.item(row, 0).text()
            val_item = self._table.item(row, 1)
            if val_item and val_item.column() == 1:
                try:
                    self._drone.set_param(name, float(val_item.text()))
                    count += 1
                except ValueError:
                    pass
        QMessageBox.information(self, "Parameters", f"Wrote {count} parameters to {self._drone.name}.")