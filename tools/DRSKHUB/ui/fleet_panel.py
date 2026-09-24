from __future__ import annotations
from PyQt6.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QLabel,
                              QPushButton, QListWidget, QListWidgetItem,
                              QLineEdit, QFrame, QSizePolicy)
from PyQt6.QtCore import Qt, pyqtSignal, QSize
from PyQt6.QtGui import QColor
from .widgets import StatusDot, SignalWidget

DRONE_COLORS = [
    "#3b82f6","#22c55e","#a855f7","#f97316",
    "#eab308","#ef4444","#06b6d4","#ec4899",
    "#10b981","#f43f5e","#8b5cf6","#14b8a6",
]


class DroneListItem(QWidget):
    def __init__(self, drone, color, parent=None):
        super().__init__(parent)
        self._drone = drone
        self._color = color
        self.setFixedHeight(58)

        h = QHBoxLayout(self)
        h.setContentsMargins(12, 6, 12, 6)
        h.setSpacing(10)

        # Avatar
        avatar = QLabel("✈")
        avatar.setFixedSize(32, 32)
        avatar.setAlignment(Qt.AlignmentFlag.AlignCenter)
        avatar.setStyleSheet(f"font-size:18px;color:{color};"
                              f"background:rgba({self._hex_to_rgb(color)},0.15);"
                              f"border-radius:6px;")
        h.addWidget(avatar)

        # Info
        info = QVBoxLayout()
        info.setSpacing(2)
        self._name_lbl = QLabel(drone.name)
        self._name_lbl.setStyleSheet("font-size:12px;font-weight:700;color:#e2e8f0;")
        self._sub_lbl  = QLabel("Connecting...")
        self._sub_lbl.setStyleSheet("font-size:10px;color:#475569;font-family:Consolas,monospace;")
        info.addWidget(self._name_lbl)
        info.addWidget(self._sub_lbl)
        h.addLayout(info, 1)

        # Right side
        right = QVBoxLayout()
        right.setSpacing(4)
        right.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
        self._sig = SignalWidget()
        self._dot = StatusDot("disconnected")
        right.addWidget(self._sig)
        right.addWidget(self._dot)
        h.addLayout(right)

    def refresh(self, drone):
        t = drone.telem
        s = drone.status.value
        self._dot.set_status(s)
        self._sig.set_strength(drone.signal_strength())
        mode = t.mode if t.mode else "–"
        armed = "ARMED" if t.armed else ""
        self._sub_lbl.setText(
            f"{mode}  {t.alt_rel:.0f}m  {t.groundspeed:.1f}m/s  {armed}"
        )

    @staticmethod
    def _hex_to_rgb(hex_color):
        h = hex_color.lstrip("#")
        r, g, b = int(h[0:2],16), int(h[2:4],16), int(h[4:6],16)
        return f"{r},{g},{b}"


class FleetPanel(QWidget):
    drone_selected = pyqtSignal(int)   # index in swarm.drones
    command_all    = pyqtSignal(str)   # "arm_all","disarm_all","rtl_all"

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumWidth(220)
        self.setMaximumWidth(260)
        self._items: list[DroneListItem] = []
        self._selected = 0
        self._build_ui()

    def _build_ui(self):
        v = QVBoxLayout(self)
        v.setContentsMargins(0, 0, 0, 0)
        v.setSpacing(0)

        # Header
        hdr = QWidget()
        hdr.setStyleSheet("background:#111318;border-bottom:1px solid #21262f;")
        hdr.setFixedHeight(40)
        hl = QHBoxLayout(hdr)
        hl.setContentsMargins(12, 6, 12, 6)
        fleet_lbl = QLabel("FLEET")
        fleet_lbl.setStyleSheet("font-size:10px;font-weight:700;letter-spacing:1.5px;color:#475569;")
        self._count_lbl = QLabel("0 units")
        self._count_lbl.setStyleSheet("font-size:10px;color:#475569;font-family:Consolas,monospace;")
        hl.addWidget(fleet_lbl)
        hl.addStretch()
        hl.addWidget(self._count_lbl)
        v.addWidget(hdr)

        # Stats bar
        stats = QWidget()
        stats.setStyleSheet("background:#0f1116;border-bottom:1px solid #21262f;")
        stats.setFixedHeight(32)
        sl = QHBoxLayout(stats)
        sl.setContentsMargins(8, 0, 8, 0)
        sl.setSpacing(0)
        self._stat_online  = self._stat_pill("ONLINE",  "0", "#22c55e")
        self._stat_armed   = self._stat_pill("ARMED",   "0", "#eab308")
        self._stat_mission = self._stat_pill("MISSION", "0", "#3b82f6")
        sl.addWidget(self._stat_online)
        sl.addWidget(self._stat_armed)
        sl.addWidget(self._stat_mission)
        v.addWidget(stats)

        # Search
        self._search = QLineEdit()
        self._search.setPlaceholderText("Search drone…")
        self._search.setStyleSheet("margin:6px;background:#181c22;border:1px solid #21262f;"
                                    "border-radius:6px;padding:4px 8px;color:#e2e8f0;font-size:12px;")
        self._search.textChanged.connect(self._filter)
        v.addWidget(self._search)

        # List
        self._list = QListWidget()
        self._list.setStyleSheet("background:transparent;border:none;outline:none;")
        self._list.setSpacing(0)
        self._list.currentRowChanged.connect(self._on_select)
        v.addWidget(self._list, 1)

        # Footer buttons
        footer = QWidget()
        footer.setStyleSheet("background:#111318;border-top:1px solid #21262f;")
        footer.setFixedHeight(40)
        fl = QHBoxLayout(footer)
        fl.setContentsMargins(8, 4, 8, 4)
        fl.setSpacing(6)
        for label, cmd, style in [
            ("ARM ALL",  "arm_all",  "background:rgba(234,179,8,.1);border:1px solid #eab308;color:#eab308;font-size:10px;font-weight:700;"),
            ("RTL ALL",  "rtl_all",  "background:rgba(239,68,68,.1);border:1px solid #ef4444;color:#ef4444;font-size:10px;font-weight:700;"),
            ("DISARM",   "disarm_all","font-size:10px;font-weight:700;"),
        ]:
            b = QPushButton(label)
            b.setStyleSheet(style + "padding:2px 6px;border-radius:5px;")
            b.setFixedHeight(26)
            b.clicked.connect(lambda checked, c=cmd: self.command_all.emit(c))
            fl.addWidget(b)
        v.addWidget(footer)

    def _stat_pill(self, label, val, color):
        w = QWidget()
        h = QHBoxLayout(w)
        h.setContentsMargins(6, 0, 6, 0)
        h.setSpacing(3)
        lbl = QLabel(label)
        lbl.setStyleSheet("font-size:9px;color:#475569;font-weight:600;letter-spacing:.5px;")
        val_lbl = QLabel(val)
        val_lbl.setStyleSheet(f"font-size:12px;font-weight:700;color:{color};font-family:Consolas,monospace;")
        setattr(self, f"_sv_{label.lower()}", val_lbl)
        h.addWidget(lbl)
        h.addWidget(val_lbl)
        return w

    def rebuild(self, swarm):
        self._list.clear()
        self._items.clear()
        for i, drone in enumerate(swarm.drones):
            color = DRONE_COLORS[i % len(DRONE_COLORS)]
            item_widget = DroneListItem(drone, color)
            item = QListWidgetItem()
            item.setSizeHint(QSize(0, 58))
            self._list.addItem(item)
            self._list.setItemWidget(item, item_widget)
            self._items.append(item_widget)
            # Hover/select styling
            item_widget.setStyleSheet("border-left:2px solid transparent;")
        self._count_lbl.setText(f"{len(swarm.drones)} units")
        if swarm.drones:
            self._list.setCurrentRow(self._selected)
            self._update_selection()

    def refresh_all(self, swarm):
        stats = swarm.stats()
        try: self._sv_online.setText(str(stats["online"]))
        except: pass
        try: self._sv_armed.setText(str(stats["armed"]))
        except: pass
        try: self._sv_mission.setText(str(stats["mission"]))
        except: pass
        for i, (item_widget, drone) in enumerate(zip(self._items, swarm.drones)):
            item_widget.refresh(drone)
        self._update_selection()

    def _on_select(self, row):
        if row < 0:
            return
        self._selected = row
        self._update_selection()
        self.drone_selected.emit(row)

    def _update_selection(self):
        for i, w in enumerate(self._items):
            if i == self._selected:
                w.setStyleSheet("border-left:2px solid #3b82f6;"
                                 "background:rgba(59,130,246,0.07);")
            else:
                w.setStyleSheet("border-left:2px solid transparent;"
                                 "background:transparent;")

    def _filter(self, text):
        for i in range(self._list.count()):
            item = self._list.item(i)
            w = self._list.itemWidget(item)
            visible = text.lower() in w._drone.name.lower() if text else True
            item.setHidden(not visible)