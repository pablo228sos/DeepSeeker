"""
widgets.py — переиспользуемые виджеты: карточки телеметрии,
             искусственный горизонт, индикатор сигнала, компас.
"""
from __future__ import annotations
import math
from PyQt6.QtWidgets import (QWidget, QLabel, QVBoxLayout, QHBoxLayout,
                              QFrame, QProgressBar, QSizePolicy)
from PyQt6.QtCore import Qt, QTimer, QRectF, QPointF
from PyQt6.QtGui import (QPainter, QColor, QPen, QBrush, QFont,
                          QLinearGradient, QPainterPath, QConicalGradient)


# ── Utility ──────────────────────────────────────────────────────

def make_label(text, cls=None, size=None, bold=False, color=None):
    lbl = QLabel(text)
    if cls:
        lbl.setProperty('class', cls)
    if size:
        f = lbl.font(); f.setPointSize(size); lbl.setFont(f)
    if bold:
        f = lbl.font(); f.setBold(True); lbl.setFont(f)
    if color:
        lbl.setStyleSheet(f'color: {color};')
    return lbl


# ── Telemetry Card ────────────────────────────────────────────────

class TelemetryCard(QFrame):
    def __init__(self, label: str, value: str = '--', unit: str = '',
                 color: str = '#e2e8f0', parent=None):
        super().__init__(parent)
        self.setProperty('class', 'card')
        self.setFixedHeight(68)

        v = QVBoxLayout(self)
        v.setContentsMargins(10, 8, 10, 8)
        v.setSpacing(2)

        self._lbl = QLabel(label.upper())
        self._lbl.setStyleSheet('font-size:9px;font-weight:700;'
                                'letter-spacing:1px;color:#475569;')
        v.addWidget(self._lbl)

        row = QHBoxLayout()
        self._val = QLabel(value)
        self._val.setStyleSheet(f'font-family: "JetBrains Mono","Consolas",monospace;'
                                f'font-size:20px;font-weight:700;color:{color};')
        self._unit = QLabel(unit)
        self._unit.setStyleSheet('font-size:11px;color:#475569;margin-top:6px;')
        row.addWidget(self._val)
        row.addWidget(self._unit)
        row.addStretch()
        v.addLayout(row)

    def update_value(self, val: str, color: str = None):
        self._val.setText(val)
        if color:
            self._val.setStyleSheet(f'font-family:"JetBrains Mono","Consolas",monospace;'
                                    f'font-size:20px;font-weight:700;color:{color};')


# ── Battery Bar ───────────────────────────────────────────────────

class BatteryWidget(QFrame):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setProperty('class', 'card')
        self.setFixedHeight(72)
        v = QVBoxLayout(self)
        v.setContentsMargins(10, 8, 10, 8)
        v.setSpacing(4)

        top = QHBoxLayout()
        self._lbl = QLabel('BATTERY')
        self._lbl.setStyleSheet('font-size:9px;font-weight:700;'
                                'letter-spacing:1px;color:#475569;')
        self._pct = QLabel('-- %')
        self._pct.setStyleSheet('font-family:"JetBrains Mono","Consolas",monospace;'
                                'font-size:18px;font-weight:700;color:#22c55e;')
        self._volt = QLabel('-- V')
        self._volt.setStyleSheet('font-size:11px;color:#475569;margin-top:4px;')
        top.addWidget(self._lbl)
        top.addStretch()
        top.addWidget(self._volt)
        v.addLayout(top)
        v.addWidget(self._pct)

        self._bar = QProgressBar()
        self._bar.setRange(0, 100)
        self._bar.setValue(0)
        self._bar.setTextVisible(False)
        self._bar.setFixedHeight(4)
        v.addWidget(self._bar)

    def update(self, pct: int, volt: float):
        self._pct.setText(f'{pct} %')
        self._volt.setText(f'{volt:.1f} V')
        self._bar.setValue(pct)
        if pct > 50:
            c = '#22c55e'; bc = 'chunk{background:#22c55e;border-radius:2px;}'
        elif pct > 20:
            c = '#eab308'; bc = 'chunk{background:#eab308;border-radius:2px;}'
        else:
            c = '#ef4444'; bc = 'chunk{background:#ef4444;border-radius:2px;}'
        self._pct.setStyleSheet(f'font-family:"JetBrains Mono","Consolas",monospace;'
                                f'font-size:18px;font-weight:700;color:{c};')
        self._bar.setStyleSheet(f'QProgressBar{{background:#21262f;border:none;border-radius:2px;}}'
                                f'QProgressBar::{bc}')


# ── Artificial Horizon (ADI) ──────────────────────────────────────

class AttitudeIndicator(QWidget):
    def __init__(self, size=160, parent=None):
        super().__init__(parent)
        self._sz   = size
        self._roll  = 0.0
        self._pitch = 0.0
        self.setFixedSize(size, size)

    def set_attitude(self, roll: float, pitch: float):
        self._roll  = roll
        self._pitch = pitch
        self.update()

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        cx = self.width() / 2
        cy = self.height() / 2
        r  = min(cx, cy) - 4

        # Clip to circle
        path = QPainterPath()
        path.addEllipse(QPointF(cx, cy), r, r)
        p.setClipPath(path)

        p.save()
        p.translate(cx, cy)
        p.rotate(self._roll)
        pitch_px = self._pitch * (r / 45.0)

        # Sky
        p.fillRect(-r*2, -r*2, r*4, int(r*2 + pitch_px), QColor('#1e3a6e'))
        # Ground
        p.fillRect(-r*2, int(pitch_px), r*4, r*4, QColor('#5c3a1e'))

        # Horizon line
        pen = QPen(QColor('#ffffff'), 1.5)
        p.setPen(pen)
        p.drawLine(int(-r), int(pitch_px), int(r), int(pitch_px))

        # Pitch ladder
        pen2 = QPen(QColor('#ffffff88'), 1)
        p.setPen(pen2)
        for deg in range(-40, 45, 10):
            if deg == 0: continue
            yp = int(pitch_px - deg * (r / 45.0))
            w  = r // 3 if deg % 20 == 0 else r // 5
            p.drawLine(-w, yp, w, yp)
        p.restore()

        # Remove clip, draw overlay
        p.setClipping(False)

        # Border
        pen3 = QPen(QColor('#2d3340'), 2)
        p.setPen(pen3)
        p.drawEllipse(QPointF(cx, cy), r, r)

        # Roll arc
        pen4 = QPen(QColor('#3b82f6'), 1.5)
        p.setPen(pen4)
        p.drawArc(int(cx-r), int(cy-r), int(r*2), int(r*2), 60*16, 60*16)
        p.drawArc(int(cx-r), int(cy-r), int(r*2), int(r*2), (180-60)*16, 60*16)

        # Aircraft symbol
        pen5 = QPen(QColor('#facc15'), 2.5)
        p.setPen(pen5)
        p.drawLine(int(cx-r*0.4), int(cy), int(cx-r*0.15), int(cy))
        p.drawLine(int(cx+r*0.15), int(cy), int(cx+r*0.4), int(cy))
        p.drawLine(int(cx), int(cy-r*0.12), int(cx), int(cy+r*0.12))
        p.drawPoint(int(cx), int(cy))


# ── Compass ───────────────────────────────────────────────────────

class CompassWidget(QWidget):
    def __init__(self, size=120, parent=None):
        super().__init__(parent)
        self._hdg = 0.0
        self.setFixedSize(size, size)

    def set_heading(self, hdg: float):
        self._hdg = hdg
        self.update()

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        cx = self.width() / 2
        cy = self.height() / 2
        r  = min(cx, cy) - 4

        # Background
        p.setBrush(QBrush(QColor('#111318')))
        p.setPen(QPen(QColor('#2d3340'), 1.5))
        p.drawEllipse(QPointF(cx, cy), r, r)

        # Cardinal marks
        labels = {0:'N', 90:'E', 180:'S', 270:'W'}
        for deg, txt in labels.items():
            angle = math.radians(deg - self._hdg - 90)
            lx = cx + (r - 12) * math.cos(angle)
            ly = cy + (r - 12) * math.sin(angle)
            p.setPen(QPen(QColor('#3b82f6' if txt == 'N' else '#94a3b8'), 1))
            f = QFont()
            f.setPointSize(7)
            f.setBold(True)
            p.setFont(f)
            p.drawText(QPointF(lx - 4, ly + 4), txt)

        # Tick marks
        for deg in range(0, 360, 10):
            angle = math.radians(deg - self._hdg - 90)
            inner = r - (8 if deg % 30 == 0 else 4)
            x1 = cx + inner * math.cos(angle)
            y1 = cy + inner * math.sin(angle)
            x2 = cx + (r-1) * math.cos(angle)
            y2 = cy + (r-1) * math.sin(angle)
            p.setPen(QPen(QColor('#475569'), 1))
            p.drawLine(QPointF(x1, y1), QPointF(x2, y2))

        # Needle (always points up = current heading)
        p.save()
        p.translate(cx, cy)
        needle = QPainterPath()
        needle.moveTo(0, -r*0.6)
        needle.lineTo(4, 0)
        needle.lineTo(0, r*0.3)
        needle.lineTo(-4, 0)
        needle.closeSubpath()
        p.setBrush(QBrush(QColor('#3b82f6')))
        p.setPen(Qt.PenStyle.NoPen)
        p.drawPath(needle)
        p.restore()

        # Center dot
        p.setBrush(QBrush(QColor('#21262f')))
        p.setPen(QPen(QColor('#3b82f6'), 1))
        p.drawEllipse(QPointF(cx, cy), 4, 4)

        # Heading text
        p.setPen(QPen(QColor('#e2e8f0'), 1))
        f2 = QFont()
        f2.setFamily('JetBrains Mono')
        f2.setPointSize(8)
        f2.setBold(True)
        p.setFont(f2)
        p.drawText(QRectF(cx-20, cy+r-18, 40, 16),
                   Qt.AlignmentFlag.AlignCenter, f'{int(self._hdg):03d}°')


# ── Signal bars ───────────────────────────────────────────────────

class SignalWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self._strength = 0  # 0-4
        self.setFixedSize(24, 16)

    def set_strength(self, s: int):
        self._strength = max(0, min(4, s))
        self.update()

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        colors = ['#22c55e', '#22c55e', '#eab308', '#ef4444']
        c = colors[min(self._strength, 3)] if self._strength > 0 else '#475569'
        w, gap = 4, 2
        heights = [4, 7, 10, 14]
        for i in range(4):
            x = i * (w + gap)
            h = heights[i]
            y = 16 - h
            col = c if i < self._strength else '#21262f'
            p.fillRect(x, y, w, h, QColor(col))


# ── Status dot ────────────────────────────────────────────────────

class StatusDot(QWidget):
    COLORS = {
        'connected': '#22c55e',
        'armed':     '#eab308',
        'mission':   '#3b82f6',
        'error':     '#ef4444',
        'connecting':'#a855f7',
        'disconnected': '#475569',
    }

    def __init__(self, status='disconnected', parent=None):
        super().__init__(parent)
        self._status = status
        self.setFixedSize(10, 10)

    def set_status(self, s: str):
        self._status = s
        self.update()

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        c = QColor(self.COLORS.get(self._status, '#475569'))
        p.setBrush(QBrush(c))
        p.setPen(Qt.PenStyle.NoPen)
        p.drawEllipse(1, 1, 8, 8)
