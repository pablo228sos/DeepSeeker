"""Единая тема DRSKHUB — минималистичный тёмный интерфейс."""

STYLESHEET = """
QWidget {
    background-color: #0a0b0d;
    color: #e2e8f0;
    font-family: 'Inter', 'Segoe UI', 'Helvetica Neue', sans-serif;
    font-size: 13px;
}
QMainWindow {
    background-color: #0a0b0d;
}

/* ── Scrollbars ── */
QScrollBar:vertical {
    background: transparent;
    width: 6px;
    margin: 0;
}
QScrollBar::handle:vertical {
    background: #2d3340;
    border-radius: 3px;
    min-height: 20px;
}
QScrollBar::handle:vertical:hover { background: #3b82f6; }
QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical { height: 0; }

QScrollBar:horizontal {
    background: transparent;
    height: 6px;
}
QScrollBar::handle:horizontal {
    background: #2d3340;
    border-radius: 3px;
}

/* ── Buttons ── */
QPushButton {
    background-color: #181c22;
    color: #94a3b8;
    border: 1px solid #21262f;
    border-radius: 6px;
    padding: 6px 14px;
    font-weight: 500;
}
QPushButton:hover {
    background-color: rgba(59,130,246,0.08);
    border-color: #3b82f6;
    color: #e2e8f0;
}
QPushButton:pressed {
    background-color: rgba(59,130,246,0.15);
}
QPushButton[class="primary"] {
    background-color: #3b82f6;
    border-color: #3b82f6;
    color: #ffffff;
    font-weight: 600;
}
QPushButton[class="primary"]:hover {
    background-color: #2563eb;
    border-color: #2563eb;
}
QPushButton[class="danger"] {
    background-color: rgba(239,68,68,0.12);
    border-color: #ef4444;
    color: #ef4444;
}
QPushButton[class="danger"]:hover {
    background-color: rgba(239,68,68,0.22);
}
QPushButton[class="success"] {
    background-color: rgba(34,197,94,0.12);
    border-color: #22c55e;
    color: #22c55e;
}
QPushButton[class="warning"] {
    background-color: rgba(234,179,8,0.12);
    border-color: #eab308;
    color: #eab308;
}
QPushButton:disabled {
    background-color: #111318;
    color: #475569;
    border-color: #1a1f28;
}

/* ── LineEdit / ComboBox ── */
QLineEdit, QSpinBox, QDoubleSpinBox {
    background-color: #181c22;
    border: 1px solid #21262f;
    border-radius: 6px;
    padding: 5px 8px;
    color: #e2e8f0;
    selection-background-color: #3b82f6;
}
QLineEdit:focus, QSpinBox:focus, QDoubleSpinBox:focus {
    border-color: #3b82f6;
}
QComboBox {
    background-color: #181c22;
    border: 1px solid #21262f;
    border-radius: 6px;
    padding: 5px 8px;
    color: #e2e8f0;
}
QComboBox:focus { border-color: #3b82f6; }
QComboBox::drop-down { border: none; width: 20px; }
QComboBox QAbstractItemView {
    background-color: #111318;
    border: 1px solid #21262f;
    color: #e2e8f0;
    selection-background-color: #3b82f6;
}

/* ── Labels ── */
QLabel[class="section-title"] {
    font-size: 10px;
    font-weight: 700;
    letter-spacing: 1px;
    color: #475569;
    text-transform: uppercase;
}
QLabel[class="value-big"] {
    font-family: 'JetBrains Mono', 'Consolas', monospace;
    font-size: 22px;
    font-weight: 700;
    color: #e2e8f0;
}
QLabel[class="value-mono"] {
    font-family: 'JetBrains Mono', 'Consolas', monospace;
    font-size: 13px;
    color: #e2e8f0;
}
QLabel[class="unit"] {
    font-size: 11px;
    color: #475569;
}
QLabel[class="green"] { color: #22c55e; }
QLabel[class="yellow"] { color: #eab308; }
QLabel[class="red"]   { color: #ef4444; }
QLabel[class="blue"]  { color: #3b82f6; }
QLabel[class="purple"] { color: #a855f7; }

/* ── Tab Widget ── */
QTabWidget::pane {
    border: none;
    background: #0a0b0d;
}
QTabBar::tab {
    background: transparent;
    color: #94a3b8;
    padding: 8px 16px;
    border-bottom: 2px solid transparent;
    font-weight: 500;
    font-size: 12px;
}
QTabBar::tab:selected {
    color: #3b82f6;
    border-bottom: 2px solid #3b82f6;
}
QTabBar::tab:hover:!selected {
    color: #e2e8f0;
}

/* ── List Widget ── */
QListWidget {
    background-color: transparent;
    border: none;
    outline: none;
}
QListWidget::item {
    padding: 0;
    border: none;
    background: transparent;
}
QListWidget::item:selected {
    background: transparent;
}

/* ── Table ── */
QTableWidget {
    background-color: transparent;
    border: none;
    gridline-color: #21262f;
    outline: none;
}
QTableWidget::item {
    padding: 4px 8px;
    border-bottom: 1px solid #181c22;
}
QTableWidget::item:selected {
    background-color: rgba(59,130,246,0.12);
    color: #e2e8f0;
}
QHeaderView::section {
    background-color: #111318;
    color: #475569;
    font-size: 10px;
    font-weight: 700;
    letter-spacing: 1px;
    text-transform: uppercase;
    padding: 6px 8px;
    border: none;
    border-bottom: 1px solid #21262f;
}

/* ── Splitter ── */
QSplitter::handle {
    background-color: #21262f;
    width: 1px;
    height: 1px;
}

/* ── Tooltip ── */
QToolTip {
    background-color: #111318;
    color: #e2e8f0;
    border: 1px solid #21262f;
    border-radius: 4px;
    padding: 4px 8px;
    font-size: 11px;
}

/* ── Progress Bar ── */
QProgressBar {
    background-color: #181c22;
    border: 1px solid #21262f;
    border-radius: 4px;
    height: 6px;
    text-align: center;
}
QProgressBar::chunk {
    border-radius: 3px;
    background-color: #3b82f6;
}

/* ── Group Box ── */
QGroupBox {
    border: 1px solid #21262f;
    border-radius: 8px;
    margin-top: 8px;
    padding-top: 8px;
    font-size: 10px;
    font-weight: 700;
    color: #475569;
    letter-spacing: 1px;
}
QGroupBox::title {
    subcontrol-origin: margin;
    left: 8px;
    padding: 0 4px;
}

/* ── Slider ── */
QSlider::groove:horizontal {
    background: #21262f;
    height: 4px;
    border-radius: 2px;
}
QSlider::handle:horizontal {
    background: #3b82f6;
    width: 12px;
    height: 12px;
    margin: -4px 0;
    border-radius: 6px;
}
QSlider::sub-page:horizontal {
    background: #3b82f6;
    border-radius: 2px;
}

/* ── CheckBox ── */
QCheckBox::indicator {
    width: 14px;
    height: 14px;
    border: 1px solid #21262f;
    border-radius: 3px;
    background: #181c22;
}
QCheckBox::indicator:checked {
    background: #3b82f6;
    border-color: #3b82f6;
}

/* ── Frame / Card ── */
QFrame[class="card"] {
    background-color: #111318;
    border: 1px solid #21262f;
    border-radius: 8px;
}
QFrame[class="card-accent"] {
    background-color: #111318;
    border: 1px solid #3b82f6;
    border-radius: 8px;
}
QFrame[class="separator"] {
    background-color: #21262f;
    max-height: 1px;
}
"""

# Color palette
C = {
    'bg':       '#0a0b0d',
    'surface':  '#111318',
    'surface2': '#181c22',
    'border':   '#21262f',
    'border2':  '#2d3340',
    'accent':   '#3b82f6',
    'green':    '#22c55e',
    'yellow':   '#eab308',
    'red':      '#ef4444',
    'orange':   '#f97316',
    'purple':   '#a855f7',
    'text':     '#e2e8f0',
    'text2':    '#94a3b8',
    'text3':    '#475569',
}
