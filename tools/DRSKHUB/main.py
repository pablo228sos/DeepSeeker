#!/usr/bin/env python3
"""DRSKHUB - Swarm Ground Control Station"""
import sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from PyQt6.QtWidgets import QApplication
from PyQt6.QtGui import QFont
from ui.main_window import MainWindow
from ui.styles import STYLESHEET

def main():
    app = QApplication(sys.argv)
    app.setApplicationName("DRSKHUB")
    app.setStyleSheet(STYLESHEET)
    font = QFont("Segoe UI", 10)
    app.setFont(font)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())

if __name__ == "__main__":
    main()
