import sys
import ctypes
import time

import matplotlib.pyplot as plt
import qdarktheme
from PyQt6.QtWidgets import QApplication

import utilities
from gui import gui_manager
import sys
import io

# Set stdout and stderr to use UTF-8
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8')
sys.stderr = io.TextIOWrapper(sys.stderr.buffer, encoding='utf-8')

if __name__ == "__main__":
    if sys.platform.startswith('win'):
        myappid = 'Flip.PathPlanner.0.2.0'
        ctypes.windll.shell32.SetCurrentProcessExplicitAppUserModelID(myappid)
    app = QApplication(sys.argv)

    qdarktheme.setup_theme()
    plt.style.use("dark_background")

    if getattr(sys, "frozen", False):
        utilities.create_files()
        utilities.load_fonts()

    window = gui_manager.AutonomousPlannerGUIManager()
    window.show()

    sys.exit(app.exec())
