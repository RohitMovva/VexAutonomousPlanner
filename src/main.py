import sys
import ctypes
import time

import matplotlib.pyplot as plt
import qdarktheme
from PyQt6.QtWidgets import QApplication

import utilities
from gui import gui_manager

if __name__ == "__main__":
    myappid = 'Flip.PathPlanner.0.2.0' # arbitrary string
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
