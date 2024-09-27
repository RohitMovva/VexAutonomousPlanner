import sys

import matplotlib.pyplot as plt
import qdarktheme
from PyQt6.QtWidgets import QApplication

import utilities
from gui import gui_manager

if __name__ == "__main__":
    app = QApplication(sys.argv)

    qdarktheme.setup_theme()
    plt.style.use("dark_background")

    if getattr(sys, "frozen", False):
        utilities.create_files()
        utilities.load_fonts()

    window = gui_manager.AutonomousPlannerGUIManager()
    window.show()

    sys.exit(app.exec())
