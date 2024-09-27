import sys

import matplotlib.pyplot as plt
import qdarktheme
from PyQt6.QtWidgets import QApplication

from bezier.cubic_bezier import *
from bezier.quadratic_bezier import *
from gui.gui_manager import *
from utilities import *

if __name__ == "__main__":
    app = QApplication(sys.argv)

    qdarktheme.setup_theme()
    plt.style.use("dark_background")

    if getattr(sys, "frozen", False):
        create_files()
        load_fonts()

    window = AutonomousPlannerGUIManager()
    window.show()

    sys.exit(app.exec())
