import ctypes
import io
import logging
import sys

import matplotlib.pyplot as plt
import qdarktheme
from PyQt6.QtWidgets import QApplication

import utilities.file_management
from gui import gui_manager
from utilities.logger_config import LogMode, setup_global_logger

# Set stdout and stderr to use UTF-8
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding="utf-8")
sys.stderr = io.TextIOWrapper(sys.stderr.buffer, encoding="utf-8")

if __name__ == "__main__":
    # Configure the global logger
    setup_global_logger(level=logging.INFO, mode=LogMode.FILE_ONLY)

    logger = logging.getLogger(__name__)
    logger.info("Application started")

    if sys.platform.startswith("win"):
        myappid = "Flip.PathPlanner.0.2.0"
        ctypes.windll.shell32.SetCurrentProcessExplicitAppUserModelID(myappid)
    app = QApplication(sys.argv)

    qdarktheme.setup_theme()
    plt.style.use("dark_background")

    if getattr(sys, "frozen", False):
        utilities.file_management.create_files()
        utilities.file_management.load_fonts()

    window = gui_manager.AutonomousPlannerGUIManager()
    window.show()

    sys.exit(app.exec())
