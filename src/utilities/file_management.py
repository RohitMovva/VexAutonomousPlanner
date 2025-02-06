import logging
import os
import sys

import matplotlib.pyplot as plt
import yaml
from PyQt6.QtGui import QFontDatabase

logger = logging.getLogger(__name__)


def load_fonts():
    font_dir = os.path.join(os.path.dirname(__file__), "fonts")
    for font_file in os.listdir(font_dir):
        QFontDatabase.addApplicationFont(os.path.join(font_dir, font_file))


# File management stuff


def create_files():
    files = [
        "routes/",
    ]
    for file in files:
        if not os.path.exists(file):
            os.makedirs(file)
            logger.info(f"Created file: {file}")


def resource_path(relative_path):
    try:
        base_path = sys._MEIPASS
        relative_path = relative_path[3:]
    except Exception:
        base_path = os.path.abspath(".")
    logger.info(f"Full resource path {os.path.join(base_path, relative_path)}")
    return os.path.join(base_path, relative_path)


def get_config_value(keyname):
    with open(resource_path("../config.yaml"), "r") as file:
        config = yaml.safe_load(file)
    if config is None:
        config = {}
    return config.get(keyname)  # Prevents error if key doesn't exist in dict


def set_config_value(keyname, value):
    with open(resource_path("../config.yaml"), "r") as file:
        config = yaml.safe_load(file)
    if config is None:
        config = {}
    config[keyname] = value
    with open(resource_path("../config.yaml"), "w") as file:
        yaml.safe_dump(config, file)


def create_mpl_plot(
    xlist, ylist, width, height, title, xlabel, ylabel, xlim=None, ylim=None, numCols=3
):
    plt.figure(figsize=(width, height))

    plt.subplot(numCols, 1, 1)
    plt.plot(xlist, ylist)
    plt.title(title)
    plt.xlabel(xlabel)
    plt.ylabel(ylabel)

    if xlim is not None:
        plt.xlim(xlim[0], xlim[1])
    if ylim is not None:
        plt.ylim(ylim[0], ylim[1])

    plt.tight_layout()
    plt.show()
