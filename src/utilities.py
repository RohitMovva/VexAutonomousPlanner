import os
import sys
import yaml
from PyQt6.QtGui import QFontDatabase
import matplotlib.pyplot as plt

def load_fonts():
    font_dir = os.path.join(os.path.dirname(__file__), 'fonts')
    for font_file in os.listdir(font_dir):
        QFontDatabase.addApplicationFont(os.path.join(font_dir, font_file))

# File management stuff

def create_files():
    files = [
        'routes/',
    ]
    for file in files:
        if not os.path.exists(file):
            os.makedirs(file)
            print(f"Created file: {file}")

def resource_path(relative_path):
    try:
        base_path = sys._MEIPASS
        relative_path = relative_path[3:]
    except Exception:
        base_path = os.path.abspath(".")
    print(os.path.join(base_path, relative_path))
    return os.path.join(base_path, relative_path)

def getConfigValue(keyname):
    with open(resource_path('../config.yaml'), 'r') as file:
        config = yaml.safe_load(file)
    if (config == None):
        config = {}
    return config.get(keyname) # Prevents error if key doesn't exist in dict

def setConfigValue(keyname, value):
    with open(resource_path('../config.yaml'), 'r') as file:
        config = yaml.safe_load(file)
    if (config == None):
        config = {}
    config[keyname] = value
    with open(resource_path('../config.yaml'), 'w') as file:
                yaml.safe_dump(config, file)

def create_mpl_plot(xlist, ylist, width, height, title, xlabel, ylabel):
    plt.figure(figsize=(width, height))

    plt.subplot(3, 1, 1)
    plt.plot(xlist, ylist)
    plt.title(title)
    plt.xlabel(xlabel)
    plt.ylabel(ylabel)


    plt.tight_layout()
    plt.show()