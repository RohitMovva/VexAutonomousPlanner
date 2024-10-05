# -*- mode: python ; coding: utf-8 -*-

import os
import sys

def get_system_fonts():
    font_paths = []
    font_dirs = [
        "/usr/share/fonts",  # Common font directories on Linux
        "/usr/local/share/fonts",
        os.path.join(os.getenv("WINDIR", "C:\\Windows"), "Fonts")  # Windows fonts
    ]
    for font_dir in font_dirs:
        for root, dirs, files in os.walk(font_dir):
            for file in files:
                if file.lower().endswith(('.ttf', '.otf')):
                    font_paths.append(os.path.join(root, file))
    return font_paths

# Determine the icon path based on the operating system
icon_path = ''
if sys.platform == "win32":
    icon_path = 'assets/flip_logo.ico'  # Windows icon
else:
    icon_path = 'assets/linux_icon.png'  # Linux icon

a = Analysis(
    ['src/main.py'],
    pathex=[],
    binaries=[],
    datas=[
        ('assets', 'assets'),
        ('config.yaml', '.'),
        ('README.md', '.'),
        ('LICENSE', '.')
    ] + [(font, 'fonts') for font in get_system_fonts()],
    hiddenimports=[
    ],
    hookspath=[],
    hooksconfig={},
    runtime_hooks=[],
    excludes=[],
    noarchive=False,
    optimize=0,
)

pyz = PYZ(a.pure)

exe = EXE(
    pyz,
    a.scripts,
    [],
    exclude_binaries=True,
    name='path_planner',
    debug=False,
    bootloader_ignore_signals=False,
    strip=False,
    upx=True,
    console=False,  # Set to False to hide the console window
    disable_windowed_traceback=False,
    argv_emulation=False,
    target_arch=None,
    codesign_identity=None,
    entitlements_file=None,
    icon=icon_path,  # Use the icon path based on the OS
)

coll = COLLECT(
    exe,
    a.binaries,
    a.datas,
    strip=False,
    upx=True,
    upx_exclude=[],
    name='PathPlanner',
)
