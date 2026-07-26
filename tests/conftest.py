import sys
from pathlib import Path

# Make src/ importable the same way the app does (main.py runs from src/).
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
