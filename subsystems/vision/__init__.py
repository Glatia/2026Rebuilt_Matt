import importlib.util
import sys
from pathlib import Path

from subsystems.vision.io import VisionIO, VisionIOTalonFX, VisionIOSim

# Import VisionSubsystem from hyphenated filename
_vision_subsystem_path = Path(__file__).parent / "vision-subsystem.py"
spec = importlib.util.spec_from_file_location("vision_subsystem", _vision_subsystem_path)
_vision_subsystem_module = importlib.util.module_from_spec(spec)
sys.modules["vision_subsystem"] = _vision_subsystem_module
spec.loader.exec_module(_vision_subsystem_module)
VisionSubsystem = _vision_subsystem_module.VisionSubsystem

__all__ = ["VisionIO", "VisionIOTalonFX", "VisionIOSim", "VisionSubsystem"]
