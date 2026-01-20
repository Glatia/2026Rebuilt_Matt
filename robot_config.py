"""
Robot configuration detection and hardware mapping.

This module detects which robot is running and provides the appropriate hardware configuration.
"""

import socket
from enum import Enum, auto
from typing import Final

from wpilib import RobotController


class Robot(Enum):
    """Enumeration of available robots."""
    LARRY = auto()  # Test robot
    COMP = auto()   # Competition robot
    UNKNOWN = auto()  # Fallback if detection fails


def detect_robot() -> Robot:
    """
    Detect which robot we're running on.
    
    Detection methods (in order of preference):
    1. MAC address of RoboRIO (most reliable)
    2. Hostname (if set differently on each robot)
    3. Environment variable ROBOT_NAME
    
    :return: The detected robot
    """
    # Method 1: Check MAC address (RoboRIO MAC addresses are unique)
    try:
        mac_address = RobotController.getMACAddress()
        # Replace these with your actual MAC addresses
        # You can find MAC addresses via: ssh admin@roborio-XXXX-frc.local "cat /sys/class/net/eth0/address"
        LARRY_MAC_ADDRESSES = [
            "00:80:2f:XX:XX:XX",  # Replace with Larry's actual MAC
            # Add other possible MAC addresses for Larry if it has multiple interfaces
        ]
        COMP_MAC_ADDRESSES = [
            "00:80:2f:YY:YY:YY",  # Replace with Comp's actual MAC
            # Add other possible MAC addresses for Comp if it has multiple interfaces
        ]
        
        if mac_address in LARRY_MAC_ADDRESSES:
            return Robot.LARRY
        if mac_address in COMP_MAC_ADDRESSES:
            return Robot.COMP
    except Exception:
        pass
    
    # Method 2: Check hostname
    try:
        hostname = socket.gethostname().lower()
        if "larry" in hostname:
            return Robot.LARRY
        if "comp" in hostname or "competition" in hostname:
            return Robot.COMP
    except Exception:
        pass
    
    # Method 3: Check environment variable (useful for testing)
    import os
    robot_name = os.environ.get("ROBOT_NAME", "").upper()
    if robot_name == "LARRY":
        return Robot.LARRY
    if robot_name == "COMP":
        return Robot.COMP
    
    # Fallback: Default to COMP for competition, or set to LARRY for testing
    # Change this default based on your preference
    return Robot.COMP  # or Robot.LARRY if you want to default to test robot


# Detect robot at module load time
currentRobot: Final[Robot] = detect_robot()


def has_subsystem(subsystem_name: str) -> bool:
    """
    Check if a subsystem is available on the current robot.
    
    :param subsystem_name: Name of the subsystem (e.g., "climber", "intake")
    :return: True if the subsystem exists on the current robot, False otherwise
    """
    # Define which subsystems exist on each robot
    # Add/remove subsystems as needed
    LARRY_SUBSYSTEMS = {
        "drivetrain",  # Always present
        "vision",      # Always present
        # Add subsystems that Larry has:
        # "climber",
        # "intake",
    }
    
    COMP_SUBSYSTEMS = {
        "drivetrain",  # Always present
        "vision",      # Always present
        "climber",     # Competition robot has climber
        "intake",      # Competition robot has intake
        # Add other Comp subsystems as needed
    }
    
    if currentRobot == Robot.LARRY:
        return subsystem_name.lower() in LARRY_SUBSYSTEMS
    else:  # COMP or UNKNOWN defaults to COMP
        return subsystem_name.lower() in COMP_SUBSYSTEMS
