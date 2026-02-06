from enum import auto, Enum
from pykit.logger import Logger
from wpilib import Alert
from typing import Final, Callable
from constants import Constants
from subsystems import Subsystem
from subsystems.turret.io import TurretIO
from math import *
from wpimath.geometry import Pose2d, Rotation2d
from wpilib import DriverStation

class TurretSubsystem(Subsystem):

    class Goal(Enum):
        NONE = auto()
        HUB = auto()
        DEPOT = auto()
        OUTPOST = auto()

    # On the reference there's something about a CANrange here and I don't know what that means so I'm leaving it.
    
    def __init__(self, io: TurretIO, robot_pose_supplier: Callable[[], Pose2d]) -> None:
        super().__init__() # Change PID controller and Initial position if needed

        self._io: Final[TurretIO] = io
        self._inputs = TurretIO.TurretIOInputs()
        self.robot_pose_supplier = robot_pose_supplier

        self.turret_disconnected_alert = Alert("Turret motor is disconnected.", Alert.AlertType.kError)

        self.independent_rotation = Rotation2d(0)
        self.current_radians = 0.0

        self.goal = self.Goal.NONE

    def periodic(self):

        # Update inputs from hardware/simulation
        self._io.update_inputs(self._inputs)

        # Log inputs to PyKit
        Logger.processInputs("Turret", self._inputs)

        # Update alerts
        self.turret_disconnected_alert.set(not self._inputs.turret_connected)

        # TODO will be implemented later for calculating the current position of the turret independent of the robot's rotation
        self.current_radians = self.robot_pose_supplier().rotation().radians() + self.independent_rotation.radians()

        if self.goal != self.Goal.NONE:
            self.rotate_to_goal(self.goal)
        
    def get_radians_to_goal(self):
        # If the robot position is in the alliance side, call get_radians_to_goal before aiming
        # If the robot is in the neutral zone, have it determine what side of the zone it's on so it knows the target to aim at
        match self.goal:
            case self.Goal.HUB:
                xdist = abs(self.robot_pose_supplier().X() - Constants.GoalLocations.BLUE_HUB.X()) if DriverStation.getAlliance == DriverStation.Alliance.kBlue else abs(self.robot_pose_supplier().X() - Constants.GoalLocations.RED_HUB.X())
                ydist = abs(self.robot_pose_supplier().Y() - Constants.GoalLocations.BLUE_HUB.Y()) if DriverStation.getAlliance == DriverStation.Alliance.kBlue else abs(self.robot_pose_supplier().Y() - Constants.GoalLocations.RED_HUB.Y())
            case self.Goal.OUTPOST:
                xdist = abs(self.robot_pose_supplier().X() - Constants.GoalLocations.BLUE_OUTPOST_PASS.X()) if DriverStation.getAlliance == DriverStation.Alliance.kBlue else abs(self.robot_pose_supplier().X() - Constants.GoalLocations.RED_OUTPOST_PASS.X())
                ydist = abs(self.robot_pose_supplier().Y() - Constants.GoalLocations.BLUE_OUTPOST_PASS.Y()) if DriverStation.getAlliance == DriverStation.Alliance.kBlue else abs(self.robot_pose_supplier().Y() - Constants.GoalLocations.RED_OUTPOST_PASS.Y())
            case self.Goal.DEPOT:
                xdist = abs(self.robot_pose_supplier().X() - Constants.GoalLocations.BLUE_DEPOT_PASS.X()) if DriverStation.getAlliance == DriverStation.Alliance.kBlue else abs(self.robot_pose_supplier().X() - Constants.GoalLocations.RED_DEPOT_PASS.X())
                ydist = abs(self.robot_pose_supplier().Y() - Constants.GoalLocations.BLUE_DEPOT_PASS.Y()) if DriverStation.getAlliance == DriverStation.Alliance.kBlue else abs(self.robot_pose_supplier().Y() - Constants.GoalLocations.RED_DEPOT_PASS.Y())
            case _:
                print("No turret goal set, returning 0.0")
                return 0.0
        return atan(ydist / xdist)

    def rotate_to_goal(self, target: Goal):
        # This function might not work because it probably isn't periodic so it'll only set the output once and then not check if the angle is correct until it's called again (which is when the target changes)
        self.goal = target
        target_radians = self.get_radians_to_goal()
        self._io.set_position(target_radians)