from enum import auto, Enum

from commands2 import Command, cmd, PIDSubsystem
from phoenix6 import utils
from phoenix6.configs import CANrangeConfiguration, TalonFXConfiguration, MotorOutputConfigs, FeedbackConfigs, HardwareLimitSwitchConfigs, ProximityParamsConfigs, CurrentLimitsConfigs
from phoenix6.controls import PositionVoltage
from phoenix6.hardware import TalonFX, CANrange
from phoenix6.signals import NeutralModeValue, ForwardLimitValue, ForwardLimitSourceValue

from pykit.autolog import autologgable_output
from pykit.logger import Logger
from wpilib import Alert
from typing import Final, Callable
from constants import Constants
from subsystems import StateSubsystem
from subsystems.turret.io import TurretIO
from math import *
from wpimath.geometry import Pose2d
from wpimath.controller import PIDController
from wpilib import DriverStation



"""
TO-DO:

 - Hardstops
    - If the proposed angle from the turret is 180 degrees or more from the turret's current position, have it rotate in the opposite direction instead
        - If the proposed angle is past the hardstop, rotate the robot instead
"""

# Using intake-subsystem.py as a reference
class TurretSubsystem(PIDSubsystem):
    """
    Responsible for aiming horizontally with the turret and vertically with the variable hood.
    """

    # On the reference there's something about a CANrange here and I don't know what that means so I'm leaving it.

    _motor_config = (TalonFXConfiguration()
                     .with_slot0(Constants.TurretConstants.GAINS)
                     .with_motor_output(MotorOutputConfigs().with_neutral_mode(NeutralModeValue.BRAKE))
                     .with_feedback(FeedbackConfigs().with_sensor_to_mechanism_ratio(Constants.TurretConstants.GEAR_RATIO))
                     .with_current_limits(CurrentLimitsConfigs().with_supply_current_limit_enable(True).with_supply_current_limit(Constants.TurretConstants.SUPPLY_CURRENT))
                     )
    
    def __init__(self, io: TurretIO, robot_pose_supplier: Callable[[], Pose2d]) -> None:
        self.controller = PIDController(
            Constants.TurretConstants.GAINS.k_p,
            Constants.TurretConstants.GAINS.k_i,
            Constants.TurretConstants.GAINS.k_d,
            ) 
        super().__init__(self.controller) # Change PID controller and Initial position if needed

        self._turret_motor = TalonFX(Constants.CanIDs.TURRET_TALON)

        self._io: Final[TurretIO] = io
        self._inputs = TurretIO.TurretIOInputs()
        self.robot_pose_supplier = robot_pose_supplier()

        self._motorDisconnectedAlert = Alert("Turret motor is disconnected.", Alert.AlertType.kError)

        self.positionRequest = PositionVoltage(0)

        self.independentAngle = 0
        self.goal = ""

    def periodic(self):

        # Update inputs from hardware/simulation
        self._io.updateInputs(self._inputs)

        # Log inputs to PyKit
        Logger.processInputs("Turret", self._inputs)

        # Update alerts
        self._motorDisconnectedAlert.set(not self._inputs.motorConnected)

        self.currentAngle = self.robot_pose_supplier.rotation() + self.independentAngle



        if self.goal:
            self.rotateTowardsGoal(self.goal)

    """
    # SMELLY CODE THAT IT TURNS OUT WE ACTUALLY NEVER NEEDED

    def getAngleToHub(self, distance, angle, in_radians = True):
        if not in_radians:
            angle = radians(angle)
        horizontal_distance = distance * sin(degrees(angle))
        vertical_distance_to_tag = distance * cos(degrees(angle))
        vertical_distance_to_hub = vertical_distance_to_tag + Constants.AimingConstants.APRILTAGTOHUBCENTRE
        proposed_angle = degrees(atan(horizontal_distance / vertical_distance_to_hub))
        return proposed_angle
    
    def getAngleToPassGoal(self, distance, angle, in_radians = True):
        if not in_radians:
            angle = radians(angle)
    """
        
    def getAngleToGoal(self):
        # If the robot position is in the alliance side, call getANgleToHub before aiming
        # If the robot is in the neutral zone, have it determine what side of the zone it's on so it knows the target to aim at
        match self.goal.lower():
            case "hub":
                xdist = abs(self.robot_pose_supplier.X() - Constants.GoalLocations.BLUEHUB.X) if DriverStation.getAlliance == DriverStation.Alliance.kBlue else abs(self.robot_pose_supplier.X() - Constants.GoalLocations.REDHUB.X)
                ydist = abs(self.robot_pose_supplier.Y() - Constants.GoalLocations.BLUEHUB.Y) if DriverStation.getAlliance == DriverStation.Alliance.kBlue else abs(self.robot_pose_supplier.Y() - Constants.GoalLocations.REDHUB.Y)
            case "outpost":
                xdist = abs(self.robot_pose_supplier.X() - Constants.GoalLocations.BLUEOUTPOSTPASS.X) if DriverStation.getAlliance == DriverStation.Alliance.kBlue else abs(self.robot_pose_supplier.X() - Constants.GoalLocations.REDOUTPOSTPASS.X)
                ydist = abs(self.robot_pose_supplier.Y() - Constants.GoalLocations.BLUEOUTPOSTPASS.Y) if DriverStation.getAlliance == DriverStation.Alliance.kBlue else abs(self.robot_pose_supplier.Y() - Constants.GoalLocations.REDOUTPOSTPASS.Y)
            case "depot":
                xdist = abs(self.robot_pose_supplier.X() - Constants.GoalLocations.BLUEDEPOTPASS.X) if DriverStation.getAlliance == DriverStation.Alliance.kBlue else abs(self.robot_pose_supplier.X() - Constants.GoalLocations.REDDEPOTPASS.X)
                ydist = abs(self.robot_pose_supplier.Y() - Constants.GoalLocations.BLUEDEPOTPASS.Y) if DriverStation.getAlliance == DriverStation.Alliance.kBlue else abs(self.robot_pose_supplier.Y() - Constants.GoalLocations.REDDEPOTPASS.Y)
            case "_":
                raise TypeError("Turret aiming target must be \"hub\", \"outpost\", or \"depot\"")
        target_angle = atan(ydist / xdist)
        return target_angle

    def rotateTowardsGoal(self):
        # This function might not work because it probably isn't periodic so it'll only set the output once and then not check if the angle is correct until it's called again (which is when the target changes)
        targetAngle = self.getAngleToGoal()
        self.positionRequest.position = targetAngle / Constants.TurretConstants.RADTOROTRATIO
        self._turret_motor.set_control(self.positionRequest)

"""
WORK LEFT TO DO

Debug rotateTowardsGoal to be periodically checking
Add logic for hardstop
Make sure that rotation actually works (IndependentAngle is unlimited, but robotAngle may or may not be (in other words, it may limit from 0 to 360 degrees rather than just counting total rotation))
Use CANcoder to figure out the independent rotation of the turret
"""