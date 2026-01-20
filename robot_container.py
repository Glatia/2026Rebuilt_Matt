import math
import os.path

import commands2.button
from commands2 import cmd, Command
from commands2.button import CommandXboxController
from pathplannerlib.auto import AutoBuilder, PathPlannerAuto
from pathplannerlib.util import FlippingUtil
from pykit.networktables.loggeddashboardchooser import LoggedDashboardChooser
from wpilib import getDeployDirectory
from wpimath.geometry import Pose2d, Rotation2d

import commands
from commands import fieldRelative
from typing import Optional
from constants import Constants
from robot_config import currentRobot, has_subsystem  # Robot detection (Larry vs Comp)
from subsystems.drive import Drive
from subsystems.climber import ClimberSubsystem
from subsystems.climber.io import ClimberIOTalonFX, ClimberIOSim
from subsystems.intake import IntakeSubsystem
from subsystems.drive.gyro import GyroIOPigeon2, GyroIOSim
from subsystems.drive.module import ModuleIOTalonFX, ModuleIOSim
from subsystems.toast import moduleConfigs
from subsystems.vision import Vision
from subsystems.vision.io import VisionIOLimelight 

from phoenix6.configs import TalonFXConfiguration
from phoenix6.configs.config_groups import NeutralModeValue, MotorOutputConfigs, FeedbackConfigs
from pykit.logger import Logger


class RobotContainer:
    def __init__(self) -> None:
        self._driver = CommandXboxController(0)
        
        # Log which robot we're running on (for debugging)
        Logger.recordMetadata("Robot", currentRobot.name)
        print(f"Initializing RobotContainer for: {currentRobot.name}")
        
        # Initialize subsystems as None - will be created conditionally
        self._climber: Optional[ClimberSubsystem] = None
        self._intake: Optional[IntakeSubsystem] = None

        match Constants.currentMode:
            case Constants.Mode.REAL:
                # Real robot, instantiate hardware IO implementations
                self._drivetrain = Drive(
                    GyroIOPigeon2(),
                    ModuleIOTalonFX(moduleConfigs[0]),
                    ModuleIOTalonFX(moduleConfigs[1]),
                    ModuleIOTalonFX(moduleConfigs[2]),
                    ModuleIOTalonFX(moduleConfigs[3]),
                    lambda _: None
                )
                self._vision = Vision(
                    self._drivetrain.addVisionMeasurement,
                    VisionIOLimelight("limelight", self._drivetrain.getRotation)
                )

                # Create climber only if it exists on this robot
                if has_subsystem("climber"):
                    # Create climber motor config
                    # Note: Constants.ClimberConstants values are automatically selected based on detected robot
                    climber_motor_config = (TalonFXConfiguration()
                        .with_slot0(Constants.ClimberConstants.GAINS)
                        .with_motor_output(MotorOutputConfigs().with_neutral_mode(NeutralModeValue.BRAKE))
                        .with_feedback(FeedbackConfigs().with_sensor_to_mechanism_ratio(Constants.ClimberConstants.GEAR_RATIO))
                    )
                    
                    # Create climber real hardware IO
                    # Note: Constants.CanIDs.CLIMB_TALON is automatically set based on detected robot (Larry vs Comp)
                    climber_io = ClimberIOTalonFX(
                        Constants.CanIDs.CLIMB_TALON,  # Different CAN ID for Larry vs Comp
                        Constants.ClimberConstants.SERVO_PORT,
                        climber_motor_config
                    )
                    
                    # Create climber subsystem with real hardware IO
                    self._climber = ClimberSubsystem(climber_io)
                    Logger.recordMetadata("Climber", "Present")
                else:
                    Logger.recordMetadata("Climber", "Not Present")
                    print("Climber subsystem not available on this robot")

            case Constants.Mode.SIM:
                # Sim robot, instantiate physics sim IO implementations (if available)
                self._drivetrain = Drive(
                    GyroIOSim(),
                    ModuleIOSim(moduleConfigs[0]),
                    ModuleIOSim(moduleConfigs[1]),
                    ModuleIOSim(moduleConfigs[2]),
                    ModuleIOSim(moduleConfigs[3]),
                    lambda _: None
                )
                self._vision = Vision(
                    self._drivetrain.addVisionMeasurement,
                    VisionIOLimelight("limelight", self._drivetrain.getRotation)
                )

                # Create climber only if it exists on this robot
                if has_subsystem("climber"):
                    # Create climber subsystem with simulation IO
                    self._climber = ClimberSubsystem(ClimberIOSim())
                    Logger.recordMetadata("Climber", "Present")
                else:
                    Logger.recordMetadata("Climber", "Not Present")
                    print("Climber subsystem not available on this robot")

        # Auto chooser
        self.autoChooser: LoggedDashboardChooser[Command] = LoggedDashboardChooser("Selected Auto")

        auto_files = os.listdir(os.path.join(getDeployDirectory(), "pathplanner", "autos"))
        for file in auto_files:
            file = file.removesuffix(".auto")
            self.autoChooser.addOption(file, PathPlannerAuto(file, False))
            self.autoChooser.addOption(f"{file} (Mirrored)", PathPlannerAuto(file, True))
        self.autoChooser.setDefaultOption("None", cmd.none())
        self.autoChooser.addOption("Basic Leave",
        self._drivetrain.run(lambda: commands.robotRelative(self._drivetrain, lambda: 0.25, lambda: 0, lambda: 0)).withTimeout(1.0))

        self.setupControllerBindings()

    def readyRobotForMatch(self) -> None:
        auto = self.autoChooser.getSelected()
        if isinstance(auto, PathPlannerAuto):
            if AutoBuilder.shouldFlip():
                self._drivetrain.setPose(FlippingUtil.flipFieldPose(auto._startingPose))
            else:
                self._drivetrain.setPose(auto._startingPose)

    def setupControllerBindings(self) -> None:
        # DRIVE CONTROLLER
        self._drivetrain.setDefaultCommand(fieldRelative(
            self._drivetrain,
            lambda: -self._driver.getLeftY(),
            lambda: -self._driver.getLeftX(),
            lambda: -self._driver.getRightX()
        ))

        # Left bumper: Robot relative
        self._driver.leftBumper().whileTrue(
            commands.robotRelative(
                self._drivetrain,
                lambda: -self._driver.getLeftY(),
                lambda: -self._driver.getLeftX(),
                lambda: -self._driver.getRightX()
            )
        )

        # A: X-brake
        self._driver.a().whileTrue(commands.brakeWithX(self._drivetrain))

        # Left trigger: Align to closest left branch
        self._driver.leftTrigger().whileTrue(
            commands.alignToClosestBranch(
                self._drivetrain,
                commands.BranchSide.LEFT,
                lambda: -self._driver.getLeftY(),
                lambda: -self._driver.getLeftX()
            )
        )

        # Right trigger: Align to closest right branch
        self._driver.rightTrigger().whileTrue(
            commands.alignToClosestBranch(
                self._drivetrain,
                commands.BranchSide.RIGHT,
                lambda: -self._driver.getLeftY(),
                lambda: -self._driver.getLeftX()
            )
        )

        # Reset gyro to 0 when start button is pressed.
        self._driver.start().onTrue(cmd.runOnce(
            lambda: self._drivetrain.setPose(
                Pose2d(self._drivetrain.getPose().translation(), Rotation2d() + Rotation2d(math.pi) if AutoBuilder.shouldFlip() else Rotation2d())
            ), self._drivetrain
        ).ignoringDisable(True))

        # Feedforward characterization
        self._driver.x().whileTrue(commands.feedforwardCharacterization(self._drivetrain))

        # Wheel radius characterization
        self._driver.b().whileTrue(commands.wheelRadiusCharacterization(self._drivetrain))

    def get_autonomous_command(self) -> commands2.Command:
        return self.autoChooser.getSelected()
    
    def get_climber(self) -> Optional[ClimberSubsystem]:
        """Get the climber subsystem if it exists on this robot."""
        return self._climber
    
    def get_intake(self) -> Optional[IntakeSubsystem]:
        """Get the intake subsystem if it exists on this robot."""
        return self._intake
    
    def has_climber(self) -> bool:
        """Check if climber subsystem exists on this robot."""
        return self._climber is not None
    
    def has_intake(self) -> bool:
        """Check if intake subsystem exists on this robot."""
        return self._intake is not None