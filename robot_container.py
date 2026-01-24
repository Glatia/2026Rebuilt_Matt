import math
import os.path

import commands2
from commands2.button import CommandXboxController, Trigger
from commands2 import cmd, Command
from pathplannerlib.auto import AutoBuilder, PathPlannerAuto
from pathplannerlib.util import FlippingUtil
from pykit.networktables.loggeddashboardchooser import LoggedDashboardChooser
from wpilib import getDeployDirectory, SmartDashboard
from wpimath.geometry import Pose2d, Rotation2d
from phoenix6 import swerve

from typing import Optional
from constants import Constants
from generated.tuner_constants import TunerConstants
from robot_config import currentRobot, has_subsystem  # Robot detection (Larry vs Comp)
from subsystems.swerve import SwerveSubsystem
from subsystems.swerve.requests import DriverAssist
from subsystems.climber import ClimberSubsystem
from subsystems.climber.io import ClimberIOTalonFX, ClimberIOSim
from subsystems.intake import IntakeSubsystem
# from subsystems.vision import VisionSubsystem

from phoenix6.configs import TalonFXConfiguration
from phoenix6.configs.config_groups import NeutralModeValue, MotorOutputConfigs, FeedbackConfigs
from pykit.logger import Logger


class RobotContainer:
    def __init__(self) -> None:
        self._driver_controller = CommandXboxController(0)
        
        # Log which robot we're running on (for debugging)
        Logger.recordMetadata("Robot", currentRobot.name)
        print(f"Initializing RobotContainer for: {currentRobot.name}")
        
        # Initialize subsystems as None - will be created conditionally
        self._climber: Optional[ClimberSubsystem] = None
        self._intake: Optional[IntakeSubsystem] = None

        match Constants.currentMode:
            case Constants.Mode.REAL:
                # Real robot, instantiate hardware IO implementations
                if has_subsystem("drivetrain"):
                    self._drivetrain = TunerConstants.create_drivetrain()
                
                """
                if has_subsystem("vision"):
                    self._vision = VisionSubsystem(
                        self._drivetrain,
                        Constants.VisionConstants.FRONT_RIGHT,
                        Constants.VisionConstants.FRONT_CENTER,
                        Constants.VisionConstants.FRONT_LEFT,
                        #Constants.VisionConstants.BACK_CENTER,
                    )
                """

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
                self._drivetrain = TunerConstants.create_drivetrain()
                """
                self._vision = VisionSubsystem(
                    self._drivetrain,
                    Constants.VisionConstants.FRONT_RIGHT,
                    Constants.VisionConstants.FRONT_CENTER,
                    Constants.VisionConstants.FRONT_LEFT,
                    #Constants.VisionConstants.BACK_CENTER,
                )
                """

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
        self.autoChooser.onChange(
            lambda _: self._set_correct_swerve_position()
        )
        self.autoChooser.addOption("Basic Leave",
            self._drivetrain.apply_request(lambda: self._robot_centric.with_velocity_x(1)).withTimeout(1.0)
        )
        # SmartDashboard.putData("Selected Auto", self.autoChooser)
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
        hid = self._driver_controller.getHID()
        self._drivetrain.setDefaultCommand(
            self._drivetrain.apply_request(
                lambda: self._field_centric
                .with_velocity_x(-hid.getLeftY() * self._max_speed)
                .with_velocity_y(-hid.getLeftX() * self._max_speed)
                .with_rotational_rate(-self._driver_controller.getRightX() * self._max_angular_rate)
            )
        )

        self._driver_controller.leftBumper().whileTrue(
            self._drivetrain.apply_request(
                lambda: self._robot_centric
                .with_velocity_x(-hid.getLeftY() * self._max_speed)
                .with_velocity_y(-hid.getLeftX() * self._max_speed)
                .with_rotational_rate(-self._driver_controller.getRightX() * self._max_angular_rate)
            )
        )

        # Reset gyro to 0 when start button is pressed.
        self._driver_controller.start().onTrue(cmd.runOnce(
            lambda: self._drivetrain.setPose(
                Pose2d(self._drivetrain.getPose().translation(), Rotation2d() + Rotation2d(math.pi) if AutoBuilder.shouldFlip() else Rotation2d())
            ), self._drivetrain
        ).ignoringDisable(True))

        self._driver_controller.a().whileTrue(self._drivetrain.apply_request(lambda: self._brake))
        self._driver_controller.b().whileTrue(
            self._drivetrain.apply_request(
                lambda: self._point.with_module_direction(Rotation2d(-hid.getLeftY(), -hid.getLeftX()))
            )
        )

        Trigger(lambda: self._driver_controller.getLeftTriggerAxis() > 0.75).onTrue(
            self._drivetrain.runOnce(lambda: self._driver_assist.with_target_pose(self._drivetrain.get_closest_branch(self._drivetrain.BranchSide.LEFT)))
        ).whileTrue(
            self._drivetrain.apply_request(
                lambda: self._driver_assist
                .with_velocity_x(-hid.getLeftY() * self._max_speed)
                .with_velocity_y(-hid.getLeftX() * self._max_speed)
            )
        )

        Trigger(lambda: self._driver_controller.getRightTriggerAxis() > 0.75).onTrue(
            self._drivetrain.runOnce(lambda: self._driver_assist.with_target_pose(self._drivetrain.get_closest_branch(self._drivetrain.BranchSide.RIGHT)))
        ).whileTrue(
            self._drivetrain.apply_request(
                lambda: self._driver_assist
                .with_velocity_x(-hid.getLeftY() * self._max_speed)
                .with_velocity_y(-hid.getLeftX() * self._max_speed)
            )
        )

    def _setup_swerve_requests(self):
        self._field_centric = (
            swerve.requests.FieldCentric()
            .with_deadband(0)
            .with_rotational_deadband(0)
            .with_drive_request_type(swerve.SwerveModule.DriveRequestType.VELOCITY)
            .with_steer_request_type(swerve.SwerveModule.SteerRequestType.POSITION)
        )

        self._robot_centric: swerve.requests.RobotCentric = (
            swerve.requests.RobotCentric()
            .with_deadband(0)
            .with_rotational_deadband(0)
            .with_drive_request_type(swerve.SwerveModule.DriveRequestType.VELOCITY)
            .with_steer_request_type(swerve.SwerveModule.SteerRequestType.POSITION)
        )

        self._driver_assist: DriverAssist = (
            DriverAssist()
            .with_deadband(self._max_speed * 0.01)
            .with_rotational_deadband(self._max_angular_rate * 0.02)
            .with_drive_request_type(swerve.SwerveModule.DriveRequestType.VELOCITY)
            .with_steer_request_type(swerve.SwerveModule.SteerRequestType.POSITION)
            .with_translation_pid(Constants.AutoAlignConstants.TRANSLATION_P, Constants.AutoAlignConstants.TRANSLATION_I, Constants.AutoAlignConstants.TRANSLATION_D)
            .with_heading_pid(Constants.AutoAlignConstants.HEADING_P, Constants.AutoAlignConstants.HEADING_I, Constants.AutoAlignConstants.HEADING_D)
        )
        
        self._brake = swerve.requests.SwerveDriveBrake()
        self._point = swerve.requests.PointWheelsAt()

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