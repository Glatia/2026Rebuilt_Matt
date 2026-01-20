import os.path
from typing import Final

import wpilib
from commands2 import CommandScheduler, Command
from phoenix6 import SignalLogger
from pykit.loggedrobot import LoggedRobot
from pykit.logger import Logger
from pykit.logreplaysource import LogReplaySource
from pykit.networktables.nt4Publisher import NT4Publisher
from pykit.wpilog.wpilogwriter import WPILOGWriter
from wpilib import DataLogManager, DriverStation, RobotBase
from wpinet import WebServer

from constants import Constants
from lib import elasticlib
from lib.elasticlib import Notification, NotificationLevel
from robot_container import RobotContainer
from util import LoggedTracer


class Robot(LoggedRobot):

    def __init__(self) -> None:
        super().__init__()

        Logger.recordMetadata("TuningMode", str(Constants.tuningMode).lower())
        Logger.recordMetadata("RuntimeType", RobotBase.getRuntimeType().name)
        Logger.recordMetadata("RobotMode", Constants.currentMode.name)

        match Constants.currentMode:
            # Running on a real robot, log to a USB stick ("/U/logs")
            case Constants.Mode.REAL:
                deploy_config = wpilib.deployinfo.getDeployData()
                if deploy_config is not None:
                    Logger.recordMetadata(
                        "Deploy Host", deploy_config.get("deploy-host", "")
                    )
                    Logger.recordMetadata(
                        "Deploy User", deploy_config.get("deploy-user", "")
                    )
                    Logger.recordMetadata(
                        "Deploy Date", deploy_config.get("deploy-date", "")
                    )
                    Logger.recordMetadata(
                        "Code Path", deploy_config.get("code-path", "")
                    )
                    Logger.recordMetadata("Git Hash", deploy_config.get("git-hash", ""))
                    Logger.recordMetadata(
                        "Git Branch", deploy_config.get("git-branch", "")
                    )
                    Logger.recordMetadata(
                        "Git Description", deploy_config.get("git-desc", "")
                    )
                Logger.addDataReciever(WPILOGWriter())
                Logger.addDataReciever(NT4Publisher(True))

            # Running a physics simulator, log to NT
            case Constants.Mode.SIM:
                Logger.addDataReciever(NT4Publisher(True))

            # Replaying a log, set up replay source
            case Constants.Mode.REPLAY:
                self.useTiming = False
                Logger.setReplaySource(LogReplaySource())
                Logger.addDataReciever(WPILOGWriter(None))

        # Start PyKit logger
        Logger.start()

        # Disable joystick connection is we aren't connected to a real field (or simulating)
        DriverStation.silenceJoystickConnectionWarning(not DriverStation.isFMSAttached() or RobotBase.isSimulation())

        # CTRE status signal logging (only enabled if USB drive is attached)
        SignalLogger.enable_auto_logging(False)
        SignalLogger.start() if SignalLogger.set_path("/media/sda1/ctre-logs/").is_ok() else SignalLogger.stop()

        # Setup web server for downloading Elastic layouts
        # (https://frc-elastic.gitbook.io/docs/additional-features-and-references/remote-layout-downloading)
        WebServer.getInstance().start(5800, self.get_deploy_directory())

        self.container: Final[RobotContainer] = RobotContainer()
        self._currentAuto: Command | None = None

    @staticmethod
    def get_deploy_directory():
        if os.path.exists("/home/lvuser"):
            return "/home/lvuser/py/deploy"
        else:
            return os.path.join(os.getcwd(), "deploy")

    def robotPeriodic(self) -> None:
        LoggedTracer.reset()
        CommandScheduler.getInstance().run()

    def disabledInit(self):
        DataLogManager.log("Robot disabled")

    def disabledPeriodic(self) -> None:
        # Workaround: Check to see if the autoChooser has a new auto, then set the robot pose
        if self._currentAuto != self.container.get_autonomous_command():
            self.container.readyRobotForMatch()
            self._currentAuto = self.container.get_autonomous_command()
            print(f"Robot pose set for auto \"{self._currentAuto.getName()}\"")

    def disabledExit(self):
        DataLogManager.log("Exiting disabled mode...")

    def autonomousInit(self) -> None:
        DataLogManager.log("Autonomous period started")

        selected_auto = self.container.get_autonomous_command()
        if selected_auto is not None:
            DataLogManager.log(f"Selected Auto: {selected_auto.getName()}")
            selected_auto.schedule()

        elasticlib.select_tab("Autonomous")
            
    def autonomousPeriodic(self) -> None:
        pass
    
    def autonomousExit(self) -> None:
        DataLogManager.log("Autonomous period ended")
        elasticlib.select_tab("Teleop")
            
    def teleopInit(self) -> None:
        DataLogManager.log("Teleoperated period started")

    def teleopPeriodic(self) -> None:
        pass

    def teleopExit(self) -> None:
        DataLogManager.log("Teleoperated period ended")
        if DriverStation.isFMSAttached():
            elasticlib.send_notification(
                Notification(
                    level=NotificationLevel.INFO.value,
                    title="Good match!",
                    description="(again)" if DriverStation.getReplayNumber() > 1 else ""
                )
            )

    def testInit(self):
        DataLogManager.log("Test period started")
        CommandScheduler.getInstance().cancelAll()
        elasticlib.select_tab("Debug")
        SignalLogger.start()

    def testPeriodic(self) -> None:
        pass

    def testExit(self):
        DataLogManager.log("Test period ended")

    def _simulationInit(self):
        pass

    def _simulationPeriodic(self) -> None:
        pass
