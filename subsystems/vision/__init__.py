from typing import Callable

from commands2 import Subsystem
from pykit.autolog import autologgable_output
from pykit.logger import Logger
from wpilib import Alert
from wpimath.geometry import Pose2d

from constants import Constants
from subsystems.toast import *
from subsystems.vision.io import VisionIO, PoseObservationType
from util import LoggedTracer


@autologgable_output
class Vision(Subsystem):

    def __init__(self, visionConsumer: Callable[[Pose2d, float, tuple[float, float, float]], None], *ios: VisionIO) -> None:
        super().__init__()
        self._consumer: Final = visionConsumer
        self._io: Final[List[VisionIO]] = list(ios)
        self.setName("Vision")

        # Initialize inputs
        self._inputs: Final[List[VisionIO.VisionIOInputs]] = []
        for i in range(len(ios)):
            self._inputs.append(VisionIO.VisionIOInputs())

        # Initialize disconnected alerts
        self._disconnectedAlerts: Final[List[Alert]] = []
        for i in range(len(ios)):
            self._disconnectedAlerts.append(Alert(f"Vision camera {i} is disconnected.", Alert.AlertType.kWarning))

    def periodic(self) -> None:
        for i in range(len(self._io)):
            self._io[i].updateInputs(self._inputs[i])
            Logger.processInputs(f"Vision/{self._inputs[i].name}", self._inputs[i])
        LoggedTracer.record("Vision/Inputs")

        allTagPoses = []
        allRobotPoses = []
        allRobotPosesAccepted = []
        allRobotPosesRejected = []

        # Loop over cameras
        for i in range(len(self._io)):
            # Update disconnected alert
            self._disconnectedAlerts[i].set(not self._inputs[i].connected)

            # Initialize logging values
            tagPoses = []
            robotPoses = []
            robotPosesAccepted = []
            robotPosesRejected = []

            # Add tag poses
            for tagId in self._inputs[i].tagIds:
                tagPose = Constants.FIELD_LAYOUT.getTagPose(tagId)
                if tagPose is not None:
                    tagPoses.append(tagPose)

            # Loop over pose observations
            for observation in self._inputs[i].poseObservations:
                # Check whether to reject pose
                rejectPose = (observation.tagCount == 0 # Must have at least one tag
                              or (observation.tagCount == 1 and observation.ambiguity > maxAmbiguity) # Cannot be ambiguous
                              or (abs(observation.pose.Z()) > maxZError) # Must have realistic Z cord

                              # Must be within field boundaries
                              or observation.pose.X() < 0.0
                              or observation.pose.X() > Constants.FIELD_LAYOUT.getFieldLength()
                              or observation.pose.Y() < 0.0
                              or observation.pose.Y() > Constants.FIELD_LAYOUT.getFieldWidth())

                # Add pose to log
                robotPoses.append(observation.pose)
                if rejectPose:
                    robotPosesRejected.append(observation.pose)
                else:
                    robotPosesAccepted.append(observation.pose)

                # Skip if rejected
                if rejectPose:
                    continue

                # Calculate standard deviation
                stdDevFactor = observation.averageTagDistance**2 / observation.tagCount
                linearStdDev = linearStdDevBaseline * stdDevFactor
                angularStdDev = angularStdDevBaseline * stdDevFactor
                if observation.poseType == PoseObservationType.MEGATAG_2:
                    linearStdDev *= linearStdDevMegatag2Factor
                    angularStdDev *= angularStdDevMegatag2Factor
                if i < len(cameraStdDevFactors):
                    linearStdDev *= cameraStdDevFactors[i]
                    angularStdDev *= cameraStdDevFactors[i]

                # Send vision observation
                self._consumer(
                    observation.pose.toPose2d(),
                    observation.timestamp,
                    (linearStdDev, linearStdDev, angularStdDev)
                )

            # Log camera data
            Logger.recordOutput(f"Vision/{self._inputs[i].name}/TagPoses", tagPoses)
            Logger.recordOutput(f"Vision/{self._inputs[i].name}/RobotPoses", robotPoses)
            Logger.recordOutput(f"Vision/{self._inputs[i].name}/RobotPosesAccepted", robotPosesAccepted)
            Logger.recordOutput(f"Vision/{self._inputs[i].name}/RobotPosesRejected", robotPosesRejected)
            allTagPoses.extend(tagPoses)
            allRobotPoses.extend(robotPoses)
            allRobotPosesAccepted.extend(robotPosesAccepted)
            allRobotPosesRejected.extend(robotPosesRejected)

        # Log combined data
        Logger.recordOutput("Vision/TagPoses", allTagPoses)
        Logger.recordOutput("Vision/RobotPoses", allRobotPoses)
        Logger.recordOutput("Vision/RobotPosesAccepted", allRobotPosesAccepted)
        Logger.recordOutput("Vision/RobotPosesRejected", allRobotPosesRejected)

        LoggedTracer.record("Vision/Periodic")
