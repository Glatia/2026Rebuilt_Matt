from abc import ABC
from dataclasses import dataclass, field
from enum import auto, Enum
from typing import List, Callable, Set, Tuple

from ntcore import NetworkTableInstance, DoubleArrayPublisher, DoublePublisher, DoubleSubscriber, DoubleArraySubscriber
from pykit.autolog import autolog
from typing_extensions import Final
from wpilib import DriverStation, RobotController
from wpimath.geometry import Pose3d, Rotation2d, Rotation3d
from wpimath.units import celsius, degreesToRadians


class PoseObservationType(Enum):
    MEGATAG_1 = auto()
    MEGATAG_2 = auto()
    OTHER = auto()


@dataclass
class TargetObservation:
    """Represents the angle to a simple target, not used for pose estimation."""
    tx: Rotation2d = field(default_factory=Rotation2d)
    ty: Rotation2d = field(default_factory=Rotation2d)


@dataclass
class PoseObservation:
    """Robot pose sample used for pose estimation."""
    timestamp: float = 0.0
    ambiguity: float = 0.0
    tagCount: int = 0
    averageTagDistance: float = 0
    poseType: PoseObservationType = PoseObservationType.OTHER
    pose: Pose3d = field(default_factory=Pose3d)


class VisionIO(ABC):

    @autolog
    @dataclass
    class VisionIOInputs:
        # Basic Info
        connected: bool = False
        name: str = "Unnamed Camera"

        # Pose Estimation
        poseObservations: List[PoseObservation] = field(default_factory=list)
        tagIds: List[int] = field(default_factory=list)

        # Hardware
        fps: float = 0
        temperature: celsius = 0.0
        latency: float = 0.0

        targetAngle: Tuple[Rotation2d, Rotation2d] = field(default_factory=tuple)
        """Simple targeting (tx and ty)"""

    def updateInputs(self, inputs: VisionIOInputs) -> None:
        pass


class VisionIOLimelight(VisionIO):

    def __init__(self, name: str, rotationSupplier: Callable[[], Rotation2d]) -> None:
        """Creates a new VisionIOLimelight.

        :param name: The configured name of the Limelight.
        :param rotationSupplier: Supplier for the current estimated rotation, used for MegaTag 2.
        """
        table = NetworkTableInstance.getDefault().getTable(name)
        self.name = name
        self._rotationSupplier: Final[Callable[[], Rotation2d]] = rotationSupplier
        self._orientationPublisher: Final[DoubleArrayPublisher] = table.getDoubleArrayTopic("robot_orientation_set").publish()
        self._throttlePublisher: Final[DoublePublisher] = table.getDoubleTopic("throttle_set").publish()
        self._latencySubscriber: Final[DoubleSubscriber] = table.getDoubleTopic("tl").subscribe(0.0)
        self._txSubscriber: Final[DoubleSubscriber] = table.getDoubleTopic("tx").subscribe(0.0)
        self._tySubscriber: Final[DoubleSubscriber] = table.getDoubleTopic("ty").subscribe(0.0)
        self._hardwareSubscriber: Final[DoubleArraySubscriber] = table.getDoubleArrayTopic("hw").subscribe([-1]*4)
        self._megatag1Subscriber: Final[DoubleArraySubscriber] = table.getDoubleArrayTopic("botpose_wpiblue").subscribe([])
        self._megatag2Subscriber: Final[DoubleArraySubscriber] = table.getDoubleArrayTopic("botpose_orb_wpiblue").subscribe([])

    @staticmethod
    def _parsePose(rawLLArray: List[float]) -> Pose3d:
        """Parses the 3D pose from a Limelight botpose array."""
        return Pose3d(
            rawLLArray[0],
            rawLLArray[1],
            rawLLArray[2],
            Rotation3d(
                degreesToRadians(rawLLArray[3]),
                degreesToRadians(rawLLArray[4]),
                degreesToRadians(rawLLArray[5])
            )
        )

    def updateInputs(self, inputs: VisionIO.VisionIOInputs) -> None:
        inputs.name = self.name

        # Update hardware status
        hw = self._hardwareSubscriber.get()
        inputs.fps = hw[0]
        inputs.temperature = hw[3]
        inputs.latency = self._latencySubscriber.get()

        # Update throttle when disabled or overheating
        if DriverStation.isDisabled():
            self._throttlePublisher.set(150.0)
        elif hw[3] >= 65:
            self._throttlePublisher.set(int(hw[3]-65))
        else:
            self._throttlePublisher.set(0.0)

        # Update connection status whether an update has been seen in 250ms
        inputs.connected = ((RobotController.getFPGATime() - self._latencySubscriber.getLastChange()) / 1000) < 250

        # Update target observation
        inputs.targetAngle = (Rotation2d.fromDegrees(self._txSubscriber.get()), Rotation2d.fromDegrees(self._tySubscriber.get()))

        # Update orientation for MegaTag 2
        self._orientationPublisher.set([self._rotationSupplier().degrees(), 0.0, 0.0, 0.0, 0.0, 0.0])
        NetworkTableInstance.getDefault().flush() # Update all values before reading

        # Read new pose observations from NetworkTables
        tagIds: Set[int] = set()
        poseObservations = []

        # Megatag 1
        for sample in self._megatag1Subscriber.readQueue():
            # Check if sample is empty
            if len(sample.value) == 0:
                continue

            # Parse for tagIds
            # (index 11 is the 1st raw fiducial tagId, step by 7 for each raw fiducial after)
            for i in range(11, len(sample.value), 7):
                tagIds.add(int(sample.value[i]))
            poseObservations.append(
                PoseObservation(
                    # Timestamp, based on timestamp of publish and latency
                    sample.time * 1.0e6 - sample.value[6] * 1.0e-3,

                    # Ambiguity, using only the first tag because ambiguity isn't applicable for multitag
                    sample.value[17] if len(sample.value) >= 18 else 0.0,

                    # Tag count
                    int(sample.value[7]),

                    # Average tag distance
                    sample.value[9],

                    # Observation type
                    PoseObservationType.MEGATAG_1,

                    # 3D pose estimate
                    self._parsePose(sample.value)
                )
            )

        # Megatag 2
        for sample in self._megatag2Subscriber.readQueue():
            if len(sample.value) == 0:
                continue

            for i in range(11, len(sample.value), 7):
                tagIds.add(int(sample.value[i]))
            poseObservations.append(
                PoseObservation(
                    # Timestamp, based on server timestamp of publish and latency
                    sample.time * 1.0e6 - sample.value[6] * 1.0e-3,

                    # Ambiguity, zeroed because the pose is already disambiguated
                    0.0,

                    # Tag count
                    int(sample.value[7]),

                    # Average tag distance
                    sample.value[9],

                    # Observation type
                    PoseObservationType.MEGATAG_2,

                    # 3D pose estimate
                    self._parsePose(sample.value)
                )
            )

        # Save to inputs
        inputs.poseObservations = poseObservations
        inputs.tagIds = list(tagIds)

