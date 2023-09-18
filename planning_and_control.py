from middleware import Node
from constants import Topics, Config, Map
from abc import ABC, abstractmethod
from utils import LogOnlyChange, calculateLinearAndRotationalError, calculateTarget_LookAhead
from pid import PID
import numpy as np

class Mission(ABC):
  def __init__(self, pAndC, initTimestamp):
    super().__init__()
    self.pAndC = pAndC
    self.initTimestamp = initTimestamp
  
  @abstractmethod
  def tick(self, timestamp) -> "Mission":
    pass


class NoMission(Mission):
  """A mission that does nothing
  """
  def __init__(self, pAndC, initTimestamp):
    super().__init__(pAndC, initTimestamp)

  def tick(self, timestamp):
    return self


class ExploreForLocalizationMission(Mission):
  """A mission to explore the map until the first
  localization message arrives. This means we have seen
  an AprilTag.

  Just by turning right
  """
  def __init__(self, pAndC, initTimestamp):
    super().__init__(pAndC, initTimestamp)

  def tick(self, timestamp):
    self.pAndC.actuation.reset()
    if self.pAndC.latestRobotPoseTimestamp is None:
      self.pAndC.actuation.spinCounterClockWise(20)
      self.pAndC.motorCmdsPub.publish(timestamp, self.pAndC.actuation.generateMotorCmd())
      return self

    else:
      # stop the motors
      self.pAndC.motorCmdsPub.publish(timestamp, self.pAndC.actuation.generateMotorCmd())
      # and return the Mission.
      # TODO: choose the next mission properly
      return ExploreForTennisBallsMission(self.pAndC, timestamp)

class ExploreForTennisBallsMission(Mission):
  def __init__(self, pAndC, initTimestamp):
    super().__init__(pAndC, initTimestamp)
    self.conditionMetCounter = 0
  
  def tick(self, timestamp):
    self.pAndC.actuation.reset()
    if self.pAndC.latestBallPositionsTimestamp and \
        self.pAndC.latestBallPositionsTimestamp > self.initTimestamp:

      self.initTimestamp = timestamp
      if self.conditionMetCounter > 1:
        # stop the robot
        self.pAndC.motorCmdsPub.publish(
            timestamp, self.pAndC.actuation.generateMotorCmd())
        self.conditionMetCounter = self.conditionMetCounter + 1
        # set the target position
        self.pAndC.targetPositionInRobotFrame = self.pAndC.latestBallPositionsInRobotFrame[0]
        # TODO return the next mission
        return LocalGoToMission(self.pAndC, timestamp)
      
      else:
        self.conditionMetCounter += 1
    
    print(self.conditionMetCounter, self.pAndC.latestBallPositionsTimestamp, self.initTimestamp)

    self.pAndC.actuation.spinCounterClockWise(20)
    self.pAndC.motorCmdsPub.publish(timestamp, self.pAndC.actuation.generateMotorCmd())
    return self


class LocalGoToMission(Mission):
  def __init__(self, pAndC, initTimestamp):
    super().__init__(pAndC, initTimestamp)
    self.forwardPID = PID(kp=50, ki=0, kd=0)
    self.ccwSpinPID = PID(kp=10, ki=0, kd=0)
  
  def tick(self, timestamp):
    self.pAndC.actuation.reset()
    localTarget = self.pAndC.targetPositionInRobotFrame[:2, 0] - Config.zero_offset
    if np.linalg.norm(localTarget) < 0.2:
      self.pAndC.actuation.apply()
      return NoMission(self.pAndC, timestamp)

    newLocalTarget = calculateTarget_LookAhead(np.zeros_like(localTarget), localTarget)
    forwardErrorMeter, ccwSpinErrorRadian = calculateLinearAndRotationalError(np.identity(4, dtype=float), newLocalTarget)

    spinCommand = self.ccwSpinPID.calculate(ccwSpinErrorRadian)
    forwardCommand = self.forwardPID.calculate(forwardErrorMeter)
    self.pAndC.actuation.reset()
    self.pAndC.actuation.spinCounterClockWise(spinCommand)
    self.pAndC.actuation.goForward(forwardCommand)
    self.pAndC.actuation.apply()
    return self

class PlanningAndControlNode(Node):
  def __init__(self):
    super().__init__("PlanningAndControlNode")
    self.motorCmdsPub = self.create_publisher(Topics.motorCommands)
    self.ballPositionSub = self.create_subscriber(Topics.ballPositions, self.ballPosition_cb)
    self.localizationSub = self.create_subscriber(Topics.fusedPose, self.localization_cb)
    self.tickSub = self.create_subscriber(Topics.isMoving, self.tick)
    self.actuation = None
    self.latestBallPositionsInMapFrame = None
    self.latestBallPositionsInRobotFrame = None
    self.latestBallPositionsTimestamp = None
    self.latestRobotPose = None
    self.latestRobotPoseTimestamp = None
    self.ballPositionsInMap = None
    self.currentMission = None
    self.targetPositionInRobotFrame = None
    self.log = None
  
  def ballPosition_cb(self, timestamp, ballPositions):
    self.latestBallPositionsInRobotFrame = ballPositions
    self.latestBallPositionsTimestamp = timestamp

  def localization_cb(self, timestamp, localization):
    self.latestRobotPose = localization
    self.latestRobotPoseTimestamp = timestamp
    if not self.latestBallPositionsInRobotFrame is None:
      self.ballPositionsInMap = self.latestRobotPose @ self.latestBallPositionsInRobotFrame

  def tick(self, timestamp, isMoving):
    """ This is subscribing to IsMoving 
    and is used as a tick function. The isMoving parameter
    might not be used
    """
    if self.actuation is None:
      from robot_interface_node import Actuation
      self.actuation = Actuation()
      self.log = LogOnlyChange()
    
    # init mission to find an april tag
    if self.currentMission is None:
      self.currentMission = ExploreForLocalizationMission(self, timestamp)

    self.log(f"Current Mission: {type(self.currentMission).__name__}")
    nextMission = self.currentMission.tick(timestamp)
    self.currentMission = nextMission
    
    # We need to support multiple missions
    # Each mission needs to define the next mission
    # Either by using an enum or returning a new mission object
    # Each mission would have access to PlanningAndControlNode data members or the self instant.

    # Mission: Search for 1st april Tag for localization

    # Mission: Search for a tennis ball

    # Mission: go to the ball
    # choose a ball to grab
    # path plan to the ball
    # run PID control to calculate motor values
    # apply actuation

    # Mission: grab the ball and move the arm up

    # Mission: Go to the other side of the map

    # Mission: Drop the ball

    # Mission: go back to our map

    # Mission: lower the arm

    # Start over.

    
    self.actuation.reset()
    self.motorCmdsPub.publish(timestamp, self.actuation.generateMotorCmd())

def test_planning_and_control_node():
  from constants import Config
  from perception import PerceptionNode
  from localization import generateLocalizationNodes
  from robot_interface_node import SensorPublisherNode
  from middleware import start_subscribers

  robot = Config.getRobot()
  sensorPublisherNode = SensorPublisherNode()
  planningAndControlNode = PlanningAndControlNode()
  generateLocalizationNodes()
  perceptionNode = PerceptionNode()
  start_subscribers()
  
  while True:
    robot.update()
    sensorPublisherNode.publishSensorData(robot)


if __name__ == "__main__":
  import multiprocessing as mp
  mp.set_start_method('spawn', force=True)
  test_planning_and_control_node()