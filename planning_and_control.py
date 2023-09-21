from middleware import Node
from constants import Topics, Config, Map
from abc import ABC, abstractmethod
from utils import LogOnlyChange, calculateLinearAndRotationalError, calculateTarget_LookAhead
from pid import PID
import numpy as np

# current orders
# ExploreForTennisBallsMission
# LocalGoToMission
# CloseArmMission
# MoveArmUpMission
# ExploreForLocalizationMission
# GlobalGoToMission # the other side
#
# NoMission for now
# Drop the other side
# Do it again

class CompetitionConstant:
  ExploreRotation = 25
  LOCAL_CONTROLLER_FORWARD_KP = 85
  LOCAL_CONTROLLER_FORWARD_KI = 0.1
  LOCAL_CONTROLLER_ROTATIONAL_KP = 35

class HomeConstants:
  ExploreRotation = 20
  LOCAL_CONTROLLER_FORWARD_KP = 80
  LOCAL_CONTROLLER_FORWARD_KI = 0.1
  LOCAL_CONTROLLER_ROTATIONAL_KP = 30

Constants = CompetitionConstant


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

class ExploreForTennisBallsMission(Mission):
  def __init__(self, pAndC, initTimestamp):
    super().__init__(pAndC, initTimestamp)
    self.conditionMetCounter = 0
  
  def tick(self, timestamp):
    self.pAndC.actuation.reset()
    if self.pAndC.latestBallPositionsTimestamp and \
        self.pAndC.latestBallPositionsTimestamp > self.initTimestamp:

      self.initTimestamp = timestamp
      if self.conditionMetCounter > -1 :
        # stop the robot
        self.pAndC.motorCmdsPub.publish(
            timestamp, self.pAndC.actuation.generateMotorCmd())
        self.conditionMetCounter = self.conditionMetCounter + 1
        # TODO return the next mission
        return LocalGoToMission(self.pAndC, timestamp)
      
      else:
        self.conditionMetCounter += 1
    
    self.pAndC.log(f"{self.conditionMetCounter}, {self.pAndC.latestBallPositionsTimestamp}, {self.initTimestamp}")

    self.pAndC.actuation.spinCounterClockWise(Constants.ExploreRotation)
    self.pAndC.motorCmdsPub.publish(timestamp, self.pAndC.actuation.generateMotorCmd())
    return self


class LocalGoToMission(Mission):
  def __init__(self, pAndC, initTimestamp):
    super().__init__(pAndC, initTimestamp)
    self.forwardPID = PID(kp=Constants.LOCAL_CONTROLLER_FORWARD_KP,
                          ki=0,
                          kd=0)
    self.forwardPIDNearby = PID(kp=Constants.LOCAL_CONTROLLER_FORWARD_KP,
                          ki=Constants.LOCAL_CONTROLLER_FORWARD_KI,
                          kd=0)
    self.ccwSpinPID = PID(kp=Constants.LOCAL_CONTROLLER_ROTATIONAL_KP,
                          ki=0,
                          kd=0)
  
  def tick(self, timestamp):
    self.pAndC.actuation.reset()

    if abs(self.pAndC.latestBallPositionsTimestamp - timestamp) > 2:
      self.pAndC.motorCmdsPub.publish(timestamp, self.pAndC.actuation.generateMotorCmd())
      return ExploreForTennisBallsMission(self.pAndC, timestamp)

    # set the target position
    localTarget = self.pAndC.latestBallPositionsInRobotFrame[:2, 0] - Config.zero_offset
    if np.linalg.norm(localTarget) < 0.1:
      print("localTarget:", localTarget)
      self.pAndC.motorCmdsPub.publish(timestamp, self.pAndC.actuation.generateMotorCmd())
      self.pAndC.log("LocalGoToMission: Target reached!")
      return CloseArmMission(self.pAndC, timestamp)

    newLocalTarget, isClose = calculateTarget_LookAhead(np.zeros_like(localTarget), localTarget, 0.2)
    forwardErrorMeter, ccwSpinErrorRadian = calculateLinearAndRotationalError(np.identity(4, dtype=float), newLocalTarget)

    spinCommand = self.ccwSpinPID.calculate(ccwSpinErrorRadian)
    if not isClose:
      forwardCommand = self.forwardPID.calculate(forwardErrorMeter)
    else:
      forwardCommand = self.forwardPIDNearby.calculate(forwardErrorMeter)
    self.pAndC.actuation.reset()
    self.pAndC.actuation.spinCounterClockWise(spinCommand)
    self.pAndC.actuation.goForward(forwardCommand)
    # self.pAndC.log(f"errors: {forwardErrorMeter}, {ccwSpinErrorRadian}")
    # self.pAndC.log(f"motor cammands: {forwardCommand}, {spinCommand}")
    self.pAndC.motorCmdsPub.publish(timestamp, self.pAndC.actuation.generateMotorCmd())
    return self

class CloseArmMission(Mission):
  def __init__(self, pAndC, initTimestamp):
    super().__init__(pAndC, initTimestamp)
  
  def tick(self, timestamp):
    self.pAndC.actuation.reset()
    if timestamp - self.initTimestamp < 1:
      # close arm for 1 second
      self.pAndC.actuation.clawCommand(32)
      self.pAndC.motorCmdsPub.publish(timestamp, self.pAndC.actuation.generateMotorCmd())
      return self
    else:
      self.pAndC.motorCmdsPub.publish(timestamp, self.pAndC.actuation.generateMotorCmd())
      return MoveArmUpMission(self.pAndC, timestamp)

class MoveArmUpMission(Mission):
  def __init__(self, pAndC, initTimestamp):
    super().__init__(pAndC, initTimestamp)
  
  def tick(self, timestamp):
    self.pAndC.actuation.reset()
    if timestamp - self.initTimestamp < 2.5:
      # close arm for 1 second
      self.pAndC.actuation.armCommand(42)
      self.pAndC.motorCmdsPub.publish(timestamp, self.pAndC.actuation.generateMotorCmd())
      return self
    else:
      self.pAndC.motorCmdsPub.publish(timestamp, self.pAndC.actuation.generateMotorCmd())
      return ExploreForLocalizationMission(self.pAndC, timestamp)

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
    if self.pAndC.latestRobotPoseTimestamp is None or \
      timestamp - self.pAndC.latestRobotPoseTimestamp > 1:
      self.pAndC.actuation.spinCounterClockWise(Constants.ExploreRotation)
      self.pAndC.motorCmdsPub.publish(timestamp, self.pAndC.actuation.generateMotorCmd())
      return self

    else:
      # stop the motors
      self.pAndC.motorCmdsPub.publish(timestamp, self.pAndC.actuation.generateMotorCmd())
      # and return the Mission.
      # TODO: choose the next mission properly
      return GlobalGoToMission(self.pAndC, timestamp)

class GlobalGoToMission(Mission):
  def __init__(self, pAndC, initTimestamp):
    super().__init__(pAndC, initTimestamp)
    self.forwardPID = PID(kp=Constants.LOCAL_CONTROLLER_FORWARD_KP,
                          ki=0,
                          kd=0)
    self.forwardPIDNearby = PID(kp=Constants.LOCAL_CONTROLLER_FORWARD_KP,
                          ki=Constants.LOCAL_CONTROLLER_FORWARD_KI,
                          kd=0)
    self.ccwSpinPID = PID(kp=Constants.LOCAL_CONTROLLER_ROTATIONAL_KP,
                          ki=0,
                          kd=0)
  
  def tick(self, timestamp):
    self.pAndC.actuation.reset()

    # set the target position
    robotPose = self.pAndC.latestRobotPose.copy()
    robotPosition2d = robotPose[:2, 3]
    globalTarget = robotPose[:2, 3].copy()
    globalTarget[0] = 1.0
    #- Config.zero_offset
    errorPosition2D = globalTarget - robotPosition2d
    if np.linalg.norm(errorPosition2D) < 0.1:
      print("robotPose", robotPose)
      print("errorPosition2D:", errorPosition2D)
      self.pAndC.motorCmdsPub.publish(timestamp, self.pAndC.actuation.generateMotorCmd())
      self.pAndC.log("LocalGoToMission: Target reached!")
      return NoMission(self.pAndC, timestamp)

    newTarget, isClose = calculateTarget_LookAhead(robotPosition2d, globalTarget, 0.2)
    forwardErrorMeter, ccwSpinErrorRadian = calculateLinearAndRotationalError(robotPose, newTarget)

    spinCommand = self.ccwSpinPID.calculate(ccwSpinErrorRadian)
    if not isClose:
      forwardCommand = self.forwardPID.calculate(forwardErrorMeter)
    else:
      forwardCommand = self.forwardPIDNearby.calculate(forwardErrorMeter)
    self.pAndC.actuation.reset()
    self.pAndC.actuation.spinCounterClockWise(spinCommand)
    self.pAndC.actuation.goForward(forwardCommand)
    self.pAndC.log(f"errors: {forwardErrorMeter}, {ccwSpinErrorRadian}")
    self.pAndC.log(f"motor cammands: {forwardCommand}, {spinCommand}")
    self.pAndC.motorCmdsPub.publish(timestamp, self.pAndC.actuation.generateMotorCmd())
    return self

class PlanningAndControlNode(Node):
  def __init__(self):
    super().__init__("PlanningAndControlNode")
    self.motorCmdsPub = self.create_publisher(Topics.motorCommands)
    self.ballPositionSub = self.create_subscriber(Topics.ballPositions, self.ballPosition_cb)
    self.localizationSub = self.create_subscriber(Topics.fusedPose, self.localization_cb)
    self.sensorsSub = self.create_subscriber(Topics.sensors, self.sensors_cb)
    self.tickSub = self.create_subscriber(Topics.isMoving, self.tick)
    self.actuation = None
    self.latestBallPositionsInMapFrame = None
    self.latestBallPositionsInRobotFrame = None
    self.latestBallPositionsTimestamp = None
    self.latestRobotPose = None
    self.latestRobotPoseTimestamp = None
    self.ballPositionsInMap = None
    self.currentMission = None
    self.sensors = None
    self.latestSensorsTimestamp = None
    self.log = None
  
  def ballPosition_cb(self, timestamp, ballPositions):
    self.latestBallPositionsInRobotFrame = ballPositions
    self.latestBallPositionsTimestamp = timestamp

  def localization_cb(self, timestamp, localization):
    self.latestRobotPose = localization
    self.latestRobotPoseTimestamp = timestamp
    if not self.latestBallPositionsInRobotFrame is None:
      self.ballPositionsInMap = self.latestRobotPose @ self.latestBallPositionsInRobotFrame
  
  def sensors_cb(self, timestamp, sensors):
    self.latestSensorsTimestamp = timestamp
    self.sensors = sensors

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
      # self.currentMission = ExploreForTennisBallsMission(self, timestamp)
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
  generateLocalizationNodes(withGraph = True)
  perceptionNode = PerceptionNode()
  start_subscribers()
  

  while True:
    robot.update()

    sensorPublisherNode.publishSensorData(robot)


if __name__ == "__main__":
  import multiprocessing as mp
  mp.set_start_method('spawn', force=True)
  test_planning_and_control_node()