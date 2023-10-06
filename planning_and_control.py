from middleware import Node
from constants import Topics, Config, Map
from abc import ABC, abstractmethod
from utils import LogOnlyChange, calculateLinearAndRotationalError, calculateTarget_LookAhead
from pid import PID
import numpy as np

# current Mission orders
# ExploreForTennisBallsMission
# LocalGoToMission
# CloseClawMission or ExploreForTennisBallsMission(if ball lost)
# MoveArmUpMission
# ExploreForLocalizationMission
# GlobalGoToMission # the other side
# OpenClawMission 
# MoveBackMission
# MoveArmDownMission
# ExploreForTennisBallsMission (loop again)

class CompetitionConstant:
  ExploreRotation = 25
  LOCAL_CONTROLLER_FORWARD_KP = 85
  LOCAL_CONTROLLER_FORWARD_KI = 0.1
  LOCAL_CONTROLLER_ROTATIONAL_KP = 35

class HomeConstants:
  ExploreRotation = 20
  LOCAL_CONTROLLER_FORWARD_KP = 80
  LOCAL_CONTROLLER_FORWARD_KI = 0.3
  LOCAL_CONTROLLER_ROTATIONAL_KP = 30

# Constants = CompetitionConstant
Constants = HomeConstants


class Mission(ABC):
  def __init__(self, pAndC, initTimestamp):
    super().__init__()
    self.pAndC = pAndC
    self.initTimestamp = initTimestamp
  
  @abstractmethod
  def tick(self, timestamp) -> "Mission":
    pass

class TimedMission(Mission):
  def __init__(self, pAndC, initTimestamp):
    super().__init__(pAndC, initTimestamp)
  
  def timedLoopContinue(self, timestamp, duration):
    if timestamp - self.initTimestamp < duration:
      return True
    else:
      return False

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
    self.pAndC.enablePerception()
    if self.pAndC.latestBallPositionsTimestamp and \
        self.pAndC.latestBallPositionsTimestamp > self.initTimestamp:

      self.initTimestamp = timestamp
      if self.conditionMetCounter > -1 :
        # stop the robot
        self.pAndC.motorCmdsPub.publish(
            timestamp, self.pAndC.actuation.generateMotorCmd())
        self.conditionMetCounter = self.conditionMetCounter + 1
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
      self.pAndC.log(f"LocalGoToMission: localTarget: {localTarget}")
      self.pAndC.motorCmdsPub.publish(timestamp, self.pAndC.actuation.generateMotorCmd())
      self.pAndC.disablePerception()
      self.pAndC.log("LocalGoToMission: Target reached!")
      return CloseClawMission(self.pAndC, timestamp)

    newLocalTarget, isClose = calculateTarget_LookAhead(np.zeros_like(localTarget), localTarget, 0.2)
    forwardErrorMeter, ccwSpinErrorRadian = calculateLinearAndRotationalError(np.identity(4, dtype=float), newLocalTarget)

    spinCommand = self.ccwSpinPID.calculate(ccwSpinErrorRadian)
    
    self.pAndC.log(f"LocalGoToMission: isClose {isClose}")
    if not isClose:
      forwardCommand = self.forwardPID.calculate(forwardErrorMeter)
    else:
      forwardCommand = self.forwardPIDNearby.calculate(forwardErrorMeter)
    self.pAndC.actuation.reset()
    self.pAndC.actuation.spinCounterClockWise(spinCommand)
    self.pAndC.actuation.goForward(forwardCommand)
    # self.pAndC.log(f"errors: {forwardErrorMeter}, {ccwSpinErrorRadian}")
    # self.pAndC.log(f"motor commands: {forwardCommand}, {spinCommand}")
    self.pAndC.motorCmdsPub.publish(timestamp, self.pAndC.actuation.generateMotorCmd())
    return self


class CloseClawMission(TimedMission):
  def __init__(self, pAndC, initTimestamp):
    super().__init__(pAndC, initTimestamp)
  
  def tick(self, timestamp):
    self.pAndC.actuation.reset()
    if self.timedLoopContinue(timestamp, duration=1):
      # close arm for 1 second
      self.pAndC.actuation.clawCommand(32)
      self.pAndC.motorCmdsPub.publish(timestamp, self.pAndC.actuation.generateMotorCmd())
      return self
    else:
      self.pAndC.motorCmdsPub.publish(timestamp, self.pAndC.actuation.generateMotorCmd())
      return MoveArmUpMission(self.pAndC, timestamp)

class MoveArmUpMission(TimedMission):
  def __init__(self, pAndC, initTimestamp):
    super().__init__(pAndC, initTimestamp)
  
  def tick(self, timestamp):
    self.pAndC.actuation.reset()
    if self.timedLoopContinue(timestamp, duration=3):
      # Move arm up for 1
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
    self.timestamp_rec = initTimestamp
    self.isSpinning = False

  def tick(self, timestamp):
    self.pAndC.actuation.reset()
    if self.pAndC.latestRobotPoseTimestamp is None or \
      timestamp - self.pAndC.latestAtPoseTimestamp > 1:
      
      if self.isSpinning:
        if timestamp - self.timestamp_rec < 1:
          self.pAndC.actuation.spinCounterClockWise(Constants.ExploreRotation)
          self.pAndC.motorCmdsPub.publish(timestamp, self.pAndC.actuation.generateMotorCmd())
        else:
          self.timestamp_rec = timestamp
          self.isSpinning = False
      else:
        if timestamp - self.timestamp_rec < 1:
          # do not spin
          self.pAndC.motorCmdsPub.publish(timestamp, self.pAndC.actuation.generateMotorCmd())
        else:
          self.timestamp_rec = timestamp
          self.isSpinning = True
      
      return self

    else:
      # stop the motors
      self.pAndC.motorCmdsPub.publish(timestamp, self.pAndC.actuation.generateMotorCmd())
      return GlobalGoToMission(self.pAndC, timestamp)

class GlobalGoToMission(Mission):
  def __init__(self, pAndC, initTimestamp):
    super().__init__(pAndC, initTimestamp)
    self.positionKp = 30
    self.orientationKp = 5
    self.positionKi = 0.5
    self.orientationKi = 0.05
    self.integralPosition = np.zeros(shape=(2,), dtype=float)
    self.integralOrientation = np.zeros(shape=(2,), dtype=float)
  
  def tick(self, timestamp):
    self.pAndC.actuation.reset()

    # set the target position
    robotPose = self.pAndC.latestRobotPose.copy()
    robotPosition2d = robotPose[:2, 3]
    robotOri = robotPose[:2, 0]
    targetPosition = robotPose[:2, 3].copy()
    # at competition
    # at home
    # targetPosition[0] = 0.5
    # targetPosition[1] = -1.5
    # targetOri = np.array([0, -1], dtype=float)
    targetPosition[0] = -0.15
    targetPosition[1] = -1.25
    targetOri = np.array([-1, 0], dtype=float)
    # self.pAndC.targetPosePub.publish(timestamp, targetPosition)
    #- Config.zero_offset
    errorPosition2D = targetPosition - robotPosition2d
    if np.linalg.norm(errorPosition2D) < 0.1:
      print("robotPose", robotPose)
      print("errorPosition2D:", errorPosition2D)
      self.pAndC.motorCmdsPub.publish(timestamp, self.pAndC.actuation.generateMotorCmd())
      self.pAndC.log("LocalGoToMission: Target reached!")
      return OpenClawMission(self.pAndC, timestamp)

    newTarget, isClose = calculateTarget_LookAhead(robotPosition2d, targetPosition, 0.5)
    self.pAndC.targetPosePub.publish(timestamp, newTarget)
    self.pAndC.globalTargetPub.publish(timestamp, targetPosition)
    self.pAndC.log(f"GlobalGoToMission: isClose {isClose}")
    u_control = np.ones((3,1), dtype=float)
    positionError = targetPosition - robotPosition2d
    orientationError = targetOri - robotOri
    u_control[:2, 0] = self.positionKp * positionError + self.orientationKp * orientationError
    if isClose:
      self.integralPosition = self.integralPosition + positionError
      self.integralOrientation = self.integralOrientation + orientationError
      integratTerms = self.positionKi * self.integralPosition + self.orientationKi * self.integralOrientation
      print("integratTerms", integratTerms)
      u_control[:2, 0] = u_control[:2, 0] + integratTerms

    u_control = (np.transpose(robotPose[:3,:3]) @ u_control).squeeze()
    forwardCommand = u_control[0]
    spinCommand = max(min(u_control[1], Constants.ExploreRotation), -1 * Constants.ExploreRotation)

    self.pAndC.actuation.reset()
    self.pAndC.actuation.spinCounterClockWise(spinCommand)
    self.pAndC.actuation.goForward(forwardCommand)
    self.pAndC.log(f"motor commands: {forwardCommand}, {spinCommand}")
    self.pAndC.motorCmdsPub.publish(timestamp, self.pAndC.actuation.generateMotorCmd())
    return self

class OpenClawMission(TimedMission):
  def __init__(self, pAndC, initTimestamp):
    super().__init__(pAndC, initTimestamp)
  
  def tick(self, timestamp):
    self.pAndC.actuation.reset()
    if self.timedLoopContinue(timestamp, duration=1):
      # open arm for 1 second
      self.pAndC.actuation.clawCommand(-32)
      self.pAndC.motorCmdsPub.publish(timestamp, self.pAndC.actuation.generateMotorCmd())
      return self
    else:
      self.pAndC.motorCmdsPub.publish(timestamp, self.pAndC.actuation.generateMotorCmd())
      return MoveBackMission(self.pAndC, timestamp)

class MoveBackMission(TimedMission):
  def __init__(self, pAndC, initTimestamp):
    super().__init__(pAndC, initTimestamp)
  
  def tick(self, timestamp):
    self.pAndC.actuation.reset()
    if self.timedLoopContinue(timestamp, duration=1):
      # go backward for 1 sec
      self.pAndC.actuation.goForward(-30)
      self.pAndC.motorCmdsPub.publish(timestamp, self.pAndC.actuation.generateMotorCmd())
      return self
    else:
      self.pAndC.motorCmdsPub.publish(timestamp, self.pAndC.actuation.generateMotorCmd())
      return MoveArmDownMission(self.pAndC, timestamp)

class MoveArmDownMission(TimedMission):
  def __init__(self, pAndC, initTimestamp):
    super().__init__(pAndC, initTimestamp)
  
  def tick(self, timestamp):
    self.pAndC.actuation.reset()
    if self.timedLoopContinue(timestamp, duration=1.5):
      # Move arm down for 1
      self.pAndC.actuation.armCommand(-24)
      self.pAndC.motorCmdsPub.publish(timestamp, self.pAndC.actuation.generateMotorCmd())
      return self
    else:
      self.pAndC.motorCmdsPub.publish(timestamp, self.pAndC.actuation.generateMotorCmd())
      return ExploreForTennisBallsMission(self.pAndC, timestamp)

class PlanningAndControlNode(Node):
  def __init__(self):
    super().__init__("PlanningAndControlNode")
    self.motorCmdsPub = self.create_publisher(Topics.motorCommands)
    self.ballPositionSub = self.create_subscriber(Topics.ballPositions, self.ballPosition_cb)
    self.localizationSub = self.create_subscriber(Topics.fusedPose, self.localization_cb)
    self.sensorsSub = self.create_subscriber(Topics.sensors, self.sensors_cb)
    self.tickSub = self.create_subscriber(Topics.isMoving, self.tick)
    self.switchPerceptionPub = self.create_publisher(Topics.switchPerception)
    self.targetPosePub = self.create_publisher(Topics.targetPose)
    self.globalTargetPub = self.create_publisher(Topics.globalTarget)
    self.actuation = None
    self.latestBallPositionsInMapFrame = None
    self.latestBallPositionsInRobotFrame = None
    self.latestBallPositionsTimestamp = None
    self.latestRobotPose = None
    self.latestRobotPoseTimestamp = None
    self.latestAtPoseTimestamp = None
    self.ballPositionsInMap = None
    self.currentMission = None
    self.sensors = None
    self.latestSensorsTimestamp = None
    self.log = None
  
  def ballPosition_cb(self, timestamp, ballPositions):
    self.latestBallPositionsInRobotFrame = ballPositions
    self.latestBallPositionsTimestamp = timestamp

  def localization_cb(self, timestamp, localization):
    self.latestAtPoseTimestamp, self.latestRobotPose = localization
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
      self.currentMission = ExploreForTennisBallsMission(self, timestamp)
      # self.currentMission = ExploreForLocalizationMission(self, timestamp)

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

  def enablePerception(self):
    self.switchPerceptionPub.publish(None, True)
  
  def disablePerception(self):
    self.switchPerceptionPub.publish(None, False)

class PlotterNode(Node):
  def __init__(self):
    super().__init__("PlotterNode")
    from constants import Map, Topics
    self.poseSub = self.create_subscriber(Topics.fusedPose, self.pose_cb)
    self.targetPoseSub = self.create_subscriber(Topics.targetPose, self.targetPose_cb)
    self.targetPoseSub = self.create_subscriber(Topics.globalTarget, self.globalTarget_cb)
    self.first_time = True
    self.robotPose = None
    self.targetPose = None
    self.globalTarget = None
  
  def pose_cb(self, timestamp, msg):
    self.robotPose = (timestamp, msg)
    self.updatePlot(timestamp)
  
  def targetPose_cb(self, timestamp, msg):
    self.targetPose = (timestamp, msg)
    self.updatePlot(timestamp)
  
  def globalTarget_cb(self, timestamp, msg):
    self.globalTarget = (timestamp, msg)
    self.updatePlot(timestamp)

  def updatePlot(self, timestamp):
    import matplotlib.pyplot as plt
    if self.first_time:
      self.fig = plt.figure(figsize=(
        (Map.X_limits[1]-Map.X_limits[0])*5,
        (Map.Y_limits[1]-Map.Y_limits[0])*5))
      self.ax = self.fig.add_subplot(1,1,1)
      plt.ion()
      plt.show()
      self.first_time = False
    
    self.ax.clear()
    
    # plot robot position
    robotPositionAndTimestamp = self.getIfNotExpired(self.robotPose, 1.0, timestamp)
    if not robotPositionAndTimestamp is None:
      _, robotPosition = robotPositionAndTimestamp
      position = robotPosition[:2, 3]
      direction = robotPosition[:2, 0] * 0.2
      self.ax.arrow(position[0], position[1], direction[0], direction[1], width=0.01)
    
    # plot target position
    targetPosition = self.getIfNotExpired(self.targetPose, float('inf'), timestamp)
    if not targetPosition is None:
      circle1 = plt.Circle((targetPosition[0], targetPosition[1]), 0.02, color="r")
      self.ax.add_patch(circle1)
    
    if not self.globalTarget is None:
      _, globalTar = self.globalTarget
      circle2 = plt.Circle((globalTar[0], globalTar[1]), 0.02, color="b")
      self.ax.add_patch(circle2)
    
    self.ax.set_xlim(Map.X_limits)
    self.ax.set_ylim(Map.Y_limits)
    self.fig.canvas.draw()
    self.fig.canvas.flush_events()

  def getIfNotExpired(self, msg, expiration, timestamp):
    if msg is None:
      return None
    
    msg_timestamp, content = msg
    if timestamp - msg_timestamp > expiration:
      return None
    
    return content

def test_planning_and_control_node():
  from constants import Config
  from perception import PerceptionNode
  from localization import generateLocalizationNodes
  from robot_interface_node import SensorPublisherNode
  from middleware import start_subscribers

  robot = Config.getRobot()
  sensorPublisherNode = SensorPublisherNode()
  perceptionNode = PerceptionNode()
  generateLocalizationNodes(withGraph = True)
  planningAndControlNode = PlanningAndControlNode()
  start_subscribers()
  

  while True:
    robot.update()

    sensorPublisherNode.publishSensorData(robot)


if __name__ == "__main__":
  import multiprocessing as mp
  mp.set_start_method('spawn', force=True)
  test_planning_and_control_node()