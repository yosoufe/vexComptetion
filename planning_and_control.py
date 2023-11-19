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
  ExploreRotation = 35
  LOCAL_CONTROLLER_FORWARD_KP = 90
  LOCAL_CONTROLLER_FORWARD_KI = 0.5
  LOCAL_CONTROLLER_ROTATIONAL_KP = 110
  LOOK_AHEAD_DISTANCE = 0.5
  MAX_ROTATION = 120

class HomeConstants:
  ExploreRotation = 20
  LOCAL_CONTROLLER_FORWARD_KP = 80
  LOCAL_CONTROLLER_FORWARD_KI = 0.3
  LOCAL_CONTROLLER_ROTATIONAL_KP = 30
  LOOK_AHEAD_DISTANCE = 0.5
  MAX_ROTATION = 100


Constants = CompetitionConstant
# Constants = HomeConstants


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
    if not self.pAndC.isPerceptionReady:
      self.pAndC.motorCmdsPub.publish(timestamp, self.pAndC.actuation.generateMotorCmd())
      return self
    
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
    
    self.pAndC.log(f"ExploreForTennisBallsMission: {self.conditionMetCounter}, {self.pAndC.latestBallPositionsTimestamp}, {self.initTimestamp}")

    self.pAndC.actuation.spinCounterClockWise(Constants.ExploreRotation)
    self.pAndC.motorCmdsPub.publish(timestamp, self.pAndC.actuation.generateMotorCmd())
    return self


class LocalController:
  def __init__(self):
    self.pController = PID(kp=np.array([Constants.LOCAL_CONTROLLER_FORWARD_KP,
                                        Constants.LOCAL_CONTROLLER_ROTATIONAL_KP], dtype=float),
                           ki=np.zeros((2,), dtype= float),
                           kd=np.zeros((2,), dtype= float))
    
    self.piController = PID(kp=np.array([Constants.LOCAL_CONTROLLER_FORWARD_KP,
                                         Constants.LOCAL_CONTROLLER_ROTATIONAL_KP], dtype=float),
                            ki=np.array([Constants.LOCAL_CONTROLLER_FORWARD_KI,
                                         0], dtype=float),
                            kd=np.zeros((2,), dtype= float))
  
  def calculate(self, currentPose, targetPosition2D):
    """
    :param currentPose: Pose of the robot at current Location
    :type currentPose: 4x4 numpy array
    :param targetPosition2D: target location
    :type targetPosition2D:  vector in shape of (2,)
    :return: acceleration vector in shape of (2,) in the robot frame
    """
    # calculate target in local frame
    homTarget = np.ones((4,1), dtype=float)
    homTarget[:2, 0] = targetPosition2D
    localTarget2D = (np.linalg.inv(currentPose) @ homTarget)[:2]
    # generate intermediate target
    intermediateTargetLocal, isClose = calculateTarget_LookAhead(np.zeros_like(localTarget2D), localTarget2D, lookAheadDistance = 0.3)
    inter = np.ones((4,1), dtype=float)
    inter[:2, 0] = intermediateTargetLocal
    intermediateTargetGlobal = currentPose @ inter

    intermediateTargetLocal = np.squeeze(intermediateTargetLocal)
    intermediateTargetGlobal = np.squeeze(intermediateTargetGlobal)
    # print("localTarget2D",np.squeeze(localTarget2D))
    # print("localTarget2D",np.squeeze(localTarget2D), "intermediateTargetLocal", intermediateTargetLocal[:2], isClose)
    # print("intermediateTargetGlobal", intermediateTargetGlobal[:2])
    # print(isClose)

      
    if isClose:
      return self.piController.calculate(intermediateTargetLocal), intermediateTargetGlobal[:2]
    else:
      return self.pController.calculate(intermediateTargetLocal), intermediateTargetGlobal[:2]

class LocalGoToMission(Mission):
  def __init__(self, pAndC, initTimestamp):
    super().__init__(pAndC, initTimestamp)
    self.controller = LocalController()
  
  def tick(self, timestamp):
    self.pAndC.actuation.reset()

    # The latest seen ball should be seen recently (not later than 2 seconds ago)
    if abs(self.pAndC.latestBallPositionsTimestamp - timestamp) > 2:
      self.pAndC.motorCmdsPub.publish(timestamp, self.pAndC.actuation.generateMotorCmd())
      return ExploreForTennisBallsMission(self.pAndC, timestamp)

    # set the target position
    localTarget = self.pAndC.latestBallPositionsInRobotFrame[:2, 0] # - Config.zero_offset
    
    

    # Mission Done if we reached the target position
    # if np.linalg.norm(localTarget) < 0.025:
    if np.all(np.absolute(localTarget) < np.array([0.01, 0.03])):
      self.pAndC.log(f"LocalGoToMission: Target reached! localTarget: {localTarget}, distance: {np.linalg.norm(localTarget)}")
      self.pAndC.motorCmdsPub.publish(timestamp, self.pAndC.actuation.generateMotorCmd())
      self.pAndC.disablePerception()
      return CloseClawMission(self.pAndC, timestamp)
      # return self
    # else:
    #   self.pAndC.log(f"LocalGoToMission: localTarget: {localTarget}, distance: {np.linalg.norm(localTarget)}")

    # run the controller
    u_control, newTarget = self.controller.calculate(np.identity(4), localTarget)

    # print("LocalGoToMission, newTarget:",newTarget, np.linalg.norm(newTarget))
    # print("LocalGoToMission, u_control:",u_control)

    self.pAndC.actuation.reset()
    self.pAndC.actuation.goForward(u_control[0])
    self.pAndC.actuation.spinCounterClockWise(u_control[1])
    self.pAndC.motorCmdsPub.publish(timestamp, self.pAndC.actuation.generateMotorCmd())
    return self


class CloseClawMission(TimedMission):
  def __init__(self, pAndC, initTimestamp):
    super().__init__(pAndC, initTimestamp)
  
  def tick(self, timestamp):
    self.pAndC.actuation.reset()
    if self.timedLoopContinue(timestamp, duration=1.5):
      # close arm for 1 second
      self.pAndC.actuation.clawCommand(52)
      self.pAndC.motorCmdsPub.publish(timestamp, self.pAndC.actuation.generateMotorCmd())
      return self
    else:
      self.pAndC.motorCmdsPub.publish(timestamp, self.pAndC.actuation.generateMotorCmd())
      return MoveBackMission(self.pAndC, timestamp, MoveArmUpMission)

class MoveArmUpMission(TimedMission):
  def __init__(self, pAndC, initTimestamp):
    super().__init__(pAndC, initTimestamp)
  
  def tick(self, timestamp):
    self.pAndC.actuation.reset()
    if self.timedLoopContinue(timestamp, duration=2.2):
      # Move arm up for 1
      self.pAndC.actuation.armCommand(52)
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
  class FirstPhasePosition(Mission):
    def __init__(self, pAndC, mainMission, initTimestamp):
      super().__init__(pAndC, initTimestamp)
      self.mainMission = mainMission
      self.controller = LocalController()
    
    def tick(self, timestamp) -> Mission:
      self.pAndC.actuation.reset()

      # set the current robot pose and target position
      robotPose = self.pAndC.latestRobotPose.copy() @ Config.R2toRT()
      robotPosition2d = robotPose[:2, 3]
      targetPosition2D = self.chooseTargetPositionPhase1()
      errorPosition2D = targetPosition2D - robotPosition2d

      # Phase done, target reached
      if np.linalg.norm(errorPosition2D) < 0.1:
        self.pAndC.motorCmdsPub.publish(timestamp, self.pAndC.actuation.generateMotorCmd())
        return GlobalGoToMission.SecondPhaseRotation(self.pAndC, self.mainMission, timestamp)
      
      
      u_control, newTarget = self.controller.calculate(robotPose, targetPosition2D)
      self.pAndC.targetPosePub.publish(timestamp, newTarget)
      self.pAndC.globalTargetPub.publish(timestamp, targetPosition2D)

      print("FirstPhasePosition errorDis", np.linalg.norm(errorPosition2D), u_control)
      
      forwardCommand = u_control[0]
      spinCommand = max(min(u_control[1], Constants.MAX_ROTATION), -1 * Constants.MAX_ROTATION)

      self.pAndC.actuation.goForward(forwardCommand)
      self.pAndC.actuation.spinCounterClockWise(spinCommand)
      self.pAndC.motorCmdsPub.publish(timestamp, self.pAndC.actuation.generateMotorCmd())
      return self
    
    def chooseTargetPositionPhase1(self):
      globalTargetPosition, targetOrientation = self.mainMission.chooseTargetPose()
      alpha = 0.3 # should be larger than Config.DISTNACE_OFFSET_X
      phase1TargetPosition = globalTargetPosition - alpha * targetOrientation[:, 0]
      return phase1TargetPosition

  class SecondPhaseRotation(Mission):
    def __init__(self, pAndC, mainMission, initTimestamp):
      super().__init__(pAndC, initTimestamp)
      self.mainMission = mainMission
      self.piController = PID(kp=65, ki=10, kd=0,
                              output_limit=np.array(
                                  [-Constants.MAX_ROTATION, Constants.MAX_ROTATION]
                              ))
    
    def tick(self, timestamp):
      self.pAndC.actuation.reset()

      # calculate heading error
      robotPose = self.pAndC.latestRobotPose.copy()
      headingErrorRad = self.calculateHeadingErrorAngleRad(robotPose)

      # Mission is done if heading error is less than 5 degrees
      if abs(headingErrorRad) < np.deg2rad(5):
        self.pAndC.motorCmdsPub.publish(timestamp, self.pAndC.actuation.generateMotorCmd())
        return GlobalGoToMission.ThirdPhasePosition(self.pAndC, self.mainMission, timestamp)
      
      # run pi controller
      spinCommand = self.piController.calculate(headingErrorRad)
      
      # only rotate. Ignore the position command
      spinCommand = max(min(spinCommand, Constants.MAX_ROTATION), -1 * Constants.MAX_ROTATION)
      self.pAndC.log(f"headingErrorRad: {np.rad2deg(headingErrorRad)}, spinCommand: {spinCommand}")
      self.pAndC.actuation.spinCounterClockWise(spinCommand)
      self.pAndC.motorCmdsPub.publish(timestamp, self.pAndC.actuation.generateMotorCmd())
      return self

    def calculateHeadingErrorAngleRad(self, robotPose):
      targetHeadingVector = self.chooseTargetHeadingVector()
      # headingErrorVector = targetHeadingVector - robotPose[:2, 0]
      b = targetHeadingVector
      a = robotPose[:2, 0]
      cosTh1 = np.dot(a,b)/(np.linalg.norm(a)*np.linalg.norm(b))
      sinTh1 = np.cross(a,b)/(np.linalg.norm(a)*np.linalg.norm(b))
      return np.arctan2(sinTh1,cosTh1)

      
    def chooseTargetHeadingVector(self):
      _, targetOrientation = self.mainMission.chooseTargetPose()
      return targetOrientation[:2, 0]
  
  class ThirdPhasePosition(Mission):
    def __init__(self, pAndC, mainMission, initTimestamp):
      super().__init__(pAndC, initTimestamp)
      self.mainMission = mainMission
      self.controller = LocalController()
    
    def tick(self, timestamp) -> Mission:
      self.pAndC.actuation.reset()

      # Choose the target and set current position
      robotPose = self.pAndC.latestRobotPose.copy()
      robotPosition2d = robotPose[:2, 3]
      targetPosition2D, _ = self.mainMission.chooseTargetPose()
      errorPosition2D = targetPosition2D - robotPosition2d

      # Mission accomplished, target reached
      if np.linalg.norm(errorPosition2D) < 0.05:
        self.pAndC.motorCmdsPub.publish(timestamp, self.pAndC.actuation.generateMotorCmd())
        return None
      
      u_control, newTarget = self.controller.calculate(robotPose, targetPosition2D)
      self.pAndC.targetPosePub.publish(timestamp, newTarget)
      self.pAndC.globalTargetPub.publish(timestamp, targetPosition2D)
      self.pAndC.log(f"GlobalGoToMission, ThirdPhasePosition: {np.linalg.norm(errorPosition2D)}, control: {u_control}")
      
      forwardCommand = u_control[0]
      spinCommand = max(min(u_control[1], Constants.MAX_ROTATION), -1 * Constants.MAX_ROTATION)

      if (forwardCommand > 40):
        self.pAndC.motorCmdsPub.publish(timestamp, self.pAndC.actuation.generateMotorCmd())
        return None

      self.pAndC.actuation.spinCounterClockWise(spinCommand)
      self.pAndC.actuation.goForward(forwardCommand)
      # self.pAndC.log(f"motor commands: {forwardCommand}, {spinCommand}")
      self.pAndC.motorCmdsPub.publish(timestamp, self.pAndC.actuation.generateMotorCmd())
      return self
      

  def __init__(self, pAndC, initTimestamp):
    super().__init__(pAndC, initTimestamp)
    self.mission = None
    self.ATTagXPositions = np.zeros((6,), dtype=float)
    for idx in range(12,18):
      self.ATTagXPositions[idx-12] = Map.getLandmark(idx)[0,3]


  
  def tick(self, timestamp):
    if self.mission is None:
      self.mission = GlobalGoToMission.FirstPhasePosition(self.pAndC, self, timestamp)
    
    self.pAndC.log(f"Current Mission: GlobalGoToMission.{type(self.mission).__name__}")
    self.mission = self.mission.tick(timestamp)
    
    if self.mission == None:
      return OpenClawMission(self.pAndC, timestamp)
    else:
      return self
  
  def chooseTargetPose(self):
    # targetPosition = np.array([0.15, -1.25], dtype=float)
    # # targetXaxis = np.array([0, -1], dtype=float)
    # targetPosition = np.array([1, 1], dtype=float)
    # targetXaxis = np.array([1, 0], dtype=float)
    # targetPosition = np.array([(-36+3) * 0.0254, 2 * 0.0254], dtype=float)
    currentXPosition = self.pAndC.latestRobotPose.copy()[0, 3]
    # targetX = max(min(currentXPosition, Map.X_limits[1] - 0.3), Map.X_limits[0]+ 0.3)
    if Map.SIDE == "SOUTH":
      shifted = self.ATTagXPositions[:5] + 3 * 0.0254
      minIdx = np.argmin(np.absolute(shifted - currentXPosition))
      targetX = shifted[minIdx]
      targetPosition = np.array([targetX, 3 * 0.0254], dtype=float)
      targetXaxis = np.array([0, 1], dtype=float)
    elif Map.SIDE == "NORTH":
      shifted = self.ATTagXPositions[1:] - 3 * 0.0254
      minIdx = np.argmin(np.absolute(shifted - currentXPosition))
      targetX = shifted[minIdx]
      targetPosition = np.array([targetX, -3 * 0.0254], dtype=float)
      targetXaxis = np.array([0, -1], dtype=float)
    
    targetXaxis = targetXaxis / np.linalg.norm(targetXaxis)
    targetYaxis = np.array([-targetXaxis[1], targetXaxis[0]],dtype=float)
    targetRotation = np.zeros((2,2), dtype=float)
    targetRotation[:, 0] = targetXaxis
    targetRotation[:, 1] = targetYaxis
    return targetPosition, targetRotation

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
      return MoveBackMission(self.pAndC, timestamp, MoveArmDownMission)

class MoveBackMission(TimedMission):
  def __init__(self, pAndC, initTimestamp, nextSuccessMission):
    super().__init__(pAndC, initTimestamp)
    self.nextSuccessMission = nextSuccessMission
  
  def tick(self, timestamp):
    self.pAndC.actuation.reset()
    if self.timedLoopContinue(timestamp, duration=1):
      # go backward for 1 sec
      self.pAndC.actuation.goForward(-80)
      self.pAndC.motorCmdsPub.publish(timestamp, self.pAndC.actuation.generateMotorCmd())
      return self
    else:
      self.pAndC.motorCmdsPub.publish(timestamp, self.pAndC.actuation.generateMotorCmd())
      return self.nextSuccessMission(self.pAndC, timestamp)

class MoveArmDownMission(TimedMission):
  def __init__(self, pAndC, initTimestamp):
    super().__init__(pAndC, initTimestamp)
  
  def tick(self, timestamp):
    self.pAndC.actuation.reset()
    if self.timedLoopContinue(timestamp, duration=1.5):
      # Move arm down for 1
      self.pAndC.actuation.armCommand(-44)
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
    self.perceptionReadySub = self.create_subscriber(Topics.perceptionReady, self.perceptionReady_cb)
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
    self.isPerceptionReady = False

  def perceptionReady_cb(self, timestamp, msg):
    self.isPerceptionReady = msg
  
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
      LocalGoToMission
      self.currentMission = ExploreForTennisBallsMission(self, timestamp)
      # self.currentMission = LocalGoToMission(self, timestamp)
      # self.currentMission = ExploreForLocalizationMission(self, timestamp)
      # self.currentMission = MoveArmUpMission(self, timestamp)
    
    self.log(f"Current Mission: {type(self.currentMission).__name__}")
    nextMission = self.currentMission.tick(timestamp)
    self.currentMission = nextMission

    
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
    lim = np.array([-80 , 80], dtype = float) * 0.0254
    if self.first_time:
      self.fig = plt.figure(figsize=np.absolute(lim) * 5)
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
    targetPosition = self.getIfNotExpired(self.targetPose, 1.0, timestamp)
    if not targetPosition is None:
      circle1 = plt.Circle((targetPosition[0], targetPosition[1]), 0.02, color="r")
      self.ax.add_patch(circle1)
    
    if not self.globalTarget is None:
      _, globalTar = self.globalTarget
      circle2 = plt.Circle((globalTar[0], globalTar[1]), 0.02, color="b")
      self.ax.add_patch(circle2)
    
    
    self.ax.set_xlim(lim)
    self.ax.set_ylim(lim)
    self.ax.grid()
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