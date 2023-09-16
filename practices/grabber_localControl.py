from cortano import RemoteInterface
from pid import PID
from robot_interface_node import Actuation, ManualControl
from perception import Perception
import numpy as np
from constants import Config, Topics

class TennisBallGrabber:
  def __init__(self, robot, manual_control = False):
    self.isManualControl = manual_control
    self.perception = Perception()
    self.actuation = Actuation(robot)
    self.manualControl = ManualControl(robot)
    # self.forwardPID = PID(kp=50, ki=3, kd=1)
    self.forwardPID = PID(kp=50, ki=0, kd=0)
    self.ccwSpinPID = PID(kp=10, ki=0, kd=0) # 40 in the session
  

  def run(self, color, depth):
    if self.isManualControl:
      self.manualControl.run()
    # detect tennis ball
    tennisBalls = self.perception.detect_balls(color, depth)
    n_balls = len(tennisBalls)
    if (n_balls > 0):
      # convert the position to chassis frame
      tennisBalls = np.vstack(tennisBalls).T # 3 x n_balls
      positions = np.ones((4,n_balls), float)
      positions[:3, :] = tennisBalls
      positionInRobotFrame = np.sort(Config.cam2RobotT() @ positions, axis = 1)
      
      # navigate the robot to that position
      target = positionInRobotFrame[:2, 0] - Config.zero_offset
      if not self.isManualControl:
        self.runControl(target)
    else:
      if not self.isManualControl:
        self.actuation.reset()
        self.actuation.apply()

    
  def runControl(self, target):
    x = target[0]
    y = target[1]
    ccwSpinErrorRadian = np.arctan2(y,x)
    forwardErrorMeter = x
    spinCommand = self.ccwSpinPID.calculate(ccwSpinErrorRadian)
    forwardCommand = self.forwardPID.calculate(forwardErrorMeter) 
    print("target:", target)
    print("errors:", forwardErrorMeter, np.degrees(ccwSpinErrorRadian))
    print("Commands:", forwardCommand, spinCommand)
    # self.actuation.reset()
    # self.actuation.spinCounterClockWise(spinCommand)
    # self.actuation.goForward(forwardCommand)
    # self.actuation.apply()

if __name__ == "__main__":
  robot = RemoteInterface(Config.ip)
  app = TennisBallGrabber(robot, manual_control= False)
  while True:
    robot.update()
    color, depth , _ = robot.read()
    app.run(color, depth)