from cortano import RemoteInterface
from pid import PID
from actuation import Actuation, ManualControl
from perception import Perception
import numpy as np

# cameraToRobotT = np.array(
#   [[0.04892364, -0.41772631, 0.90725477, -0.32358759],
#    [-0.99831263, -0.0435315, 0.03379077, 0.09919108],
#    [0.02448331, -0.89973047, -0.43575871, 0.24580773],
#    [0., 0., 0., 1.]],
#   dtype=float
# )

cameraToRobotT = np.array(
  [[ 0.04183374 ,-0.40959374 , 0.91130835 ,-0.33556501],
   [-0.99828073 ,-0.05146286,  0.02269589,  0.10270806],
   [ 0.03691244 ,-0.90393521 ,-0.42607349 , 0.2442368 ],
   [ 0.          ,0.         , 0.         , 1.        ]],
  dtype=float
)

class TennisBallGraber:
  def __init__(self):
    self.perception = Perception()
    self.robot = RemoteInterface("192.168.68.68")
    self.actuation = Actuation(self.robot)
    # self.manualControl = ManualControl(self.robot) # for debugging and testing
    self.forwardPID = PID(kp=50, ki=3, kd=1)
    self.ccwSpinPID = PID(kp=10, ki=0, kd=0)

  def loop(self):
    while True:
      self.robot.update()
      self.run()
  

  def run(self):
    # self.manualControl.run()
    color, depth, _ = self.robot.read()
    # detect tennis ball
    tennisBalls = self.perception.detect_balls(color, depth)
    n_balls = len(tennisBalls)
    if (n_balls > 0):
      # convert the position to chassis frame
      tennisBalls = np.vstack(tennisBalls).T/1000.0 # 3 x n_balls
      positions = np.ones((4,n_balls), float)
      positions[:3, :] = tennisBalls
      positionInRobotFrame = np.sort(cameraToRobotT @ positions, axis = 1)
      # print(positionInRobotFrame[:3, :])

      # navigate the robot to that position
      target = positionInRobotFrame[:2, 0]
      self.runControl(target)
    else:
      self.actuation.reset()
      self.actuation.apply()

    
  def runControl(self, target):
    x = target[0]
    y = target[1]
    ccwSpinErrorDegree = np.arctan2(y,x)
    forwardErrorMeter = x
    spinCommand = self.ccwSpinPID.calculate(ccwSpinErrorDegree)
    forwardCommand = self.forwardPID.calculate(forwardErrorMeter)
    print("errors:", forwardErrorMeter, ccwSpinErrorDegree)
    print("Commands:", forwardCommand, spinCommand)
    self.actuation.reset()
    self.actuation.spinCounterClockWise(spinCommand)
    self.actuation.goForward(forwardCommand)
    self.actuation.apply()

if __name__ == "__main__":
  app = TennisBallGraber()
  app.loop()
