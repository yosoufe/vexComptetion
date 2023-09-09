class Actuation:
  RIGHT_MOTOR_INDEX = 0
  LEFT_MOTOR_INDEX = 9
  CLAW_MOTOR_INDEX = 7
  ARM_MOTOR_INDEX = 8

  def __init__(self, robot):
    self.robot = robot
    self.reset()

  def reset(self):
    self.rightMotor = 0
    self.leftMotor = 0
    self.clawMotor = 0
    self.armMotor = 0

  def goForward(self, command):
    self.rightMotor += command
    self.leftMotor -= command

  def spinCounterClockWise(self, command):
    self.rightMotor -= command
    self.leftMotor -= command

  def armCommand(self, command):
    self.armMotor = command

  def clawCommand(self, command):
    self.clawMotor = command

  def apply(self):
    self.robot.motor[Actuation.RIGHT_MOTOR_INDEX] = self.rightMotor
    self.robot.motor[Actuation.LEFT_MOTOR_INDEX] = self.leftMotor
    self.robot.motor[Actuation.CLAW_MOTOR_INDEX] = self.clawMotor
    self.robot.motor[Actuation.ARM_MOTOR_INDEX] = self.armMotor

class ManualControl:
  def __init__(self, robot):
    self.robot = robot
    self.actuation = Actuation(robot)
  
  def run(self):
    self.actuation.reset()
    self.actuation.goForward((self.robot.keys["w"] - self.robot.keys["s"])* 30)
    self.actuation.spinCounterClockWise((self.robot.keys["a"] - self.robot.keys["d"])*25)
    arm = self.robot.keys["r"] - self.robot.keys["f"]
    if arm == 0:
      self.actuation.armCommand(10)
    elif arm == 1:
      self.actuation.armCommand(42)
    else:
      self.actuation.armCommand(-1)

    self.actuation.clawCommand((self.robot.keys["q"] - self.robot.keys["e"])*42)
    self.actuation.apply()

if __name__ == "__main__":
  from config import Config
  from cortano import RemoteInterface
  robot = RemoteInterface(Config.ip)
  manualControl = ManualControl(robot)
  while True:
    robot.update()
    manualControl.run()