from middleware import Node
from constants import Topics

class Actuation:
  RIGHT_MOTOR_INDEX = 0
  LEFT_MOTOR_INDEX = 9
  CLAW_MOTOR_INDEX = 7
  ARM_MOTOR_INDEX = 8

  def __init__(self):
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

  def update(self, robot):
    robot.motor[Actuation.RIGHT_MOTOR_INDEX] = self.rightMotor
    robot.motor[Actuation.LEFT_MOTOR_INDEX] = self.leftMotor
    robot.motor[Actuation.CLAW_MOTOR_INDEX] = self.clawMotor
    robot.motor[Actuation.ARM_MOTOR_INDEX] = self.armMotor

class ManualControl:
  def __init__(self,):
    self.actuation = Actuation()
  
  def update(self, robot):
    self.actuation.reset()
    self.actuation.goForward((robot.keys["w"] - robot.keys["s"])* 30)
    self.actuation.spinCounterClockWise((robot.keys["a"] - robot.keys["d"])*25)
    arm = robot.keys["r"] - robot.keys["f"]
    if arm == 0:
      self.actuation.armCommand(10)
    elif arm == 1:
      self.actuation.armCommand(42)
    else:
      self.actuation.armCommand(-1)

    self.actuation.clawCommand((robot.keys["q"] - robot.keys["e"])*42)
    self.actuation.update(robot)

class SensorPublisherNode(Node):
  def __init__(self):
    super().__init__("SensorReader")
    self.rgbd_pub = self.create_publisher(Topics.rgbd)
    self.isMoving_pub = self.create_publisher(Topics.isMoving)

  def publishSensorData(self, robot):
    color, depth, sensors = robot.read()
    timestamp = sensors[0]
    self.rgbd_pub.publish(timestamp, (color, depth), block=False)
    isMoving = ((robot.motor[Actuation.RIGHT_MOTOR_INDEX] != 0) or (robot.motor[Actuation.LEFT_MOTOR_INDEX] != 0))
    self.isMoving_pub.publish(timestamp, isMoving)

class SensorReaderLoggerNode(Node):
  def __init__(self):
    super().__init__("SensorReaderLoggerNode")
    self.rgbd_sub = self.create_subscriber(Topics.rgbd, self.rgbd_cb)
    self.isMoving_sub = self.create_subscriber(Topics.isMoving, self.isMoving_cb)
  
  def rgbd_cb(self, timestamp, rgbd):
    print("SensorReaderLoggerNode rgbd_cb:")
    print(timestamp, rgbd)

  def isMoving_cb(self, timestamp, isMoving):
    print("SensorReaderLoggerNode isMoving_cb:")
    print(timestamp, isMoving)

def test_sensor_reader_node():
  from middleware import start_subscribers
  from constants import Config, Topics
  sensorReader = SensorPublisherNode()
  start_subscribers()

  robot = Config.getRobot()
  while True:
    robot.update()
    sensorReader.publishSensorData(robot)

def test_manual_control():
  from constants import Config, Topics
  from cortano import RemoteInterface
  robot = Config.getRobot()
  manualControl = ManualControl()
  while True:
    robot.update()
    manualControl.run(robot)

def test_robot_interface_nodes():
  from constants import Config, Topics
  from middleware import start_subscribers
  robot = Config.getRobot()
  # create sensor reader node
  SensorPublisherNode = SensorPublisherNode()

  # create manual control
  manualControl = ManualControl()

  # SensorReaderLoggerNode
  sensorReaderLogger = SensorReaderLoggerNode()

  # run the nodes
  start_subscribers()
  while True:
    robot.update()
    manualControl.update(robot)
    SensorPublisherNode.publishSensorData(robot)

if __name__ == "__main__":
  test_robot_interface_nodes()