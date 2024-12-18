from middleware import Node
from constants import Topics
import keyboard
import numpy as np
import pygame

# Initialize the joystick module
pygame.joystick.init()

if pygame.joystick.get_count() == 0:
    print("No joysticks connected.")
    pygame.quit()
else:
    # Initialize the first joystick
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    print(f"Joystick initialized: {joystick.get_name()}")

def translate(value, leftMin, leftMax, rightMin, rightMax):
    # Figure out how 'wide' each range is
    leftSpan = leftMax - leftMin
    rightSpan = rightMax - rightMin

    # Convert the left range into a 0-1 range (float)
    valueScaled = float(value - leftMin) / float(leftSpan)

    # Convert the 0-1 range into a value in the right range.
    return rightMin + (valueScaled * rightSpan)

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
    self.armMotor = 12

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
    robot.motor = self.generateMotorCmd()

  def generateMotorCmd(self):
    cmd = np.zeros(shape=(10,), dtype=np.int32)
    cmd[Actuation.RIGHT_MOTOR_INDEX] = self.rightMotor
    cmd[Actuation.LEFT_MOTOR_INDEX] = self.leftMotor
    cmd[Actuation.CLAW_MOTOR_INDEX] = self.clawMotor
    cmd[Actuation.ARM_MOTOR_INDEX] = self.armMotor
    return cmd

class ManualControl:
  def __init__(self,):
    self.actuation = Actuation()

  def update(self, robot):
    self.actuation.reset()
    self.actuation.goForward((robot.keys["w"] - robot.keys["s"])* 40)
    self.actuation.spinCounterClockWise((robot.keys["a"] - robot.keys["d"])*35)
    arm = robot.keys["r"] - robot.keys["f"]
    if arm == 0:
      self.actuation.armCommand(10)
    elif arm == 1:
      self.actuation.armCommand(52)
    else:
      self.actuation.armCommand(-20)

    self.actuation.clawCommand((robot.keys["e"] - robot.keys["q"])*44)
    self.actuation.update(robot)

class ManualControlNode(Node):
  def __init__(self):
    super().__init__("ManualControlNode")
    self.commandPublisher = self.create_publisher(Topics.motorCommands)
    self.actuation = Actuation()

  # def publishMotorCommands(self, timestamp):
  #   self.actuation.reset()
  #   self.actuation.goForward((keyboard.is_pressed("w") - keyboard.is_pressed("s"))* 60)
  #   self.actuation.spinCounterClockWise((keyboard.is_pressed("a") - keyboard.is_pressed("d"))*55)
  #   arm = keyboard.is_pressed("r") - keyboard.is_pressed("f")
  #   if arm == 0:
  #     self.actuation.armCommand(10)
  #   elif arm == 1:
  #     self.actuation.armCommand(42)
  #   else:
  #     self.actuation.armCommand(-42)

  #   self.actuation.clawCommand((keyboard.is_pressed("q") - keyboard.is_pressed("e"))*42)
  #   self.commandPublisher.publish(timestamp, self.actuation.generateMotorCmd())

  def publishMotorCommands(self, timestamp):
    self.actuation.reset()
    pygame.event.pump()
    # Read axis values
    forward = joystick.get_axis(1)
    forward = translate(forward, -1.0, 1.0, -128, 128)
    spinCCW = -joystick.get_axis(2)
    spinCCW = translate(spinCCW, -1.0, 1.0, -128, 128)
    # print(f"Axis Motion 2 - X: {x_axis_2}, Y: {y_axis_2}")

    # Read button states
    button_states = [joystick.get_button(i) for i in range(joystick.get_numbuttons())]
    print(f"Buttons {button_states}")
    self.actuation.goForward(-forward)
    self.actuation.spinCounterClockWise(spinCCW)
    arm = joystick.get_button(3) - joystick.get_button(0)
    if arm == 0:
      self.actuation.armCommand(10)
    elif arm == 1:
      self.actuation.armCommand(42)
    else:
      self.actuation.armCommand(-42)

    self.actuation.clawCommand((joystick.get_button(4) - joystick.get_button(5) + joystick.get_button(2) - joystick.get_button(1))*42)
    self.commandPublisher.publish(timestamp, self.actuation.generateMotorCmd())

class SensorPublisherNode(Node):
  def __init__(self):
    super().__init__("SensorReader")
    self.rgbd_pub = self.create_publisher(Topics.rgbd)
    self.isMoving_pub = self.create_publisher(Topics.isMoving)
    self.sensorPub = self.create_publisher(Topics.sensors)

  def publishSensorData(self, robot):
    color, depth, sensors = robot.read()
    timestamp = sensors[0]
    self.rgbd_pub.publish(timestamp, (color, depth), block=False)
    isMoving = ((robot.motor[Actuation.RIGHT_MOTOR_INDEX] != 0) or (robot.motor[Actuation.LEFT_MOTOR_INDEX] != 0))
    self.isMoving_pub.publish(timestamp, isMoving)
    self.sensorPub.publish(timestamp, sensors[1:])

class SensorReaderLoggerNode(Node):
  def __init__(self):
    super().__init__("SensorReaderLoggerNode")
    self.rgbd_sub = self.create_subscriber(Topics.rgbd, self.rgbd_cb)
    self.isMoving_sub = self.create_subscriber(Topics.isMoving, self.isMoving_cb)
    self.isMoving_sub = self.create_subscriber(Topics.sensors, self.sensors_cb)

  def rgbd_cb(self, timestamp, rgbd):
    # print("SensorReaderLoggerNode rgbd_cb:")
    # print(timestamp, rgbd)
    pass

  def isMoving_cb(self, timestamp, isMoving):
    print("SensorReaderLoggerNode isMoving_cb:", timestamp, isMoving)

  def sensors_cb(self, timestamp, sensors):
    print("sensors:", sensors)

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

def test_manual_control_node():
  from constants import Config, Topics
  from middleware import start_subscribers
  import time
  robot = Config.getRobot()
  # create sensor reader node
  sensorPublisherNode = SensorPublisherNode()

  # create manual control
  manualControlNode = ManualControlNode()

  # SensorReaderLoggerNode
  sensorReaderLogger = SensorReaderLoggerNode()

  # run the nodes
  start_subscribers()
  while True:
    robot.update()
    manualControlNode.publishMotorCommands(time.time())
    sensorPublisherNode.publishSensorData(robot)

if __name__ == "__main__":
  # test_robot_interface_nodes()
  test_manual_control_node()