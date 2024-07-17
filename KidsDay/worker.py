from cortano import VexCortex, lan
import pygame
import time
import sys, signal

pygame.init()
pygame.joystick.init()


while pygame.joystick.get_count() == 0:
  print("No joysticks connected.")
  time.sleep(0.5)

joystick = pygame.joystick.Joystick(0)
joystick.init()
print("Joystick initialized:", joystick.get_name())

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

  def stop(self):
    robot.motor[Actuation.RIGHT_MOTOR_INDEX] = 0
    robot.motor[Actuation.LEFT_MOTOR_INDEX] = 0
    robot.motor[Actuation.CLAW_MOTOR_INDEX] = 0
    robot.motor[Actuation.ARM_MOTOR_INDEX] = 0

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

  def generateMotorCmd(self, robot):
    robot.motor[Actuation.RIGHT_MOTOR_INDEX] = int(self.rightMotor)
    robot.motor[Actuation.LEFT_MOTOR_INDEX] = int(self.leftMotor)
    robot.motor[Actuation.CLAW_MOTOR_INDEX] = int(self.clawMotor)
    robot.motor[Actuation.ARM_MOTOR_INDEX] = int(self.armMotor)
    print("motor:",self.rightMotor, self.leftMotor, self.clawMotor, self.armMotor)
    print("sensors:", robot.sensor)

if __name__ == "__main__":
  robot = VexCortex("/dev/ttyUSB0")
  lan.control(robot)
  lan.start("robot", frame_shape=(360, 640))

  actuation = Actuation()

  while robot.running():
    # time.sleep(0.01)
    lan.check_alive()
    pygame.event.pump()
    actuation.reset()
    print("axis:", joystick.get_axis(1), joystick.get_axis(2))
    forward = joystick.get_axis(1)
    forward = translate(forward, -1.0, 1.0, -128, 128)
    spinCCW = -joystick.get_axis(2)
    spinCCW = translate(spinCCW, -1.0, 1.0, -128, 128)

    # Read button states
    button_states = [joystick.get_button(i) for i in range(joystick.get_numbuttons())]
    print("Buttons", button_states)
    actuation.goForward(-forward)
    actuation.spinCounterClockWise(spinCCW)
    arm = joystick.get_button(4) - joystick.get_button(3)
    armAngle = robot.sensor[0]
    if arm == 1 and armAngle < 3337:
      actuation.armCommand(52)
    elif arm == -1 and armAngle > 2503:
      actuation.armCommand(-52)
    else:
      actuation.armCommand(10)

    actuation.clawCommand((joystick.get_button(0) - joystick.get_button(1))*42)

    actuation.generateMotorCmd(robot)

  actuation.stop()
  print("stopping")
  lan.stop()