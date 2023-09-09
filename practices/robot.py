from cortano import RemoteInterface

MOTOR_RIGHT = 0
MOTOR_LEFT = 9
MOTOR_CLAW = 7
MOTOR_ARM = 8


if __name__ == "__main__":
  robot = RemoteInterface(Config.ip)
  while True:
    robot.update() # must never be forgotten
    color, depth, sensors = robot.read()

    # forward/backward
    forward = robot.keys["w"] - robot.keys["s"]
    motor_right =  forward * 50
    motor_left  = -forward * 50

    # right/left
    right = robot.keys["d"] - robot.keys["a"]
    robot.motor[MOTOR_RIGHT] = motor_right + (right* 45)
    robot.motor[MOTOR_LEFT] = motor_left + (right*45)

    # arm up/down
    arm = robot.keys["r"] - robot.keys["f"]
    if arm == 0:
      robot.motor[MOTOR_ARM] = 10
    elif arm == 1:
      robot.motor[MOTOR_ARM] = 42
    else:
      robot.motor[MOTOR_ARM] = -1

    # claw
    claw = robot.keys["q"] - robot.keys["e"]
    robot.motor[7] = claw * 42 
    
    # print(f"claw: {robot.motor[MOTOR_CLAW]}")
    # print(f"arm: {robot.motor[MOTOR_ARM]}")
    # print(f"left: {robot.motor[MOTOR_LEFT]}")
    # print(f"right: {robot.motor[MOTOR_RIGHT]}")
    # print(f"right key: {right}")
    # print(sensors)
    # print(f"--------------------")

