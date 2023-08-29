from cortano import RemoteInterface

if __name__ == "__main__":
  robot = RemoteInterface("192.168.68.68")
  while True:
    robot.update() # must never be forgotten
    color, depth, sensors = robot.read()

    forward = robot.keys["w"] - robot.keys["s"]
    robot.motor[0] =  forward * 50
    robot.motor[9] = -forward * 50
    
    arm = robot.keys["q"] - robot.keys["e"]
    claw = robot.keys["a"] - robot.keys["d"]

    robot.motor[7] = claw * 42 
    if arm == 0:
      robot.motor[8] = 10
    elif arm == 1:
      robot.motor[8] = 42
    else:
      robot.motor[8] = -1
    print(f"claw: {robot.motor[7]}")
    print(f"arm: {robot.motor[8]}")
    print(sensors)
    print(f"--------------------")

