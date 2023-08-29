# import camera
from cortano import RemoteInterface
import numpy as np
import sys, os

print(os.path.dirname(os.path.realpath(__file__)) + "/..")
sys.path.insert(0, os.path.realpath(os.path.dirname(os.path.realpath(__file__)) + "/.."))
import perception


if __name__ == "__main__":
    robot = RemoteInterface("192.168.68.68")

    perc = perception.Perception()

    while True:
        robot.update()
        color, depth, sensors = robot.read()
        # color, depth = robot.read()

        ball_poses = perc.detect_balls(color, depth)
        print(ball_poses)