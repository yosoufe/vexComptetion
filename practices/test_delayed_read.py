from cortano import RemoteInterface
import time

if __name__ == "__main__":
    robot = RemoteInterface(Config.ip)

    while True:
        robot.update()
        color, depth, sensors = robot.read()
        time.sleep(1)