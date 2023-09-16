from perception import PerceptionNode
from localization import LocalizationNode
from robot_interface_node import ManualControl, SensorPublisherNode
from constants import Config
import multiprocessing as mp
from middleware import start_subscribers


if __name__ == "__main__":
  mp.set_start_method('spawn', force=True)
  manualControl = ManualControl()
  localizationNode = LocalizationNode()
  perceptionNode = PerceptionNode()
  sensorPublisherNode = SensorPublisherNode()
  start_subscribers()

  robot = Config.getRobot()
  while True:
    robot.update()
    manualControl.update(robot)
    sensorPublisherNode.publishSensorData(robot)