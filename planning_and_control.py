from middleware import Node
from constants import Topics, Map


class PlanningAndControlNode(Node):
  def __init__(self):
    super().__init__("PlanningAndControlNode")
    self.motorCmdsPub = self.create_publisher(Topics.motorCommands)
    self.ballPositionSub = self.create_subscriber(Topics.ballPositions, self.ballPosition_cb)
    self.localizationSub = self.create_subscriber(Topics.fusedPose, self.localization_cb)
    self.tickSub = self.create_subscriber(Topics.isMoving, self.tick)
    self.actuation = None
    self.latestBallPositions = None
    self.latestBallPositionsTimestamp = None
    self.latestRobotPose = None
    self.latestRobotPoseTimestamp = None
    self.ballPositionsInMap = None
  
  def ballPosition_cb(self, timestamp, ballPositions):
    self.latestBallPositions = ballPositions
    self.latestBallPositionsTimestamp = timestamp

  def localization_cb(self, timestamp, localization):
    self.latestRobotPose = localization
    self.latestRobotPoseTimestamp = timestamp
    self.ballPositionsInMap = self.latestRobotPose @ self.latestBallPositions

  def tick(self, timestamp, isMoving):
    """ This is subscribing to IsMoving 
    and is used as a tick function. The isMoving parameter
    might not be used
    """
    if self.actuation is None:
      from robot_interface_node import Actuation
      self.actuation = Actuation()
    
    # We need to support multiple missions
    # Each mission needs to define the next mission
    # Either by using an enum or returning a new mission object
    # Each mission would have access to PlanningAndControlNode data members or the self instant.

    # Mission: Search for 1st april Tag for localization

    # Mission: Search for a tennis ball

    # Mission: go to the ball
    # choose a ball to grab
    # path plan to the ball
    # run PID control to calculate motor values
    # apply actuation

    # Mission: grab the ball and move the arm up

    # Mission: Go to the other side of the map

    # Mission: Drop the ball

    # Mission: go back to our map

    # Mission: lower the arm

    # Start over.

    
    self.actuation.reset()
    self.motorCmdsPub.publish(timestamp, self.actuation.generateMotorCmd())

def test_planning_and_control_node():
  from constants import Config
  from perception import PerceptionNode
  from localization import LocalizationNode
  from robot_interface_node import SensorPublisherNode
  from middleware import start_subscribers

  robot = Config.getRobot()
  sensorPublisherNode = SensorPublisherNode()
  planningAndControlNode = PlanningAndControlNode()
  localizationNode = LocalizationNode()
  perceptionNode = PerceptionNode()
  start_subscribers()
  
  while True:
    robot.update()
    sensorPublisherNode.publishSensorData(robot)


if __name__ == "__main__":
  import multiprocessing as mp
  mp.set_start_method('spawn', force=True)
  test_planning_and_control_node()