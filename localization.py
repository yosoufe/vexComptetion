from middleware import Node
import numpy as np
from constants import Config, Topics, Map
import cv2
import cupoch as cph
from planning_and_control import PlotterNode
from scipy.spatial.transform import Rotation as R
cph.initialize_allocator(cph.PoolAllocation, 1000000000)

PRINT_DETECTED_TAGS = False

class ATLocalizerNode(Node):
  def __init__(self):
    super().__init__("ATLocalizerNode")
    self.atPosePub = self.create_publisher(Topics.atPose)
    self.rgbdSub = self.create_subscriber(Topics.rgbd, self.localize)
    self.detector = None
  
  def detect(self, color_image, debug = False):
    from pyapriltags import Detector
    if self.detector is None:
      self.detector = Detector(families='tag16h5',
                               nthreads=4,
                               quad_decimate=1.0,
                               quad_sigma=0.0,
                               refine_edges=1,
                              #  decode_sharpening=0.25,
                               decode_sharpening=1.2,
                               debug=0)
    tags = self.detector.detect(
      img = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY),
      estimate_tag_pose = True,
      camera_params = Config.camera_params,
      tag_size = Map.tag_size)

    # print(tags)
    # for tag in tags:
    #   print("tag:", tag.tag_id, tag.decision_margin, tag.pose_err)
    
    tags = [tag for tag in tags if tag.decision_margin > 100 and
            tag.tag_id in Map.tag_ids
            and abs(tag.pose_err) < 8e-6]

    if debug:
      for tag in tags:
        print("tag:", tag.tag_id, tag.decision_margin, tag.pose_err)
        # print(repr(tag.pose_R))
        # print(repr(tag.pose_t))
    
    return tags

  def localize(self, timestamp, rgbd):
    # just use the first tag in the list
    color, _ = rgbd
    tags = self.detect(color, debug= PRINT_DETECTED_TAGS)
    if len(tags) == 0:
      return
    tag = tags[0]
    AtToCamera = self.tagToT(tag)
    landMarkToMap = Map.getLandmark(tag.tag_id)
    cameraToAt = np.linalg.inv(AtToCamera)
    robotInMap = landMarkToMap @ cameraToAt @ Config.robot2CamT()
    self.atPosePub.publish(timestamp, robotInMap)

  def tagToT(self, tag):
    T = np.eye(4,dtype = float)
    T[:3,:3] = tag.pose_R
    T[:3, 3] = tag.pose_t.squeeze()
    return T


class RgbdOdometryNode(Node):
  def __init__(self):
    super().__init__("RgbdOdometryNode")
    self.odom_publisher = self.create_publisher(Topics.odom)
    self.rgbd_subscriber = self.create_subscriber(Topics.rgbd, self.rgbd_cb)
    self.isMoving_subscriber = self.create_subscriber(Topics.isMoving, self.isMoving_cb)
    self.prev_cph_rgbd = None
    self.cur_trans = np.identity(4)
    self.isMoving = False
    self.timestampWhenStoppedMoving = 0

  def isMoving_cb(self, timestamp, isMoving):
    if self.isMoving == True and isMoving == False:
      self.timestampWhenStoppedMoving = timestamp
    self.isMoving = isMoving
  
  def rgbd_cb(self, timestamp, rgbd):
    # https://github.com/neka-nat/cupoch/blob/8e48f1c4d91bae5bfb4536152e72de19159f1006/examples/python/ros/realsense_rgbd_odometry_node.py#L96
    color, depth = rgbd
    depth = depth.astype(np.float32)
    color = cph.geometry.Image(color)
    depth = cph.geometry.Image(depth)
    cph_rgbd = cph.geometry.RGBDImage.create_from_color_and_depth(
        color, depth, depth_scale = 1000
    )

    camera_intrinsics = cph.camera.PinholeCameraIntrinsic(
      Config.width, Config.height,
      Config.fx, Config.fy, Config.cx, Config.cy)
    option = cph.odometry.OdometryOption()
    option.min_depth = 0.40
    option.max_depth = 15

    if not self.prev_cph_rgbd is None \
      and (self.isMoving or \
           (not self.isMoving and \
            (timestamp - self.timestampWhenStoppedMoving) < 0.3)):
      res, odomInCamFrame, _ = cph.odometry.compute_rgbd_odometry(
          cph_rgbd,
          self.prev_cph_rgbd,
          camera_intrinsics,
          np.identity(4),
          cph.odometry.RGBDOdometryJacobianFromHybridTerm(),
          option,
      )

      if res:
        # odomInCamFrame is T from current cam 2 prev cam
        odomInRobotFrame = Config.cam2RobotT() @ odomInCamFrame @ Config.robot2CamT()
        # print("%.10f"%(np.linalg.norm(odomInRobotFrame[:2,3])))
        euler = R.from_matrix(odomInRobotFrame[:3,:3]).as_euler('zyx', degrees=True)
        # print(f"localization yaw: {euler[0]:.10f}")
        if (np.linalg.norm(odomInRobotFrame[:2,3]) < 0.05 and abs(euler[0]) < 10):
          self.odom_publisher.publish(timestamp, odomInRobotFrame)


    self.prev_cph_rgbd = cph_rgbd

class LocalizationNode(Node):
  def __init__(self):
    super().__init__("LocalizationNode")
    self.odomSub = self.create_subscriber(Topics.odom, self.odom_cb)
    self.atPoseSub = self.create_subscriber(Topics.atPose, self.at_pose_cb)
    self.fusePosePublisher = self.create_publisher(Topics.fusedPose)
    self.currentPose = None
    self.lastAtPoseTimeStamp = 0
  
  def odom_cb(self, timestamp, odom):
    # we need at least the first AT Localization
    # And don't use RGBD odom if AT Localization is more recent.
    if timestamp < self.lastAtPoseTimeStamp or self.lastAtPoseTimeStamp == 0:
      return
    self.currentPose = self.currentPose @ odom
    self.publishCurrentPose(timestamp)

  def at_pose_cb(self, timestamp, atPose):
    self.lastAtPoseTimeStamp = timestamp
    self.currentPose = atPose
    self.publishCurrentPose(timestamp)
  
  def publishCurrentPose(self, timestamp):
    # print("localizationNode", self.currentPose[:2, 3], self.currentPose[:2, 3] * 39.3701)
    self.fusePosePublisher.publish(timestamp, (self.lastAtPoseTimeStamp, self.currentPose))

def generateLocalizationNodes(withGraph = False):
  RgbdOdometryNode()
  ATLocalizerNode()
  LocalizationNode()
  if withGraph:
    PlotterNode()

def test_at_localizerNode():
  from middleware import start_subscribers
  from robot_interface_node import ManualControl, SensorPublisherNode
  robot = Config.getRobot()
  manualControl = ManualControl()
  atLocalizerNode = ATLocalizerNode()
  sensorPublisherNode = SensorPublisherNode()
  start_subscribers()

  while True:
    robot.update()
    manualControl.update(robot)
    sensorPublisherNode.publishSensorData(robot)


def test_rgbd_odom_node():
  from middleware import start_subscribers
  from robot_interface_node import ManualControl, SensorPublisherNode
  manualControl = ManualControl()
  rgbdOdometry = RgbdOdometryNode()
  sensorReader = SensorPublisherNode()
  start_subscribers()

  robot = Config.getRobot()
  while True:
    robot.update()
    manualControl.update(robot)
    sensorReader.publishSensorData(robot)

def test_localization_node():
  from middleware import start_subscribers
  from robot_interface_node import ManualControl, SensorPublisherNode
  manualControl = ManualControl()
  generateLocalizationNodes()
  sensorReader = SensorPublisherNode()
  posePlotter = PlotterNode()
  start_subscribers()

  robot = Config.getRobot()
  while True:
    robot.update()
    manualControl.update(robot)
    sensorReader.publishSensorData(robot)

if __name__ == "__main__":
  import multiprocessing
  multiprocessing.set_start_method('spawn', force=True)
  # test_at_localizerNode()
  # test_rgbd_odom_node()
  test_localization_node()