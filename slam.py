from middleware import Node, start_subscribers
import numpy as np
import cupoch as cph
from cortano import RemoteInterface
from datetime import datetime
from config import Config

import multiprocessing

cph.initialize_allocator(cph.PoolAllocation, 1000000000)


class RgbdOdometryNode(Node):
  def __init__(self):
    super().__init__("RgbdOdometryNode")
    # self.pose_publisher = self.create_publisher("rgbd_pose")
    self.rgbd_subscriber = self.create_subscriber("rgbd", self.rgbd_callback)
    self.prev_cph_rgbd = None
    self.cur_trans = np.identity(4)
  
  def rgbd_callback(self, rgbd):
    color, depth = rgbd
    depth = depth.astype(np.float32)
    color = cph.geometry.Image(color)
    depth = cph.geometry.Image(depth)
    cph_rgbd = cph.geometry.RGBDImage.create_from_color_and_depth(
        color, depth, depth_scale = 1000
    )

    camera_intrinsics = cph.camera.PinholeCameraIntrinsic(
      Config.height,Config.width,Config.fx,Config.fy,Config.cx,Config.cy)
    option = cph.odometry.OdometryOption()
    option.min_depth = 0.30
    option.max_depth = 4

    if not self.prev_cph_rgbd is None:
      res, odo_trans, _ = cph.odometry.compute_rgbd_odometry(
          cph_rgbd,
          self.prev_cph_rgbd,
          camera_intrinsics,
          np.identity(4),
          cph.odometry.RGBDOdometryJacobianFromHybridTerm(),
          option,
      )

      if res:
        self.cur_trans = self.cur_trans @ odo_trans
        print(self.cur_trans[:3,3])

    self.prev_cph_rgbd = cph_rgbd

class SensorReader(Node):
  def __init__(self):
    super().__init__(SensorReader)
    self.rgbd_pub = self.create_publisher("rgbd")

  def update(self, robot):
    color, depth, _ = robot.read()
    self.rgbd_pub.publish((color, depth), block=False)


if __name__ == "__main__":
  multiprocessing.set_start_method('spawn', force=True)
  rgbdOdometry = RgbdOdometryNode()
  sensorReader = SensorReader()
  start_subscribers()

  robot = RemoteInterface("192.168.68.68")
  while True:
    robot.update()
    sensorReader.update(robot)