import numpy as np
from torchvision.models.detection import maskrcnn_resnet50_fpn
from torchvision import transforms
from PIL import Image
import torch
import cv2
from constants import Config, Topics
from ultralytics import YOLO
from middleware import Node

class Perception:
  def __init__(self):
    self.model = maskrcnn_resnet50_fpn(pretrained=True, pretrained_backbone=True)
    self.model.eval()
    self.model.to('cuda')
    self.preprocess = transforms.Compose([ transforms.ToTensor(), ])
    self.yolo = YOLO("yolov8n.pt")

  
  def detect_balls(self, color, depth, show_mask = True):
    """ positions in camera frame in meters
    """
    input_tensor = self.preprocess(Image.fromarray(color))
    # print(input_tensor)
    # input_tensor = input_tensor * 0.9
    input_batch = input_tensor.unsqueeze(0).to('cuda')
    with torch.no_grad():
      output = self.model(input_batch)[0]
    output = {l: output[l].to('cpu').numpy() for l in output}

    ballPositionsInMeters = []

    object_index = 37
    prob_threshold = 0.2
    indeces_found = np.logical_and(output["labels"]==object_index, output["scores"] > prob_threshold)
    masks  = output["masks"][indeces_found]
    # print(output["labels"], masks)
    single_mask = np.zeros((360, 640), dtype=np.uint8)
    for idx in range(len(masks)):
      reshaped_mask = masks[idx].reshape((360, 640))
      indecies = reshaped_mask>0.15
      single_mask[indecies] = 255

      ball_depth = depth * (reshaped_mask > 0)
      xyz = self.get_XYZ(ball_depth)
      num_pixels = np.sum(ball_depth > 0)

      if num_pixels > 0:
        average_xyz = np.sum(xyz, axis=0) / num_pixels / 1000.0 # convert to meters
        ballPositionsInMeters.append(average_xyz)
    
    if show_mask:
      cv2.imshow("color", single_mask)
      cv2.waitKey(1)

    # relative to camera frame
    return ballPositionsInMeters
  
  def detect_ballsYolo(self, color, depth):
    """ positions in camera frame in meters
    """
    output = self.yolo(color, device="cuda")
    print("----------------")
    print(len(output))
    # print(output)
    for result in output:
      print(result.boxes.cls.nelement() == 0)
      print(result.boxes.cls)
      print(result)
      if result.boxes.cls.cpu() == 32:
        print(result)
        # print(result.__doc__)
        # print(result.boxes.__doc__)
        # print(result.boxes.cls)

    # ballPositionsInMeters = []

    # object_index = 37
    # prob_threshold = 0.2
    # indeces_found = np.logical_and(output["labels"]==object_index, output["scores"] > prob_threshold)
    # masks  = output["masks"][indeces_found]
    # # print(output["labels"], masks)
    # single_mask = np.zeros((360, 640), dtype=np.uint8)
    # for idx in range(len(masks)):
    #   reshaped_mask = masks[idx].reshape((360, 640))
    #   indecies = reshaped_mask>0.15
    #   single_mask[indecies] = 255

    #   ball_depth = depth * (reshaped_mask > 0)
    #   xyz = self.get_XYZ(ball_depth)
    #   num_pixels = np.sum(ball_depth > 0)

    #   if num_pixels > 0:
    #     average_xyz = np.sum(xyz, axis=0) / num_pixels / 1000.0 # convert to meters
    #     ballPositionsInMeters.append(average_xyz)
    
    # if show_mask:
    #   cv2.imshow("color", single_mask)
    #   cv2.waitKey(1)

    # # relative to camera frame
    # return ballPositionsInMeters

  def get_XYZ(self, depth_image):
    h, w = (Config.height, Config.width)
    U = np.tile(np.arange(w).reshape((1, w)), (h, 1))
    V = np.tile(np.arange(h).reshape((h, 1)), (1, w))
    U = (U - Config.cx) / Config.fx
    V = (V - Config.cy) / Config.fy

    Z = depth_image
    X = U * Z
    Y = V * Z
    # formatting magic
    XYZ = np.concatenate((
        X.reshape((-1, 1)),
        Y.reshape((-1, 1)),
        Z.reshape((-1, 1))
    ), axis=-1)
    return XYZ

class PerceptionNode(Node):
  def __init__(self):
    super().__init__("PerceptionNode")
    self.rgbdSub = self.create_subscriber(Topics.rgbd, self.rgbd_cb)
    self.ballPositionPublisher = self.create_publisher(Topics.ballPositions)
    
    self.perception = None

  def rgbd_cb(self, timestamp, rgbd):
    if self.perception is None:
      self.perception = Perception()
    
    color, depth = rgbd
    # ballPositionsInMeters = self.perception.detect_balls(color, depth)
    ballPositionsInMeters = self.perception.detect_ballsYolo(color, depth)
    return
    n_balls = len(ballPositionsInMeters)
    if (n_balls > 0):
      # convert the ball positions from 
      # camera frame to robot frame
      tennisBalls = np.vstack(ballPositionsInMeters).T # 3 x n_balls
      positions = np.ones((4,n_balls), float)
      positions[:3, :] = tennisBalls
      positionsInRobotFrame = np.sort(Config.cam2RobotT() @ positions, axis = 1)
      
      # print(positionsInRobotFrame[:2, :] * 39.3701, "in inches")
      self.ballPositionPublisher.publish(timestamp, positionsInRobotFrame)


def test_perception_class():
  robot = Config.getRobot()
  perc = Perception()

  while True:
    robot.update()
    color, depth, _ = robot.read()

    ballPositionsInMeters = perc.detect_balls(color, depth)
    print(ballPositionsInMeters)

      

def test_perception_node():
  from middleware import start_subscribers
  from robot_interface_node import ManualControl, SensorPublisherNode
  manualControl = ManualControl()
  sensorReader = SensorPublisherNode()
  perception = PerceptionNode()
  start_subscribers()
  
  robot = Config.getRobot()
  while True:
    robot.update()
    manualControl.update(robot)
    sensorReader.publishSensorData(robot)

if __name__ == "__main__":
  import multiprocessing
  multiprocessing.set_start_method('spawn', force=True)
  # test_perception_class()
  test_perception_node()