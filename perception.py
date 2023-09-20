import numpy as np
from torchvision.models.detection import maskrcnn_resnet50_fpn
from torchvision import transforms
from PIL import Image
import torch
import cv2
from constants import Config, Topics
from middleware import Node
from ultralytics import YOLO


class Perception:
  def __init__(self):  
    self.model = None
    self.yolo = None

  
  def detect_balls(self, color, depth, show_mask = True):
    """ positions in camera frame in meters
    """
    if self.model is None:
      self.model = maskrcnn_resnet50_fpn(pretrained=True, pretrained_backbone=True)
      self.model.eval()
      self.model.to('cuda')
      self.preprocess = transforms.Compose([ transforms.ToTensor(), ])
    rotated_image = np.rot90(color)
    input_tensor = self.preprocess(Image.fromarray(rotated_image))
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
    single_mask = np.zeros_like(depth, dtype=np.uint8)
    for idx in range(len(masks)):
      rotated_mask = np.rot90(masks[idx], 3, axes=(1,2))
      reshaped_mask = rotated_mask.reshape((360, 640))
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
  
  def detect_ballsYolo(self, color, depth, show_debug = True ):
    """ positions in camera frame in meters
    """
    if self.yolo is None:
      # self.yolo = YOLO("yolov8n.pt")
      self.yolo = YOLO("yolov8x.pt")
    rotated_image = np.rot90(color)
    output = self.yolo(rotated_image, device="cuda", verbose=False, conf=0.15)
    # print(output)
    result = output[0]
    bboxes = np.array(result.boxes.xyxy.cpu(), dtype="int")
    classes = np.array(result.boxes.cls.cpu(), dtype="int")
    tennisBallsIdx = classes == 32
    tennisBallBoxes = bboxes[tennisBallsIdx]
    
    ballPositionsInMeters = []
    single_mask = np.zeros_like(depth, dtype=np.uint8)

    for cls, bbox in zip(classes[tennisBallsIdx], tennisBallBoxes):
      (x, y, x2, y2) = bbox
      # shrink bbx
      factor = 0.5 # shrink factor
      (rx, ry) = (x2 - x)/2.0, (y2 - y)/2.0
      (cx, cy) = ((x2 + x)/2.0, (y2 + y)/2.0)
      (x, y, x2, y2) = (cx - rx*factor, cy - ry*factor, cx + rx*factor, cy + ry *factor)
      (x,y,x2,y2) = tuple([int(np.round(i)) for i in (x, y, x2, y2)])
      # rotate bounding box according to depth
      (x, y, x2, y2) = (640 - y,x, 640- y2, x2)
      mask = np.zeros_like(depth, np.int8)
      mask[min(y,y2):max(y,y2), min(x,x2):max(x,x2)] = 1
      single_mask[min(y,y2):max(y,y2), min(x,x2):max(x,x2)] = 255
      ball_depth = depth * mask
      xyz = self.get_XYZ(ball_depth)
      num_pixels = np.sum(ball_depth > 0)
      if num_pixels > 0:
        average_xyz = np.sum(xyz, axis=0) / num_pixels / 1000.0 # convert to meters
        ballPositionsInMeters.append(average_xyz)
      if show_debug:
        cv2.rectangle(color, (x, y), (x2, y2), (0, 0, 225), 2)
        cv2.putText(color, str(cls), (x2, y - 5), cv2.FONT_HERSHEY_PLAIN, 2, (0, 0, 225), 2)
    
    if show_debug:
      cv2.imshow("single_mask", single_mask)
      cv2.imshow("color", color)
      cv2.waitKey(1)
    
    return ballPositionsInMeters

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