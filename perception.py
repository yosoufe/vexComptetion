import numpy as np
from torchvision.models.detection import maskrcnn_resnet50_fpn
from torchvision import transforms
from PIL import Image
import torch
import cv2
from constants import Config, Topics

class Perception:
  def __init__(self):
    self.model = maskrcnn_resnet50_fpn(pretrained=True, pretrained_backbone=True)
    self.model.eval()
    self.model.to('cuda')
    self.preprocess = transforms.Compose([ transforms.ToTensor(), ])

  
  def detect_balls(self, color, depth, show_mask = True):
    """ positions in camera frame in meters
    """
    input_tensor = self.preprocess(Image.fromarray(color))
    # print(input_tensor)
    input_tensor = input_tensor * 0.9
    input_batch = input_tensor.unsqueeze(0).to('cuda')
    with torch.no_grad():
      output = self.model(input_batch)[0]
    output = {l: output[l].to('cpu').numpy() for l in output}

    ball_poses = []

    object_index = 37
    indeces_found = np.logical_and(output["labels"]==object_index, output["scores"] > 0.2)
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
        average_xyz = np.sum(xyz, axis=0) / num_pixels / 1000.0
        ball_poses.append(average_xyz)
    
    if show_mask:
      cv2.imshow("color", single_mask)
      cv2.waitKey(1)

    return ball_poses

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

if __name__ == "__main__":
  robot = Config.getRobot()
  perc = Perception()

  while True:
    robot.update()
    color, depth, sensors = robot.read()
    # color, depth = robot.read()

    ball_poses = perc.detect_balls(color, depth)
    print(ball_poses)