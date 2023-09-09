from pyapriltags import Detector
import cv2
from config import Config

class ATDetector:
  def __init__(self):
    self.detector = Detector(families='tag16h5',
                             nthreads=1,
                             quad_decimate=1.0,
                             quad_sigma=0.0,
                             refine_edges=1,
                             decode_sharpening=0.25,
                             debug=0)
  
  def detect(self, color_image):
    tags = self.detector.detect(
        img = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY),
        estimate_tag_pose = True,
        camera_params = Config.camera_params,
        tag_size = Config.tag_size)
    
    for tag in tags:
      if tag.decision_margin < 50:
        continue 
      print(repr(tag.tag_id))
      print(repr(tag.pose_R))
      print(repr(tag.pose_t))
      print(repr(tag))
    
    # print("------------")

if __name__ == "__main__":
  from cortano import RemoteInterface
  from actuation import ManualControl
  robot = RemoteInterface(Config.ip)
  at_detector = ATDetector()
  manualControl = ManualControl(robot)

  while True:
    robot.update()
    manualControl.run()
    color, _, _ = robot.read()
    # color, depth = robot.read()

    ball_poses = at_detector.detect(color)
    # print(ball_poses)