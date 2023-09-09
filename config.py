import numpy as np

class Config:
  ip = "192.168.50.206"
  fx = 460.92495728   # FOV(x) -> depth2xyz -> focal length (x)
  fy = 460.85058594   # FOV(y) -> depth2xyz -> focal length (y)
  cx = 315.10949707   # 640 (width) 320
  cy = 176.72598267   # 360 (height) 180
  width = 360
  height = 640
  camera_params = ( fx, fy, cx, cy )
  tag_size = 3 * 25.4 # mm
  at_zero_rot = np.array([[ 0.0695368 ,  0.99752359,  0.01055085],
                          [-0.46330579,  0.02292656,  0.88590187],
                          [ 0.88346612, -0.06649105,  0.46375269]])
  at_zero_position  = np.array([[107.75917427],
                                [101.96002389],
                                [450.90017004]])
  _cam2RobotT = None
  _robot2CamT = None
  _AT2Robot = np.array([[0, -1, 0, 0],
                       [-1, 0, 0, 0],
                       [0, 0, -1, 0],
                       [0, 0, 0, 1]], dtype=float)
  
  def cam2RobotT():
    if Config._cam2RobotT is None:
      AT2Camera = np.ones((4,4), dtype= float)
      AT2Camera[:3, :3] = Config.at_zero_rot
      AT2Camera[:3, 3] = Config.at_zero_position.squeeze()
      # print(AT2Camera)
      Config._cam2RobotT = Config._AT2Robot @ np.linalg.inv(AT2Camera)
    return Config._cam2RobotT

  def robot2CamT():
    if Config._robot2CamT is None:
      Config._robot2CamT = np.linalg.inv(Config.cam2RobotT())
    return Config._robot2CamT