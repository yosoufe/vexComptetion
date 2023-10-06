import numpy as np
# everything is in metrics


class Config:
  # ip = "192.168.50.206" # in practice sessions
  ip = "192.168.68.68"  # at home
  fx = 460.92495728   # FOV(x) -> depth2xyz -> focal length (x)
  fy = 460.85058594   # FOV(y) -> depth2xyz -> focal length (y)
  cx = 315.10949707   # 640 (width) 320
  cy = 176.72598267   # 360 (height) 180
  width = 640
  height = 360
  camera_params = (fx, fy, cx, cy)

  # Robot frame in camera frame
  at_zero_rot = np.array([[0.10036645, -0.11373534, -0.98842847],
                          [0.9810449,  0.17682528,  0.07926998],
                          [0.16576335, -0.97764876,  0.1293268]])
  at_zero_position = np.array([[-0.08517979],
                               [0.07962572],
                               [0.30808843]])

  _cam2RobotT = None
  _robot2CamT = None
  _CalibrationAT2Robot = np.array([[0, -1, 0, 0],
                                   [-1, 0, 0, 0],
                                   [0, 0, -1, 0],
                                   [0, 0, 0, 1]], dtype=float)

  @staticmethod
  def cam2RobotT():
    if Config._cam2RobotT is None:
      AT2Camera = np.eye(4, dtype=float)
      AT2Camera[:3, :3] = Config.at_zero_rot
      AT2Camera[:3, 3] = Config.at_zero_position.squeeze()
      # print(AT2Camera)
      Config._cam2RobotT = Config._CalibrationAT2Robot @ np.linalg.inv(AT2Camera)
    return Config._cam2RobotT

  @staticmethod
  def robot2CamT():
    if Config._robot2CamT is None:
      Config._robot2CamT = np.linalg.inv(Config.cam2RobotT())
    return Config._robot2CamT

  zero_offset = np.array([-0.08315515, 0.04375288])

  _robot = None
  _robotNode = None

  @staticmethod
  def getRobot():
    # from cortano import RemoteInterface
    from remote_interface_node import RemoteInterfaceNodeMultiProcessSafe, RemoteInterfaceNode
    import multiprocessing as mp
    if Config._robot is None:
      queue = mp.Queue(maxsize=1)
      Config._robot = RemoteInterfaceNodeMultiProcessSafe(queue, Config.ip, port=9999)
      Config._robotNode = RemoteInterfaceNode(queue)
    return Config._robot


class OldConfigs:
  at_zero_rot = np.array([[0.0695368, 0.99752359, 0.01055085],
                          [-0.46330579, 0.02292656, 0.88590187],
                          [0.88346612, -0.06649105, 0.46375269]])
  at_zero_position = np.array([[107.75917427],
                               [101.96002389],
                               [450.90017004]])

# using ball detector for the calibration

# cameraToRobotT = np.array(
#   [[0.04892364, -0.41772631, 0.90725477, -0.32358759],
#    [-0.99831263, -0.0435315, 0.03379077, 0.09919108],
#    [0.02448331, -0.89973047, -0.43575871, 0.24580773],
#    [0., 0., 0., 1.]],
#   dtype=float
# )

# cameraToRobotT = np.array(
#   [[ 0.04183374 ,-0.40959374 , 0.91130835 ,-0.33556501],
#    [-0.99828073 ,-0.05146286,  0.02269589,  0.10270806],
#    [ 0.03691244 ,-0.90393521 ,-0.42607349 , 0.2442368 ],
#    [ 0.          ,0.         , 0.         , 1.        ]],
#   dtype=float
# )


# zero offset: target: [-0.08315515, 0.04375288]


# MAP at Home

class _HomeMap:
  tag_size = 72 / 1000  # m
  tag_ids = set(list(range(1, 5)))
  _landmarks = {
      1: np.array([[1, 0, 0, 0],
                   [0, 0, 1, 0],
                   [0, -1, 0, 0],
                   [0, 0, 0, 1]], dtype=float),
      3: np.array([[0.99471329, -0.01590299, -0.10145225, 1.04627609],
                   [0.02096435, 0.99857778, 0.0490196, 0.02472891],
                   [0.1005284, -0.05088733, 0.99363198, 0.02251224],
                   [0., 0., 0., 1.]]),
      2: np.array([[-3.86255540e-02, 3.89327231e-02, -9.98495022e-01, -2.59944926e-01],
                   [7.12936093e-04, 9.99241520e-01,
                    3.89342511e-02, -2.82976469e-02],
                   [9.99253501e-01, 7.91993879e-04, -
                    3.86240139e-02, -1.05946318e+00],
                   [0.00000000e+00, 0.00000000e+00, 0.00000000e+00, 1.00000000e+00]]),
      4: np.array([[-1.15711296e-02, 7.91264896e-04, 9.99932739e-01, 1.88734180e+00],
                   [-2.29588177e-02, 9.99735853e-01, -
                    1.05678642e-03, 1.74137121e-03],
                   [-9.99669446e-01, -2.29695017e-02, -
                    1.15499066e-02, -5.35296744e-01],
                   [0.00000000e+00, 0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])
  }

  X_limits = np.array([-1, 2],dtype= float)
  Y_limits = np.array([-2, 0.2],dtype= float)
  GRID_MAP = None
  GRID_SIZE_METERS = 0.025

  @staticmethod
  def getLandmark(id):
    if id == 1:
      return _HomeMap._landmarks[1]
    return _HomeMap._landmarks[1] @ _HomeMap._landmarks[id]


# Map at competition


class _CompetitionMap:
  tag_size = 3 * 0.0254  # m
  # tag_size = 2.735 * 0.0254 # m
  tag_ids = set(list(range(1, 9)))
  # in _landmarks, positions are in inches
  # they are converted to meters
  # in the function below
  _landmarks = {
      1: np.array([[0, 0, -1, -72],
                   [1, 0, 0, 24],
                   [0, -1, 0, 0],
                   [0, 0, 0, 1]], dtype=float),
      2: np.array([[1, 0, 0, -24],
                   [0, 0, 1, 72],
                   [0, -1, 0, 0],
                   [0, 0, 0, 1]], dtype=float),
      3: np.array([[1, 0, 0, 24],
                   [0, 0, 1, 72],
                   [0, -1, 0, 0],
                   [0, 0, 0, 1]], dtype=float),
      4: np.array([[0, 0, 1, 72],
                   [-1, 0, 0, 24],
                   [0, -1, 0, 0],
                   [0, 0, 0, 1]], dtype=float),
      5: np.array([[0, 0, 1, 72],
                   [-1, 0, 0, -24],
                   [0, -1, 0, 0],
                   [0, 0, 0, 1]], dtype=float),
      6: np.array([[-1, 0, 0, 24],
                   [0, 0, -1, -72],
                   [0, -1, 0, 0],
                   [0, 0, 0, 1]], dtype=float),
      7: np.array([[-1, 0, 0, -24.],
                   [0, 0, -1, -72.],
                   [0, -1, 0, 0],
                   [0, 0, 0, 1]], dtype=float),
      8: np.array([[0, 0, -1, -72],
                   [1, 0, 0, -24],
                   [0, -1, 0, 0],
                   [0, 0, 0, 1]], dtype=float)
  }

  X_limits = np.array([-72 * 0.0254, 72 * 0.0254], dtype= float)
  Y_limits = np.array([-72 * 0.0254, 72 * 0.0254], dtype= float)
  GRID_MAP = None
  GRID_SIZE_METERS = 0.025

  @staticmethod
  def getLandmark(id):
    """ Transformation from  april tag frame to map frame
    """
    transform = _CompetitionMap._landmarks[id].copy()
    # convert to meters
    transform[:2, 3] = transform[:2, 3] * 0.0254
    # z component of the center
    transform[2, 3] = 10 * 0.0254
    return transform


# Map = _CompetitionMap
Map = _HomeMap

def getGridMap():
  if Map.GRID_MAP is None:
    dx = Map.X_limits[1] - Map.X_limits[0]
    dy = Map.Y_limits[1] - Map.Y_limits[0]
    resolution = 0.025 # meters, close to 1 inch
    Map.GRID_MAP = np.zeros(shape=(np.ceil(dx/resolution), np.ceil(dy/resolution+1)), dtype=int)
  return Map.GRID_MAP

def getGridIdxForXY(positions):
  idx = np.round((positions - np.array([Map.X_limits[0], Map.Y_limits[0]]))/Map.GRID_SIZE_METERS)
  return idx.astype(int)

class Topics:
  isMoving = "isMoving"
  rgbd = "rgbd"
  atPose = "atPose"
  odom = "odom"                   # robot frame relative to previous robot frame
  fusedPose = "fusedPose"         # robot frame relative to map frame
  ballPositions = "ballPositions" # relative to robot frame
  motorCommands = "motorCommands" # 
  sensors = "sensors"             # sensors: Potentiometer and switches
  switchPerception = "switchPerception" # to enable or disable perception

  # for debugging
  targetPose = "targetPose" # the target pose
  globalTarget = "globalTarget"

if __name__ == "__main__":
  print(getGridIdxForXY(np.array([0,0])))
  print(getGridIdxForXY(np.array([Map.X_limits[0], Map.Y_limits[0]])))
  # print(np.array([Map.X_limits[0], Map.Y_limits[0]]))