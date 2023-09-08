class Config:
  fx = 460.92495728   # FOV(x) -> depth2xyz -> focal length (x)
  fy = 460.85058594   # FOV(y) -> depth2xyz -> focal length (y)
  cx = 315.10949707   # 640 (width) 320
  cy = 176.72598267   # 360 (height) 180
  width = 360
  height = 640
  camera_params = ( fx, fy, cx, cy )
  tag_size = 4 * 25.4 # mm