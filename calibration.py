from cortano import RemoteInterface
import pygame
import numpy as np
import perception

class Button():
  white = (255,255,255)
  black = (0,0,0)
  red = (255,0,0)

  def __init__(self, x, y, text, w, h):
    self.text = text
    self.rect = pygame.Rect(x,y,w,h)
    self.clicked = False
  
  def handle_click(self):
    action = False
    
    # get mouse position
    pos = pygame.mouse.get_pos()

    # check mouseover and clicked conditions
    if self.rect.collidepoint(pos):
      if pygame.mouse.get_pressed()[0] == 1 and self.clicked == False:
        self.clicked = True
        action = True
    if pygame.mouse.get_pressed()[0] == 0:
      self.clicked = False
    return action

  def draw(self, screen):
    
    pygame.draw.rect(screen, (Button.white), self.rect, 2)
    font = pygame.font.SysFont('Arial', 25)
    
    # draw button on screen
    screen.blit(font.render(self.text, True, Button.red), (self.rect.x, self.rect.y))

    return self.handle_click()

    

class CalibrationInterface(RemoteInterface):
  def __init__(self, host="0.0.0.0",):
    super().__init__(host)
    self.Z_button = Button(x = 10, y = 370, w= 400, h=30, text= "Capture Ball Position For Z")
    self.X_button = Button(x = 10, y = 410, w= 400, h=30, text= "Capture Ball Position For X")
    self.Zero_button = Button(x = 10, y = 450, w= 400, h=30, text= "Capture Ball Position For Origin")
    

    self.poses_for_z = []
    self.poses_for_x = []
    self.origin_capture = None
    self.pose = np.identity(4, dtype=float)
    self.perc = perception.Perception()
  
  def update(self):
    super().update()
    color, depth, _ = self.read()
    balls = self.perc.detect_balls(color, depth)

    if len(balls) != 1:
      return
    
    ball = balls[0]
    
    if self.Z_button.draw(self.screen):
      self.poses_for_z.append(ball)
      print("ball:", ball)
      print("captured for Z")
      self.calculate_z()
      print(self.pose)
    
    if self.X_button.draw(self.screen):
      self.poses_for_x.append(ball)
      print("ball:", ball)
      print("captured for X")
      self.calculate_x()
      print(self.pose)

    
    if self.Zero_button.draw(self.screen):
      self.origin_capture = ball
      print("ball:", ball)
      print("Captured for Origin")
      self.calculate_origin()
      print(self.pose)
    
    pygame.display.update()
  
  def calculate_z(self):
    if len(self.poses_for_z) < 5:
      return
    
    # https://math.stackexchange.com/questions/3869/what-is-the-intuitive-relationship-between-svd-and-pca/3871#3871
    # https://math.stackexchange.com/questions/99299/best-fitting-plane-given-a-set-of-points
    
    num_samples = len(self.poses_for_z)
    A = np.ones((num_samples, 3), dtype=float)
    B = np.zeros((num_samples, 1), dtype=float)

    points = np.stack(self.poses_for_z).T
    print('shape:', points.shape)
    svd = np.linalg.svd(points - np.mean(points, axis=1, keepdims=True))
    left = svd[0]
    normal = left[:, -1]
    if (normal[2] >0):
      normal = -1 * normal
    print(normal)

    self.pose[:3, 2] = normal.squeeze()
  
  def calculate_x(self):
    if len(self.poses_for_x) != 2:
      return
    
    p1 = self.poses_for_x[0]
    p2 = self.poses_for_x[1]
    x_axis = p1 - p2
    x_axis = x_axis/np.linalg.norm(x_axis)
    if x_axis[2] < 0:
      x_axis = -1 * x_axis
    x_axis = x_axis.squeeze()
    self.pose[:3, 0] = x_axis
    z_axis = self.pose[:3, 2]
    self.pose[:3, 1] = np.cross(z_axis, x_axis)
  
  def calculate_origin(self):
    self.pose[:3, 3] = self.origin_capture.squeeze()/1000

    camera_to_robot = np.identity(4, dtype=float)
    camera_to_robot[:3,:3] = self.pose[:3, :3].T
    camera_to_robot[:3, 3] = (-self.pose[:3, :3].T @ self.pose[:3, [3]]).squeeze()
    print("camera to robot transformation: \n", np.linalg.inv(self.pose))
    print("camera to robot transformation: \n", camera_to_robot)
    print("robot to camera transformation: \n", self.pose)

class Application:
  def __init__(self):
    self.calib = CalibrationInterface("192.168.68.68")

  def run(self):
    while True:
      self.calib.update()

if __name__ == "__main__":
  app = Application()
  app.run()
  