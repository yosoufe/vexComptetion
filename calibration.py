from cortano import RemoteInterface
import pygame
import numpy as np

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
    pygame.display.update()

    return self.handle_click()

    

class CalibrationInterface(RemoteInterface):
  def __init__(self, host="0.0.0.0",):
    super().__init__(host)
    self.Z_button = Button(x = 10, y = 370, w= 400, h=30, text= "Capture Ball Position For Z")
    self.X_button = Button(x = 10, y = 410, w= 400, h=30, text= "Capture Ball Position For X")
    self.Zero_button = Button(x = 10, y = 450, w= 400, h=30, text= "Capture Ball Position For Origin")
    

    self.poses_for_z = []
    self.poses_for_x = []
    self.pose = np.zeros((4,4), dtype=float)
  
  def update(self):
    super().update()
    color, depth, _ = self.calib.read()
    
    if self.Z_button.draw(self.screen):
      print("Z clicked")
    
    if self.X_button.draw(self.screen):
      print("X clicked")
    
    if self.Zero_button.draw(self.screen):
      print("Origin clicked")
    
  

class Application:
  def __init__(self):
    self.calib = CalibrationInterface("192.168.68.68")

  def run(self):
    while True:
      self.calib.update()

if __name__ == "__main__":
  app = Application()
  app.run()
  