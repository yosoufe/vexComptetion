# to delete later
# we need it in at least one commit

import cv2
import numpy as np
import time
import pygame
import sys
from cortano import lan
from middleware import Node
from constants import Topics
import multiprocessing as mp

class RemoteInterfaceNode(Node):
  def __init__(self, host="0.0.0.0", port=9999, subscribeForMotorCommands = False):
    """Remote Interface showing the data coming in from the robot

    Args:
        host (str, optional): host ip of the robot. Defaults to "0.0.0.0".
    """
    super().__init__("RemoteInterfaceNode")
    lan.start(host, port, frame_shape=(360, 640))
    self.motor_vals = np.zeros((10,), np.int32)
    self.sensor_vals = np.zeros((20,), np.int32)

    pygame.init()
    self.screen = pygame.display.set_mode((1280, 720))
    self.clock = pygame.time.Clock()
    self.screen.fill((63, 63, 63))

    self.color = None
    self.depth = None

    self.keys = {k[2:]: 0 for k in dir(pygame) if k.startswith("K_")}
    self.keynames = list(self.keys.keys())

    self.free_frame1 = np.zeros((360, 640, 3), np.uint8)
    self.free_frame2 = np.zeros((360, 640, 3), np.uint8)
    
    if subscribeForMotorCommands:
      self.motorCommandsSub = self.create_subscriber(Topics.motorCommands, self.motorCmds_cb)
    self.subscribeForMotorCommands = subscribeForMotorCommands
    self.motor_queue = mp.Queue(maxsize= 1)
  
  def motorCmds_cb(self, timestamp, motorCommands):
    try:
      # this function runs in different process
      # Because of the callback above for the subscriber
      self.motor_queue.put_nowait(motorCommands)
    except:
      pass

    # open3d to viz rgb and depth

  def __del__(self):
    lan.stop()

  def disp1(self, frame):
    """Set an optional output frame to view in disp 1

    Args:
        frame (np.ndarray): frame sized (360, 640, 3) that can be displayed in real time
    """
    self.free_frame1 = frame

  def disp2(self, frame):
    """Set an optional output frame to view in disp 2

    Args:
        frame (np.ndarray): frame sized (360, 640, 3) that can be displayed in real time
    """
    self.free_frame2 = frame
  
  def depth2rgb(self, depth):
    """Turn a depth frame into a viewable rgb frame

    Args:
        depth (np.ndarray): depth frame

    Returns:
        np.ndarray: depth frame as color
    """
    return cv2.applyColorMap(np.sqrt(depth).astype(np.uint8), cv2.COLORMAP_JET)
  
  @property
  def motor(self):
    return self.motor_vals
  
  @motor.setter
  def motor(self, value):
    self.motor_vals = value
  
  def sensor(self, idx):
    return self.sensor_vals[idx]
  
  def read(self):
    """Read sensor values from the robot, including color and depth

    Returns:
        (np.ndarray, np.ndarray, np.ndarray): color, depth, other sensor values
    """
    return self.color, self.depth, np.copy(self.sensor_vals)
  
  def update(self):
    """Update the robot by receiving information over WiFi
    """
    for event in pygame.event.get():
      if event.type == pygame.QUIT:
        lan.stop()
        sys.exit(0)
      elif event.type == pygame.KEYDOWN:
        for keycode in self.keys.keys():
          if event.key == getattr(pygame, f"K_{keycode}"):
            self.keys[keycode] = 1
      elif event.type == pygame.KEYUP:
        for keycode in self.keys.keys():
          if event.key == getattr(pygame, f"K_{keycode}"):
            self.keys[keycode] = 0

    self.color, self.depth = lan.get_frame()
    if self.color is not None and self.depth is not None:
      depthrgb = self.depth2rgb(self.depth)
      a = (np.swapaxes(np.flip(self.free_frame1, axis=-1), 0, 1))
      b = (np.swapaxes(np.flip(self.free_frame2, axis=-1), 0, 1))
      c = (np.swapaxes(np.flip(self.color, axis=-1), 0, 1))
      d = (np.swapaxes(np.flip(depthrgb, axis=-1), 0, 1))
      a = pygame.surfarray.make_surface(a)
      b = pygame.surfarray.make_surface(b)
      c = pygame.surfarray.make_surface(c)
      d = pygame.surfarray.make_surface(d)
      self.screen.blit(c, (0, 0))
      self.screen.blit(d, (640, 0))
      self.screen.blit(a, (0, 360))
      self.screen.blit(b, (640, 360))

    if self.subscribeForMotorCommands:
      try:
        self.motor_vals = self.motor_queue.get_nowait()
      except:
        pass
    
    lan.write(self.motor)

    self.sensor_vals = lan.read()

    pygame.display.flip()