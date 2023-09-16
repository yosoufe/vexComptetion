from cortano import RemoteInterface
from constants import Topics
from middleware import Node

class RemoteInterfaceNodeMultiprocessorSafe(RemoteInterface):
  def __init__(self, motorCmdQueue, host="0.0.0.0", port=9999):
    super().__init__(host, port)
    self.motorCmdQueue = motorCmdQueue
  
  @property
  def motor(self):
    try:
      self.motor_vals = self.motorCmdQueue.get_nowait()
    except:
      pass
    return self.motor_vals

class RemoteInterfaceNode(Node):
  def __init__(self, queue):
    super().__init__("RemoteInterfaceNode")
    self.create_subscriber(Topics.motorCommands, self.motorCmds_cb)
    self.motorCmds_queue = queue

  def motorCmds_cb(self, timestamp, motorCommands):
    try:
      # this function runs in different process
      # Because of the callback above for the subscriber
      self.motorCmds_queue.put_nowait(motorCommands)
    except:
      pass