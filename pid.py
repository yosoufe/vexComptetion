import numpy as np
import matplotlib.pyplot as plt

class PlotterForControl:
  def __init__(self, title = None, legend = None):
    self.fig = plt.figure(figsize=(8,6))
    self.ax = self.fig.add_subplot(1,1,1)
    if title is not None:
      self.fig.suptitle(title)
    if legend is not None:
      self.legend = legend
    else:
      self.legend = None
    plt.ion()
    plt.show()
    self.data = None
  
  def plot(self, inp):
    if self.data is None:
      self.data = np.zeros((1, inp.shape[0]))
      self.data[0] = inp
    if self.legend is None:
      self.legend = [f"{idx}" for idx in range(inp.shape[0])]
    else:
      # print("plotter", self.data.shape, inp.shape)
      self.data = np.vstack((self.data, inp))
    
    if self.data.shape[0] % 20 == 0:
      self.ax.clear()
      self.ax.plot(self.data)
      self.data = self.data[-1000:]
      self.ax.grid()
      self.ax.legend(self.legend)
      self.fig.canvas.draw()
      self.fig.canvas.flush_events()

class PID:
  def __init__(self, kp, ki, kd, output_limit = None, withPlot = False):
    self.kp = kp
    self.ki = ki
    self.kd = kd
    self.output_limit = output_limit
    self.reset()
    self.withPlot = withPlot
    if self.withPlot:
      self.plotter = PlotterForControl(
        title="pid",
        legend=["input", "control", "P", "d", "I"])

  def reset(self):
    self.integral = None
    self.prevInput = None

  def calculate(self, input):
    if not isinstance(input, np.ndarray):
      input = np.array((input,), dtype=float)
    p_term = self._p_term(input)
    d_term = self._d_term(input)
    i_term = self._i_term(input)
    proposedOutput = p_term + i_term + d_term
    
    
    if not self.output_limit is None:
      # ignore the i_term. It is saturated.
      saturated_dimensions = np.logical_or(proposedOutput > self.output_limit[1], proposedOutput < self.output_limit[0])
      normal_dimensions = np.logical_not(saturated_dimensions)
      # don't update the integral for saturated dimensions
      proposedOutput[saturated_dimensions] = p_term + d_term + self.integral[saturated_dimensions]
      self.integral[normal_dimensions] = i_term[normal_dimensions]
    else:
      self.integral = i_term
    
    if self.withPlot:
      if isinstance(d_term, np.ndarray):
        d = d_term.squeeze()
      else:
        d = d_term
      # toPlot = np.array([np.rad2deg(input).squeeze(),proposedOutput.squeeze(), p_term.squeeze(), d, i_term.squeeze()])
      toPlot = np.array([39.3701*input.squeeze(),proposedOutput.squeeze(), p_term.squeeze(), d, i_term.squeeze()])
      self.plotter.plot(toPlot)
      
    return proposedOutput.squeeze()

  def _p_term(self, input):
    return self.kp * input

  def _d_term(self, input):
    if np.linalg.norm(self.kd) == 0:
      return 0

    if self.prevInput is None:
      self.prevInput = input
      return 0
    
    cmd = (input - self.prevInput) * self.kd
    self.prevInput = input
    return cmd

  def _i_term(self, input):
    if self.integral is None:
      self.integral = np.zeros_like(input)
    if np.linalg.norm(self.ki) == 0:
      return np.zeros_like(input)
    return self.integral + input * self.ki


def test_scalar():
  pid = PID(10, 0.2, 1, [-5, 8])
  pid.reset()
  print(pid.calculate(2))
  pid = PID(10, 0.2, 1)
  pid.reset()
  print(pid.calculate(2))

def test_vector():
  pid = PID(10, 0.2, 1, np.array([[-5, -5], [8, 8]], dtype=float))
  pid.reset()
  print(pid.calculate(np.array(([1,2]),dtype=float)))

  pid = PID(10, 0.2, 1)
  pid.reset()
  print(pid.calculate(np.array(([1,2]),dtype=float)))
  print(pid.calculate(np.array(([0.5,1.5]),dtype=float)))

def test_vector_with_vector_constants():
  pid = PID(np.array([2,3],dtype=float), 0, 0)
  pid.reset()

  print(pid.calculate(np.array(([1,2]),dtype=float)))
  print(pid.calculate(np.array(([-1,-3]),dtype=float)))

if __name__ == "__main__":
  # test_scalar()
  # test_vector()
  test_vector_with_vector_constants()