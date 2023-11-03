import numpy as np

class PID:
  def __init__(self, kp, ki, kd, output_limit = None):
    self.kp = kp
    self.ki = ki
    self.kd = kd
    self.output_limit = output_limit
    self.reset()

  def reset(self):
    self.integral = None
    self.prevInput = None

  def calculate(self, input):
    if not isinstance(input, np.ndarray):
      input = np.array((input,), dtype=float)
    proposedOutput = self._p_term(input) + self._i_term(input) + self._d_term(input)
    
    
    if not self.output_limit is None:
      # ignore the i_term. It is saturated.
      saturated_dimensions = np.logical_or(proposedOutput > self.output_limit[1], proposedOutput < self.output_limit[0])
      normal_dimensions = np.logical_not(saturated_dimensions)
      proposedOutput[saturated_dimensions] = self._p_term(input) + self._d_term(input)
      self.integral[normal_dimensions] = self._i_term(input)[normal_dimensions]
      return proposedOutput.squeeze()
    else:
      self.integral = self._i_term(input)
      
    return proposedOutput.squeeze()

  def _p_term(self, input):
    return self.kp * input

  def _d_term(self, input):
    if self.kd == 0:
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
    if self.ki == 0:
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