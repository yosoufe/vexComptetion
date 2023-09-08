class PID:
  def __init__(self, kp, ki, kd, output_limit = None):
    self.kp = kp
    self.ki = ki
    self.kd = kd
    self.output_limit = output_limit
    self.reset()

  def reset(self):
    self.integral = 0
    self.prevInput = None

  def calculate(self, input):
    return self._p_term(input) + self._i_term(input) + self._d_term(input)

  def _p_term(self, input):
    return self.kp * input

  def _i_term(self, input):
    if self.kd == 0:
      return 0

    if self.prevInput is None:
      self.prevInput = input
      return 0
    
    cmd = (input - self.prevInput) * self.kd
    self.prevInput = input
    return cmd

  def _d_term(self, input):
    if self.ki == 0:
      return 0
    
    # anti wind-up against saturation
    proposed_integral = self.integral + input * self.ki
    if self.output_limit != None:
      if proposed_integral > self.output_limit[1] or proposed_integral < self.output_limit[0]:
        return self.integral
    
    self.integral = proposed_integral
    return self.integral
