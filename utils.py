from constants import getGridIdxForXY, getGridMap
import numpy as np

class LogOnlyChange:
  def __init__(self) -> None:
    self._previous_message = None

  def print(self, msg):
    if self._previous_message != msg:
      print(msg)
      self._previous_message = msg
  
  def __call__(self, msg):
    return self.print(msg)

def find_path(source, destination):
  gridMap = getGridMap()


def calculateTarget_LookAhead(currentPosition, destination):
  lookAheadDistance = 0.20 # meters
  distanceToTarget = np.linalg.norm(destination - currentPosition)
  lookAheadDistance = min(lookAheadDistance, distanceToTarget)
  return currentPosition + (destination - currentPosition) / distanceToTarget * lookAheadDistance


def calculateLinearAndRotationalError(currentPose, targetPosition):
  """Calculates the linear (distance) and rotational error to target
  """
  currentPosition = currentPose[:2, 3]
  currentOrientationX = currentPose[:2, 0]
  relativeDestination = targetPosition - currentPosition
  distance = np.linalg.norm(relativeDestination)

  # signed_angle = atan2(b.y,b.x) - atan2(a.y,a.x)
  rotationalError = np.arctan2(relativeDestination[1], relativeDestination[0]) - \
      np.arctan2(currentOrientationX[1], currentOrientationX[1])

  return distance, rotationalError
