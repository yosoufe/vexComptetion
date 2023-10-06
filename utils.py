from constants import getGridIdxForXY, getGridMap
import numpy as np
from inspect import getframeinfo, stack

def debuginfo(message):
    caller = getframeinfo(stack()[1][0])
    print("%s:%d - %s" % (caller.filename, caller.lineno, message)) # python3 syntax print

class LogOnlyChange:
  def __init__(self) -> None:
    self._previous_messages = dict()
  
  def __call__(self, msg):
    caller = getframeinfo(stack()[1][0])
    if not (caller.filename, caller.lineno) in self._previous_messages:
      self._previous_messages[(caller.filename, caller.lineno)] = msg
      # print(f"{caller.filename}:{caller.lineno} , {msg}")
      print(f"{msg}")
    else:
      if self._previous_messages[(caller.filename, caller.lineno) ] != msg:
        self._previous_messages[(caller.filename, caller.lineno) ] = msg
        # print(f"{caller.filename}:{caller.lineno} , {msg}")
        print(f"{msg}")


def find_path(source, destination):
  gridMap = getGridMap()


def calculateTarget_LookAhead(currentPosition, destination, lookAheadDistance = 0.20):
  distanceToTarget = np.linalg.norm(destination - currentPosition)
  lookAheadDistance = min(lookAheadDistance, distanceToTarget)
  isClose = abs(distanceToTarget - lookAheadDistance) < 0.01
  return currentPosition + (destination - currentPosition) / distanceToTarget * lookAheadDistance, isClose


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
