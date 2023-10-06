import copy
import numpy as np
import multiprocessing as mp
import time

def test1():
    class A:
        def __init__(self, name) -> None:
            self.name = name

        def __del__(self, ) -> None:
            print("deleting", self.name)


    arr =  np.random.rand(4,4)
    tpl = (124, arr, A("aaaa"), A("bbbbb"))

    q = mp.Queue(maxsize=1)


    q.put(tpl)
    print('del tpl')
    del tpl
    print(q.full())


    tpl2 = q.get()
    print(q.full())
    print('del tpl2')
    del tpl2
    print(q.full())



    time.sleep(5)


def test_BP():
    from enum import Enum
    class MissionState(Enum):
        CONTINUE = 0
        SUCCESS = 1
        FAILED = 2
    
    class MissionA:
        def __init__(self, fixArg, arg1):
            print("MissionA", fixArg, arg1)
    class MissionB:
        def __init__(self, fixArg, arg1):
            print("MissionB", fixArg, arg1)
    class MissionC:
        def __init__(self, fixArg):
            print("MissionC", fixArg)

    initMission = (MissionA, ("argu",))
    BP = {
        MissionA: {
            MissionState.SUCCESS : (MissionB, ("argument",)),
            MissionState.FAILED : (MissionC, ())
        },
    }

    fixArg = "fixArg"
    currentMission = initMission[0](fixArg, *initMission[1])
    print(currentMission)
    NextMissionEntry = BP[currentMission.__class__][MissionState.FAILED]
    currentMission = NextMissionEntry[0](fixArg, *NextMissionEntry[1])
    print(currentMission)


test_BP()