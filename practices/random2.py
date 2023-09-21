import copy
import numpy as np
import multiprocessing as mp
import time

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
