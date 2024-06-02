import multiprocessing as mp
import select
import copy
import queue

class _Broker:
  def __init__(self):
    self.subscribers = {} # topic to List[subscriber]
    self.publishers = {} # topic to publisher since there is only one publisher node for each topic
    self.matches = [] # list of (topic, publisher node, subscriber node)
    self.nodes_with_subscribers = []

  def request_subscription(self, subscriber):
    if subscriber.topic in self.subscribers:
      self.subscribers[subscriber.topic].append(subscriber)
    else:
      self.subscribers[subscriber.topic] = [subscriber]
    self._match_subscriber_to_publisher(subscriber)

  def _match_subscriber_to_publisher(self, subscriber):
    if subscriber.topic in self.publishers:
      queue = mp.Queue(subscriber.buffer_size)
      subscriber.node._add_sub_queue(queue)
      self.publishers[subscriber.topic]._add_pub_queue(queue)
      self.matches.append((subscriber.topic, self.publishers[subscriber.topic]._node_name, subscriber.node._name))
      self.nodes_with_subscribers.append(subscriber.node)

  def request_publisher(self, publisher):
    if publisher._topic in self.publishers:
      raise Exception("Multiple nodes cannot publish on a same topic")
    self.publishers[publisher._topic] = publisher
    self._match_publisher_to_subscribers(publisher)

  def _match_publisher_to_subscribers(self, publisher):
    if publisher._topic in self.subscribers:
      for subscriber in self.subscribers[publisher._topic]:
        queue = mp.Queue(subscriber.buffer_size)
        subscriber.node._add_sub_queue(queue)
        self.nodes_with_subscribers.append(subscriber.node)
        publisher._add_pub_queue(queue)
        self.matches.append((subscriber.topic, publisher._node_name, subscriber.node._name))


class _GlobalObjs:
  _broker = _Broker()
  _nodes = []
  _prints = set()

def start_subscribers():
  set_of_nodes_with_subscribers = set(_GlobalObjs._broker.nodes_with_subscribers)
  for node in set_of_nodes_with_subscribers:
    node.start()
    print(f"{node._name} started!!")


def printOnce(msg):
  if msg in _GlobalObjs._prints:
    return
  _GlobalObjs._prints.add(msg)
  print(msg)

class Node:
  def __init__(self, name):
    self._name = name
    self._sub_queues = []
    self._topic2Subscription = {}

    # just to keep the node alive without keeping
    # the instance in some main function.
    _GlobalObjs._nodes.append(self)

  def create_publisher(self, topic):
    pub = _Publisher(self._name, topic)
    _GlobalObjs._broker.request_publisher(pub)
    return pub

  def create_subscriber(self, topic, callback, buffer_size = 1):
    sub = _Subscriber(self, topic, callback, buffer_size)
    _GlobalObjs._broker.request_subscription(sub)
    if topic in self._topic2Subscription:
      raise Exception(f"Multiple subscription to single topics is not allowed in single node: (Topic: {topic})")
    self._topic2Subscription[topic] = sub
    return sub

  def _add_sub_queue(self, sub_queue):
    self._sub_queues.append(sub_queue)

  def _subscription_run(self):
    tsts = [que._reader for que in self._sub_queues]
    mmmm = {que._reader: que for que in self._sub_queues}
    while True:
      # this line only works in Linux.
      # We need a workaround for windows
      conncections, _, _ = select.select(tsts, [], [])
      for connection in conncections:
        queu = mmmm[connection]
        item = queu.get()
        topic, timestamp, msg = copy.deepcopy(item)
        del item
        # topic, timestamp, msg = copy.deepcopy(queu.get())
        subscriber = self._topic2Subscription[topic]
        subscriber.callback(timestamp, msg)

  def start(self):
    self._process = mp.Process(target=self._subscription_run)
    self._process.start()


class _Publisher:
  def __init__(self, node_name, topic):
    self._topic = topic
    self._node_name = node_name
    self._pub_queues = []

  def publish(self, timestamp, msg, block = False):
    if len(self._pub_queues) == 0:
      printOnce(f"There is no subscriber for topic {self._topic}")
      return
    for pub_queue in self._pub_queues:
        try:
          pub_queue.put((self._topic, timestamp, msg), block = block)
        except Exception:
          if not block:
            with pub_queue._rlock:
              # if full, read one to open a slot and then put one
              if pub_queue.full():
                try:
                  item = pub_queue.get(False)
                  del item
                except queue.Empty:
                  pass
                except:
                  pass
              # this works only because there is only one subscriber for each topic
              # We might need a lock for this before opening an slot
              try:
                pub_queue.put((self._topic, timestamp, msg), block = False)
                break
              except:
                pass

  def _add_pub_queue(self, queue):
    self._pub_queues.append(queue)

class _Subscriber:
  def __init__(self, node, topic, callback, buffer_size):
    self.topic = topic
    self.node = node
    self.callback = callback
    self.buffer_size = buffer_size



def test1():
  import numpy as np

  class PublisherNode(Node):
    def __init__(self):
      super().__init__("PublisherNode")
      self.publisher = self.create_publisher("test_topic")

    def run(self):
      for idx in range(3):
        print(f"publishing {idx}")
        timestamp = idx
        self.publisher.publish(timestamp, f"publisher says {idx}", block=True)

  class SubscriberNode(Node):
    def __init__(self):
      super().__init__("SubscriberNode")
      self.subscriber = self.create_subscriber("test_topic", self.callback)
      self.subscriber2 = self.create_subscriber("end_results", self.callback2)

    def callback(self, timestamp, msg):
      print(f"from subscriber 1, msg: {msg}, {timestamp}")

    def callback2(self, timestamp, msg):
      print(f"from subscriber 1, end_results: {msg}, {timestamp}")

  class SubscriberPublisherNode(Node):
    def __init__(self):
      super().__init__("SubscriberPublisherNode")
      self.subscriber = self.create_subscriber("test_topic", self.callback)
      self.publisher = self.create_publisher("end_results")

    def callback(self, timestamp, msg):
      print(f"from SubscriberPublisherNode, msg: {msg}, {timestamp}")
      self.publisher.publish(timestamp, np.random.random((3,3)), block=True)


  publisherNode = PublisherNode()
  sub = SubscriberNode()
  subPub = SubscriberPublisherNode()

  print(_GlobalObjs._broker.matches)
  print("\n\n\n")
  print(subPub.publisher.pub_queues)
  print(sub._sub_queues)

  start_subscribers()

  publisherNode.run()

if __name__ == "__main__":
  test1()