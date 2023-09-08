
import multiprocessing as mp
import select

class Broker:
  def __init__(self):
    self.subscribers = {} # topic to [subscriber]
    self.publishers = {} # topic to publisher since there is one publisher node for each topic
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
      self.matches.append((subscriber.topic, self.publishers[subscriber.topic].node_name, subscriber.node._name))
      self.nodes_with_subscribers.append(subscriber.node)
  
  def request_publisher(self, publisher):
    if publisher.topic in self.publishers:
      raise Exception("Multiple nodes cannot publish on a same topic")
    self.publishers[publisher.topic] = publisher
    self._match_publisher_to_subscribers(publisher)
  
  def _match_publisher_to_subscribers(self, publisher):
    if publisher.topic in self.subscribers:
      for subscriber in self.subscribers[publisher.topic]:
        queue = mp.Queue(subscriber.buffer_size)
        subscriber.node._add_sub_queue(queue)
        self.nodes_with_subscribers.append(subscriber.node)
        publisher._add_pub_queue(queue)
        self.matches.append((subscriber.topic, publisher.node_name, subscriber.node._name))


class GlobalObjs:
  broker = Broker()

def start_subscribers():
  set_of_nodes_with_subscribers = set(GlobalObjs.broker.nodes_with_subscribers)
  for node in set_of_nodes_with_subscribers:
    node.start()


class Connection:
  def __init__(self, topic, queue):
    self.topic = topic
    self.queue = queue

class Node:
  def __init__(self, name):
    self._name = name
    self._sub_queues = []
    self._topic2Subscription = {}

  def create_publisher(self, topic):
    pub = Publisher(self._name, topic)
    GlobalObjs.broker.request_publisher(pub)
    return pub

  def create_subscriber(self, topic, callback, buffer_size = 1):
    sub = Subscriber(self, topic, callback, buffer_size)
    GlobalObjs.broker.request_subscription(sub)
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
      conncections, _, _ = select.select(tsts, [], [])
      for connection in conncections:
        queu = mmmm[connection] 
        topic, msg = queu.get()
        subscriber = self._topic2Subscription[topic]
        subscriber.callback(msg)
  
  def start(self):
    self._process = mp.Process(target=self._subscription_run)
    self._process.start()


class Publisher:
  def __init__(self, node_name, topic):
    self.topic = topic
    self.node_name = node_name
    self.pub_queues = []

  def publish(self, msg, block = True):
    if len(self.pub_queues) == 0:
      print("there is no subscriber for topic", self.topic)
      return
    for pub_queue in self.pub_queues:
      try:
        pub_queue.put((self.topic, msg), block = block)
      except Exception:
        pass

  def _add_pub_queue(self, queue):
    self.pub_queues.append(queue)

class Subscriber:
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
        self.publisher.publish(f"publisher says {idx}", block=True)
    
  class SubscriberNode(Node):
    def __init__(self):
      super().__init__("SubscriberNode")
      self.subscriber = self.create_subscriber("test_topic", self.callback)
      self.subscriber2 = self.create_subscriber("end_results", self.callback2)
    
    def callback(self, msg):
      print(f"from subscriber 1, msg: {msg}")

    def callback2(self, msg):
      print(f"from subscriber 1, end_results: {msg}")
  
  class SubscriberPublisherNode(Node):
    def __init__(self):
      super().__init__("SubscriberPublisherNode")
      self.subscriber = self.create_subscriber("test_topic", self.callback)
      self.publisher = self.create_publisher("end_results")
    
    def callback(self, msg):
      print(f"from SubscriberPublisherNode, msg: {msg}")
      self.publisher.publish(np.random.random((3,3)), block=True)
    
  
  publisherNode = PublisherNode()
  sub = SubscriberNode()
  subPub = SubscriberPublisherNode()

  print(GlobalObjs.broker.matches)
  print("\n\n\n")
  print(subPub.publisher.pub_queues)
  print(sub._sub_queues)

  start_subscribers()

  publisherNode.run()
  
if __name__ == "__main__":
  test1()