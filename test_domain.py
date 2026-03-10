from cyclonedds.topic import Topic
from cyclonedds.idl import IdlStruct, IdlEnum
from cyclonedds.sub import DataReader, Qos, Listener, Subscriber
import sys
sys.path.append("/home/code")
from unitree_sdk2py.idl.std_msgs.msg.dds_._String_ import String_
from cyclonedds.domain import Domain, DomainParticipant
from cyclonedds.builtin import BuiltinDataReader, BuiltinTopic
from cyclonedds.core import *
from test_idl_type import TestMsgType
test_dp = DomainParticipant(0)

qos = Qos()
listener = Listener()
test_topic = Topic(test_dp, "test_dds_topic", TestMsgType, qos=qos, listener=listener)
test_sub = Subscriber(test_dp, qos, listener)
test_dr = DataReader(test_sub, test_topic, qos, listener)
import time
while True:
    print(test_dr.read())
    time.sleep(1)

#reader = DataReader(test_dp, test_topic, qos=qos, listener=listener)
#print(reader.take(N=2))

