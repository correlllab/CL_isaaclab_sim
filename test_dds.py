#!/usr/bin/env python3

import time

from cyclonedds.domain import DomainParticipant
from cyclonedds.pub import Publisher, DataWriter
from cyclonedds.topic import Topic
from cyclonedds.idl import IdlStruct
from cyclonedds.idl.annotations import final
from cyclonedds.idl.types import char


@final
class CharMsg(IdlStruct):
    data: char


participant = DomainParticipant()
publisher = Publisher(participant)
topic = Topic(participant, "chatter", CharMsg)
writer = DataWriter(publisher, topic)

count = 0

while True:
    msg = CharMsg()
    msg.data = "a"
    writer.write(msg)
    print(f"Published: {msg.data}")
    count += 1
    time.sleep(1.0)  # 1 Hz
