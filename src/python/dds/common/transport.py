"""Unitree channel adapters sharing the Cyclone domain created by ROS RMW.

The SDK factory explicitly creates a Domain with its own XML configuration,
which conflicts with rmw_cyclonedds_cpp in the same process. An implicit
participant joins the existing domain while preserving SDK channel semantics.
"""
from cyclonedds.domain import DomainParticipant
from unitree_sdk2py.core.channel import Channel

_participant = None
_domain = None


def initialize(domain):
    global _participant, _domain
    if not 1 <= domain <= 232:
        raise ValueError('Simulation DDS domain must be 1..232')
    if _participant is not None and _domain != domain:
        raise RuntimeError('Cannot switch the active simulation DDS domain')
    if _participant is None:
        _participant = DomainParticipant(domain)
        _domain = domain


class ChannelPublisher:
    def __init__(self, name, message_type):
        if _participant is None:
            raise RuntimeError('Initialize simulation DDS transport first')
        self.channel = Channel(_participant, name, message_type, None)

    def Init(self):
        self.channel.SetWriter(None)

    def Write(self, sample):
        return self.channel.Write(sample)

    def Close(self):
        self.channel.CloseWriter()


class ChannelSubscriber:
    def __init__(self, name, message_type):
        if _participant is None:
            raise RuntimeError('Initialize simulation DDS transport first')
        self.channel = Channel(_participant, name, message_type, None)

    def Init(self, handler, queue_len=0):
        self.channel.SetReader(None, handler, queue_len)

    def Close(self):
        self.channel.CloseReader()
