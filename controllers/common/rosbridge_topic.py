# -*- coding: utf_8 -*-
from rosbridge_udp import Message


class RosBridgeTopic(object):
    def __init__(self, rosbridge_udp, name, message_type, compression=None, latch=False, throttle_rate=0,
                 queue_size=100, queue_length=0, reconnect_on_close=True):
        self.ros = rosbridge_udp
        self.name = name
        self.message_type = message_type
        self.compression = compression
        self.latch = latch
        self.throttle_rate = throttle_rate
        self.queue_size = queue_size
        self.queue_length = queue_length

        self._subscribe_id = None
        self._advertise_id = None

        if self.compression is None:
            self.compression = 'none'

    @property
    def is_advertised(self):
        return self._advertise_id is not None

    @property
    def is_subscribed(self):
        return self._subscribe_id is not None

    def advertise(self):
        if self.is_advertised:
            print("Error: topic %s is already advitesed." % self.name)
            return False
        try:
            self._advertise_id = 'advertise:%s:%d' % (
                self.name, self.ros.id_counter)

            data = {
                'op': 'advertise',
                'id': self._advertise_id,
                'type': self.message_type,
                'topic': self.name,
                'latch': self.latch,
                'queue_size': self.queue_size
            }

            self.ros.send_message(Message(data))
            return True
        except Exception as e:
            print(str(e))
        return False

    def publish(self, message):
        data = {
            'op': 'publish',
            'id': self._advertise_id,
            'topic': self.name,
            'msg': dict(message),
            'latch': self.latch
        }
        self.ros.send_message(Message(data))

    def subscribe(self):
        if self._subscribe_id:
            print("Error: this topic already in use.")
            return False

        self._subscribe_id = 'subscribe:%s:%d' % (
            self.name, self.ros.id_counter)

        data = {
            'op': 'subscribe',
            'id': self._subscribe_id,
            'type': self.message_type,
            'topic': self.name,
            'compression': self.compression,
            'throttle_rate': self.throttle_rate,
            'queue_length': self.queue_length
        }
        self.ros.send_message(Message(data))

    def get_messages_from_queue(self):
        # {'topic': '/chatter', 'msg': {'data': 'hoge'}, 'op': 'publish'}
        messages = []
        recv_data = self.ros.get_recv_data()
        for d in recv_data:
            if d['topic'] == self.name:
                messages.append(d['msg'])
        return messages
