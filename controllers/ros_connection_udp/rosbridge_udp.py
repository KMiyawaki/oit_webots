# -*- coding: utf_8 -*-
import json
import socket
import select

try:
    from collections import UserDict
except ImportError:
    from UserDict import UserDict


class Message(UserDict):
    def __init__(self, values=None):
        self.data = {}
        if values is not None:
            self.update(values)


class RosBridgeUDP(object):
    def __init__(self, ip='127.0.0.1', port=9090, bufsize=4096, select_timeout=0.005):
        self.serv_address = (ip, port)
        self.bufsize = bufsize
        self.select_timeout = select_timeout
        self.recv_data = []
        self._id_counter = 0
        self.sock = None
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sock.setblocking(False)
        except Exception as e:
            print(str(e))
            self.sock = None

    def terminate(self):
        pass

    def run(self):
        pass

    @property
    def is_connected(self):
        return self.sock is not None

    @property
    def id_counter(self):
        self._id_counter += 1
        return self._id_counter

    def send_message(self, message):
        try:
            return self.sendto(json.dumps(message.data))
        except Exception as e:
            print(str(e))
            return 0

    def sendto(self, data):
        try:
            return self.sock.sendto(data.encode('utf-8'), self.serv_address)
        except Exception as e:
            print(str(e))
            return 0

    def get_recv_data(self):
        return self.recv_data

    def wait(self):
        self.recv_data = []
        try:
            readfds = set([self.sock])
            rready, _, _ = select.select(readfds, [], [], self.select_timeout)
            for sock in rready:
                if sock != self.sock:
                    print("Warning: UnKnow socket")
                else:
                    data, _ = self.sock.recvfrom(self.bufsize)
                    self.recv_data.append(dict(json.loads(data)))
        except Exception as e:
            print(str(e))
            return 0
        return self.get_recv_data()
