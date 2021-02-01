# -*- coding: utf_8 -*-
import socket
import select
from utils import bson_serialize


class RosBridgeSIGVerse(object):
    def __init__(self, ip='127.0.0.1', port=50001, bufsize=4096):
        self.serv_address = (ip, port)
        self.bufsize = bufsize
        self._id_counter = 0
        self.sock = None
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.connect(self.serv_address)
            self.sock.setblocking(False)
        except Exception as e:
            print(str(e))
            self.sock = None

    def terminate(self):
        self.sock.close()
        self.sock = None

    @property
    def is_connected(self):
        return self.sock is not None

    @property
    def id_counter(self):
        self._id_counter += 1
        return self._id_counter

    def send_message(self, message):
        try:
            return self.sock.send(bson_serialize(message))
        except Exception as e:
            print(str(e))
            return 0
