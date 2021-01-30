# -*- coding: utf_8 -*-
import json
import socket
import select
from utils import bson_serialize
from json.decoder import WHITESPACE


class RosBridgeTCP(object):
    def __init__(self, ip='127.0.0.1', port=9090, bufsize=4096, select_timeout=0.005, bson_only=False):
        self.serv_address = (ip, port)
        self.bufsize = bufsize
        self.select_timeout = select_timeout
        self.recv_data = []
        self._id_counter = 0
        self.sock = None
        # bson_only has SERIOUS RISK. It cannot send floating point numbers
        self.bson_only = bson_only
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.connect(self.serv_address)
            self.sock.setblocking(False)
        except Exception as e:
            print(str(e))
            self.sock = None

    def terminate(self):
        if self.sock is not None:
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
            if self.bson_only:
                return self.sock.send(bson_serialize(message))
            else:
                return self.sock.send(json.dumps(message).encode())
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
                    text = data.decode()
                    jsons = list(self.loads_iter(text))
                    for j in jsons:
                        self.recv_data.append(dict(j))
        except Exception as e:
            print(str(e))
            return 0
        return self.get_recv_data()

    def loads_iter(self, s):
        size = len(s)
        decoder = json.JSONDecoder()

        end = 0
        while True:
            idx = WHITESPACE.match(s[end:]).end()
            i = end + idx
            if i >= size:
                break
            ob, end = decoder.raw_decode(s, i)
            yield ob
