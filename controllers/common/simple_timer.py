# -*- coding: utf_8 -*-

class SimpleTimer(object):
    def __init__(self, sec=0, hz=1):
        self.set_time(sec)
        self.period = 1.0 / hz

    def set_time(self, sec):
        self.tm = sec

    def elapsed(self, sec):
        return sec - self.tm

    def check_elapsed(self, sec, set_if_elapsed=True):
        elapsed = self.elapsed(sec)
        if elapsed > self.period:
            if set_if_elapsed:
                self.set_time(sec)
            return (True, elapsed)
        return (False, elapsed)
