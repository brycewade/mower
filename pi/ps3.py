#!/usr/bin/env python3
import fcntl
import os
import struct


class Joystick:
    def __init__(self, file="/dev/input/js0"):
        # Open the file non-blocking
        self.f = open(file, "rb")
        fd = self.f.fileno()
        flag = fcntl.fcntl(fd, fcntl.F_GETFL)
        fcntl.fcntl(fd, fcntl.F_SETFL, flag | os.O_NONBLOCK)
        flag = fcntl.fcntl(fd, fcntl.F_GETFL)
        # read through any existing events
        trash = self.f.read()
        self.axis = [0] * 6
        self.button = [0] * 17


    def __exit__(self):
        self.f.close()


    def process_events(self):
        input = self.f.read(8)
        while input:
            e_time, e_value, e_type, e_number = struct.unpack("IhBB", input)
            if e_type == 2:
                self.axis[e_number] = e_value
            if e_type == 1:
                self.button[e_number] = e_value
            input = self.f.read(8)


    def get_axis(self, axis):
        return self.axis[axis]


    def get_button(self, button):
        return self.button[button]