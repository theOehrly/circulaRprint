# circulaRprint                                 #
# Circular Plotter controller for Raspberry Pi  #
# Author: Oehrly, 2018                          #
#################################################
#
# the two segment buffers allow for two different ways of running
# either calculate all movement before running so the pi can focus on more accurate timing when running
# or calculate movement in a separate thread while running, higher cpu load might cause worse timing

import _thread
import time


class SegmentBufferThread:
    def __init__(self, provider):
        self.provider = provider
        self.target_buffer_size = 50

        self.segments = list()

    def start(self):
        _thread.start_new_thread(self.update, ())

        print("Preparing buffer...")
        while len(self.segments) < self.target_buffer_size:
            pass
        print("Target buffer size reached")

        return

    def update(self):
        while True:
            if len(self.segments) < self.target_buffer_size:
                segment = self.provider.serve_next()
                if segment.gcommand:
                    self.segments.append(segment)
                else:
                    break
            else:
                time.sleep(0.1)

    def serve_next(self):
        if self.segments:
            return self.segments.pop(0)
        else:
            return None


class SegmentBufferPreCalc:
    def __init__(self, provider):
        self.provider = provider

        self.segments = list()

    def start(self):
        print("Processing gcode... This might take some time")
        while True:
            segment = self.provider.serve_next()
            if segment.gcommand:
                self.segments.append(segment)
            else:
                break

    def serve_next(self):
        if self.segments:
            return self.segments.pop(0)
        else:
            return None
