# circulaRprint                                 #
# Circular Plotter controller for Raspberry Pi  #
# Author: Oehrly, 2018                          #
#################################################
#
# main controller class
#
# It is intended that this class is imported and started
# from a separate program/script.
# Take a look at run.py for basic usage.

from hardware import cleanup_gpio
from gcodeparser import GcodeParser
from timingplanner import SegmentAccelerationPlanner, StepAccelerationPlanner
from pathplanner import PathPlanner
from config import config as defaultconfig

from math import sqrt
import time

import platform

# enables you to just run it on a windows machine with out getting errors as it probably has no GPIO Pins
# TODO detect Raspberry, not OS
if platform.system() == 'Windows':
    from hardware import FakeBipolarStepperMotor as BipolarStepperMotor
else:
    from hardware import BipolarStepperMotor


######################################
# some functions for debugging purpose


def write_segment_to_file(controller):
    dist_overall = 0
    with open('misc/timing_v3_1.txt', 'w') as fobj:
        fobj.write('Speed, X, Y, Dist (Seg), Dist (Overall)\n')
        for gcmd in controller.gcommands:
            for ps in gcmd.path:
                dist_overall += float(ps.distance)
                fobj.write(
                    str(sqrt(ps.entry_speed_sqr)) + ',' + str(ps.x) + ',' + str(ps.y) + ',' + str(ps.distance) + ',' +
                    str(dist_overall) + '\n')

        fobj.close()


def write_steps_to_file(controller):
    with open('misc/timing_v3_2.txt', 'w') as fobj:
        fobj.write('Timing B\n')
        for gcmd in controller.gcommands:
            for ps in gcmd.path:
                for tmg in ps.b_timing:
                    fobj.write(str(tmg)+'\n')
        fobj.close()

##########################################


class Controller:
    # main controller class for plotter
    def __init__(self, config=None, mode=None):
        if config:
            self.config = config
        else:
            self.config = defaultconfig

        # motor classes
        self.MA = BipolarStepperMotor(self.config['MA'], full_step=self.config['MA_full_step_seq'],
                                      backlash_correction=self.config['MA_backlash_correction'][0],
                                      backlash_correction_delay=self.config['MA_backlash_correction'][1],
                                      motor_dir=self.config['MA_dir'], max_speed=self.config['a_feed_rate'],
                                      always_unhold=self.config['MA_always_unhold'])

        self.MB = BipolarStepperMotor(self.config['MB'], full_step=self.config['MB_full_step_seq'],
                                      backlash_correction=self.config['MB_backlash_correction'][0],
                                      backlash_correction_delay=self.config['MB_backlash_correction'][1],
                                      motor_dir=self.config['MB_dir'], max_speed=self.config['b_feed_rate'],
                                      always_unhold=self.config['MB_always_unhold'])

        self.MZ = BipolarStepperMotor(self.config['MZ'], full_step=self.config['MZ_full_step_seq'],
                                      backlash_correction=self.config['MZ_backlash_correction'][0],
                                      backlash_correction_delay=self.config['MZ_backlash_correction'][1],
                                      motor_dir=self.config['MZ_dir'], max_speed=self.config['z_feed_rate'],
                                      always_unhold=self.config['MZ_always_unhold'])

        self.gcommands = list()
        self.gcommand_index = int(0)

        self.gcodeparser = GcodeParser()

        self.PathPlanner = PathPlanner(self.config, source=self)
        self.SegAP = SegmentAccelerationPlanner(self.config, provider=self.PathPlanner)
        self.StepAP = StepAccelerationPlanner(self.config, provider=self.SegAP)

        if not mode:
            self.mode = int(self.config['Default_mode'])
        else:
            self.mode = int(mode)

        if mode == 0:
            from segmentbuffers import SegmentBufferPreCalc
            self.SegBuffer = SegmentBufferPreCalc(provider=self.StepAP)
            self.provider = self.SegBuffer

        elif mode == 1:
            from segmentbuffers import SegmentBufferThread
            self.SegBuffer = SegmentBufferThread(provider=self.StepAP)
            self.provider = self.SegBuffer

    def load_gcode_file(self, filename):
        self.gcommands = self.gcodeparser.file(filename)
        # returns a list of gcommand objects (see gcodeparser.py)

        if 'debugout' in self.config:
            # pass gcode on to debugout
            self.config['debugout'].gcode(self.gcommands)

    def serve_next(self):
        # serve next gcommand to the class requesting it (normally pathplaner)
        # rprint [gcommand] -> pathplanner [segment] -> SegmentAP -> StepAP -> rprint.run
        # each class in this chain requests next segment/gcommand by calling provider.serve_next
        # provider is always the class before

        if self.gcommand_index < len(self.gcommands):
            gcommand = self.gcommands[self.gcommand_index]
            self.gcommand_index += 1
            return gcommand
        else:
            return None

    def run(self):
        # initiate provider
        self.provider.start()

        # debugging functions
        # write_steps_to_file(self)
        # write_segment_to_file(self)

        t1 = time.time()

        try:
            while True:
                segment = self.provider.serve_next()
                if not segment:
                    break

                if not segment.gcommand:
                    break

                # check if setup or movement command
                # setup commands (Mxx) are not implemented
                if not segment.gcommand.setup:
                    if segment.b_steps:
                        self.MB.move_timed(segment.b_steps, segment.b_timing)
                    if segment.a_steps:
                        self.MA.move_timed(segment.a_steps, segment.a_timing)
                    if segment.z_steps:
                        self.MZ.move(segment.z_steps, delay=segment.z_delay)

        except KeyboardInterrupt:
            self.move_home()
            self.shutdown()

        t2 = time.time()

        print("Runtime: " + str(round(t2 - t1, 1)))

    def find_center(self):  # turns the table back and forth a bit to check whether tool tip is centered
        self.MA.move(500)
        self.MA.move(-500)

    def shutdown(self):
        # shuts down everything (immediately!) without moving home
        # do move_home() first for proper non-emergency shutdown
        self.MA.unhold()
        self.MB.unhold()
        cleanup_gpio()

    def move_home(self):
        # at the moment only used for panic home movement
        # very dirty way but doesn't rely on any other part of the controller (except hardware.py)
        # relying on as few code as possible is the intention
        # this won't go in a straight line and might take some time and unnecessary table rotations

        a = -self.MA.position * self.MA.MOTOR_DIR
        b = -self.MB.position * self.MB.MOTOR_DIR
        z = -self.MZ.position * self.MZ.MOTOR_DIR

        self.MA.move(a, delay=0.003)
        self.MB.move(b)
        self.MZ.move(z, 0.005)
