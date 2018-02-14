# circulaRprint                                 #
# Circular Plotter controller for Raspberry Pi  #
# Author: Oehrly, 2018                          #
#################################################
#
# stepper motor interface class
# and fake class for simulation runs on non-pi hardware

import time
import RPi.GPIO as GPIO
from mathfunc import sign

# sequence for A1, B2, A2, B1
full_step_seq = [[1, 1, 0, 0], [0, 1, 1, 0], [0, 0, 1, 1], [1, 0, 0, 1]]
# full step sequence. maximum torque
half_step_seq = [[1, 0, 0, 0], [1, 1, 0, 0], [0, 1, 0, 0], [0, 1, 1, 0], [0, 0, 1, 0], [0, 0, 1, 1], [0, 0, 0, 1],
                 [1, 0, 0, 1]]


# half-step sequence. double resolution. But the torque of the stepper motor is not constant
# num_phase = len(PHASE_SEQ)


def cleanup_gpio():
    GPIO.cleanup()


class BipolarStepperMotor:
    def __init__(self, pins, motor_dir=1, full_step=False, backlash_correction=0, backlash_correction_delay=0,
                 max_speed=0, always_unhold=False):
        # initiate a Bipolar_Stepper_Motor object

        # setup GPIO pins
        GPIO.setmode(GPIO.BCM)

        self.A1, self.A2, self.B1, self.B2 = pins

        GPIO.setup(self.A1, GPIO.OUT)
        GPIO.setup(self.A2, GPIO.OUT)
        GPIO.setup(self.B1, GPIO.OUT)
        GPIO.setup(self.B2, GPIO.OUT)

        # make sure all outputs are low for now
        self.unhold()

        # configure which step sequence is used
        if full_step:
            self.PHASE_SEQ = full_step_seq
        else:
            self.PHASE_SEQ = half_step_seq
        self.num_phase = len(self.PHASE_SEQ)
        self.phase = 0

        # set some more configuration values and create some variables
        self.BACKLASH_CORRECTION = backlash_correction
        self.BACKLASH_CORRECTION_DELAY = backlash_correction_delay
        self.MOTOR_DIR = motor_dir
        self.MIN_DELAY = 1 / max_speed
        self.ALWAYS_UNHOLD = always_unhold

        self.direction = 0
        self.position = 0

        self.last_time = 0

    def move(self, steps, delay=0.01):
        # moves stepper motor with same timing for each step
        if not steps:
            return

        new_direction = sign(steps) * self.MOTOR_DIR

        # check for change of direction and do backlash correction
        if not new_direction == 0:
            if new_direction != self.direction:
                self.direction = new_direction
                steps = abs(steps) + self.BACKLASH_CORRECTION
            else:
                self.direction = new_direction
                steps = abs(steps)

        # move by the number of steps specified
        for _ in range(steps):
            self.position += self.direction
            next_phase = (self.phase + self.direction) % self.num_phase
            GPIO.output(self.A1, self.PHASE_SEQ[next_phase][0])
            GPIO.output(self.B2, self.PHASE_SEQ[next_phase][1])
            GPIO.output(self.A2, self.PHASE_SEQ[next_phase][2])
            GPIO.output(self.B1, self.PHASE_SEQ[next_phase][3])
            self.phase = next_phase
            time.sleep(delay)

        if self.ALWAYS_UNHOLD:
            # set all outputs low after every movement
            self.unhold()

    def move_timed(self, steps, timing_list):
        # moves the stepper motor with different timing for each step
        # timing_list contains multiple floats (or ints), the number is the delay between steps in seconds
        if not steps:
            return

        new_direction = sign(steps) * self.MOTOR_DIR

        # check for change of direction and do backlash correction
        if not new_direction == 0:
            if new_direction != self.direction:
                self.direction = new_direction
                timing_list = [self.BACKLASH_CORRECTION_DELAY] * self.BACKLASH_CORRECTION + timing_list

            else:
                self.direction = new_direction
        # move
        for target_delay in timing_list:
            if target_delay < self.MIN_DELAY:
                target_delay = self.MIN_DELAY
            current_time = time.time()
            current_delay = current_time - self.last_time
            if current_delay < target_delay:
                time.sleep(target_delay - current_delay)
            self.last_time = current_time

            self.position += self.direction
            next_phase = (self.phase + self.direction) % self.num_phase
            GPIO.output(self.A1, self.PHASE_SEQ[next_phase][0])
            GPIO.output(self.B2, self.PHASE_SEQ[next_phase][1])
            GPIO.output(self.A2, self.PHASE_SEQ[next_phase][2])
            GPIO.output(self.B1, self.PHASE_SEQ[next_phase][3])
            self.phase = next_phase

        if self.ALWAYS_UNHOLD:
            # set all outputs low after every movement
            self.unhold()

    def unhold(self):
        # set all outputs low
        GPIO.output(self.A1, 0)
        GPIO.output(self.A2, 0)
        GPIO.output(self.B1, 0)
        GPIO.output(self.B2, 0)


class FakeBipolarStepperMotor:
    # can be used for running on non-raspberry computers for testing, ...
    def __init__(self, pins, motor_dir=1, full_step=False, backlash_correction=0, backlash_correction_delay=0,
                 max_speed=0, always_unhold=False):
        # initiate a Bipolar_Stepper_Motor object
        GPIO.setmode(GPIO.BCM)

        self.a1, self.a2, self.b1, self.b2 = pins

        self.phase = 0

        self.direction = 0
        self.position = 0

        self.t_last = 0

        self.MOTOR_DIR = motor_dir

    def move(self, steps, delay=0.01):
        self.direction = sign(steps) * self.MOTOR_DIR
        steps = abs(steps)

        for _ in range(steps):
            self.position += self.direction

            # time.sleep(delay)  # commented as fake class is only used for testing purpose and I don't want to wait

    def move_timed(self, steps, timing_list):
        self.direction = sign(steps) * self.MOTOR_DIR

        for target_delay in timing_list:

            delay = target_delay - (time.time() - self.t_last)
            if delay > 0:
                # time.sleep(delay)  # well... see above; but obviously you can turn it on, if you want
                pass

            self.t_last = time.time()

            self.position += self.direction

    def unhold(self):
        pass
