#!/usr/bin/python
# motor_control.py
# Control car with PWM
#
# Author : DWARD CHEN
# Date   : 2017/05/08

import RPi.GPIO as GPIO
import time


class HC_SR04Driver(object):
    def __init__(self, trig_pin, echo_pin, scale=1.0, bias=0.0):
        self.trig_pin = trig_pin
        self.echo_pin = echo_pin
        self.scale = scale
        self.bias = bias

        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(trig_pin, GPIO.OUT)
        GPIO.setup(echo_pin, GPIO.IN)

    def measure(self, delay=0.5, n_shots=1):
        GPIO.output(self.trig_pin, GPIO.LOW)
        time.sleep(delay)
        GPIO.output(self.trig_pin, GPIO.HIGH)
        time.sleep(0.00001)
        GPIO.output(self.trig_pin, GPIO.LOW)

        while GPIO.input(self.echo_pin) == 0:
            pass
        pulse_start = time.time()

        while GPIO.input(self.echo_pin) == 1:
            pass
        pulse_end = time.time()

        distance = (pulse_end - pulse_start) * 17150 * self.scale + self.bias

        if 2 < distance < 400:
            return distance
        else:
            return float('nan')



