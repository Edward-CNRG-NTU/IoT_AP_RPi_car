#!/usr/bin/python
# hc_sr04_driver.py
# Ultrasonic range finder
#
# Author : DWARD CHEN
# Date   : 2017/05/10

import RPi.GPIO as GPIO
import time
import numpy as np


class HC_SR04Driver(object):
    def __init__(self, trig_echo_pin, scale=1.0, bias=0.0):
        self.trig_echo_pin = trig_echo_pin
        self.n_modules = len(trig_echo_pin)
        self.scale = scale
        self.bias = bias

        GPIO.setmode(GPIO.BOARD)
        for i in range(self.n_modules):
            GPIO.setup(trig_echo_pin[i][0], GPIO.OUT)
            GPIO.setup(trig_echo_pin[i][1], GPIO.IN)

    def measure(self, delay=0.2, n_median=1):
        range_n = range(self.n_modules)
        distance = np.zeros([n_median, self.n_modules])
        t1 = np.zeros([self.n_modules])
        t2 = np.zeros([self.n_modules])
        while n_median > 0:
            [GPIO.output(self.trig_echo_pin[i][0], GPIO.LOW) for i in range_n]
            time.sleep(delay)
            [GPIO.output(self.trig_echo_pin[i][0], GPIO.HIGH) for i in range_n]
            time.sleep(0.00001)
            [GPIO.output(self.trig_echo_pin[i][0], GPIO.LOW) for i in range_n]

            while 0 in t2:
                t = time.time()
                for i in range_n:
                    if GPIO.input(self.trig_echo_pin[i][1]) == 1:
                        if t1[i] == 0:
                            t1[i] = t
                    elif t1[i] > 0:
                        t2[i] = t

            n_median -= 1

            distance[n_median, :] = (t2 - t1) * 17150 * self.scale + self.bias

            time.sleep(1)

        return np.median(distance, axis=0)


