#!/usr/bin/python
# hc_sr04_driver.py
# Ultrasonic range finder
#
# Author : DWARD CHEN
# Date   : 2017/05/10

import RPi.GPIO as GPIO
import time
import numpy as np


class HC_SR04(object):
    def __init__(self, trig_pin, echo_pin, scale=1.0, bias=0.0):
        self.trig_pin = trig_pin
        self.echo_pin = echo_pin
        self.scale = scale
        self.bias = bias

        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(trig_pin, GPIO.OUT)
        GPIO.setup(echo_pin, GPIO.IN)

    def measure(self, delay=0.2, n_median=1, quick_return=False):
        distances = np.zeros([n_median]) + np.nan
        t1 = time.time()
        time_out = 1.0 + 0.2*n_median
        while n_median > 0:
            GPIO.output(self.trig_pin, GPIO.LOW)
            time.sleep(delay)
            GPIO.output(self.trig_pin, GPIO.HIGH)
            time.sleep(0.00001)
            GPIO.output(self.trig_pin, GPIO.LOW)

            while GPIO.input(self.echo_pin) == 0:
                if (time.time() - t1) > time_out:
                    return float('inf'), float('inf')
            pulse_start = time.time()

            while GPIO.input(self.echo_pin) == 1:
                if (time.time() - t1) > time_out:
                    return float('inf'), float('inf')
                pass
            pulse_end = time.time()

            n_median -= 1

            distances[n_median] = (pulse_end - pulse_start) * 17150 * self.scale + self.bias

            if quick_return and 0.0 < np.nanstd(distances) < 2.0:
                return np.nanmean(distances), np.nanstd(distances)

        distance = np.nanmedian(distances)

        if distance < 2:
            return 2, np.nanstd(distances)
        elif distance < 400:
            return distance, np.nanstd(distances)
        else:
            return 400, np.nanstd(distances)



