#!/usr/bin/python
# motor_control.py
# Control car with PWM
#
# Author : DWARD CHEN
# Date   : 2017/05/08

import RPi.GPIO as GPIO
import time


class motor_control(object):
    def __init__(self, r1_pin=16, r2_pin=18, l1_pin=11, l2_pin=13, delay=0.1, dc_level=80, balance=1.0):
        self.r1_pin = r1_pin
        self.r2_pin = r2_pin
        self.l1_pin = l1_pin
        self.l2_pin = l2_pin
        self.delay = delay
        self.balance = balance
        self.l_level = dc_level*2/(balance + 1)
        self.r_level = self.l_level*balance

        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(r1_pin, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(r2_pin, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(l1_pin, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(l2_pin, GPIO.OUT, initial=GPIO.LOW)

        self.pwm_r1 = GPIO.PWM(r1_pin, 500)
        self.pwm_r2 = GPIO.PWM(r2_pin, 500)
        self.pwm_l1 = GPIO.PWM(l1_pin, 500)
        self.pwm_l2 = GPIO.PWM(l2_pin, 500)
        self.pwm_r1.start(0)
        self.pwm_r2.start(0)
        self.pwm_l1.start(0)
        self.pwm_l1.start(0)

    def stop(self):
        self.pwm_r1.ChangeDutyCycle(0)
        self.pwm_r2.ChangeDutyCycle(0)
        self.pwm_l1.ChangeDutyCycle(0)
        self.pwm_l2.ChangeDutyCycle(0)

    def forward(self):
        self.pwm_r1.ChangeDutyCycle(self.dc_level)
        self.pwm_r2.ChangeDutyCycle(0)
        self.pwm_l1.ChangeDutyCycle(self.dc_level)
        self.pwm_l2.ChangeDutyCycle(0)
        time.sleep(self.delay)
        self.stop()

    def backward():
        pwm_r1.ChangeDutyCycle(0)
        pwm_r2.ChangeDutyCycle(dc)
        pwm_l1.ChangeDutyCycle(0)
        pwm_l2.ChangeDutyCycle(dc)
        time.sleep(t)
        stop()

    def turnLeft():
        pwm_r1.ChangeDutyCycle(dc)
        pwm_r2.ChangeDutyCycle(0)
        pwm_l1.ChangeDutyCycle(0)
        pwm_l2.ChangeDutyCycle(0)
        time.sleep(t)
        stop()

    def turnRight():
        pwm_r1.ChangeDutyCycle(0)
        pwm_r2.ChangeDutyCycle(0)
        pwm_l1.ChangeDutyCycle(dc)
        pwm_l2.ChangeDutyCycle(0)
        time.sleep(t)
        stop()


    def cleanup():
        stop()
        pwm_r1.stop()
        pwm_r2.stop()
        pwm_l1.stop()
        pwm_l2.stop()
        GPIO.cleanup()

