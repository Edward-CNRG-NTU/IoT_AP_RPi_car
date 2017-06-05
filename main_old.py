import cv2
import time
import RPi.GPIO as GPIO
from hc_sr04_driver import HC_SR04
from motor_control import MotorControl
import numpy as np
import threading

HSV_YELLOW = {'lower': (22, 90, 80), 'upper': (33, 255, 255)}
HSV_RED    = {'lower': (0, 100, 80), 'upper': (10, 255, 255)}

Frame_Width  = 320
Frame_Height = 240
View_Center = Frame_Width / 2

camera = cv2.VideoCapture(0)
camera.set(cv2.CAP_PROP_FRAME_WIDTH,  Frame_Width)
camera.set(cv2.CAP_PROP_FRAME_HEIGHT, Frame_Height)

motor = MotorControl(dc_level=80, t=0.3)

hc_sr04_1 = HC_SR04(22, 32)
hc_sr04_2 = HC_SR04(29, 31)
hc_sr04_3 = HC_SR04(33, 37)

GPIO.setmode(GPIO.BOARD)
GPIO.setup(12, GPIO.OUT, initial=GPIO.LOW)
servo_pwm = GPIO.PWM(12, 50)
servo_pwm.start(7.5)


def launch_camera_routine():
    stopper = threading.Event()

    def camera_routine():
        while not stopper.is_set():
            global g_frame
            (_, g_frame) = camera.read()
            cv2.imshow("raw", g_frame)
            time.sleep(0.05)

        print('camera_routine stopped!')

    t = threading.Thread(target=camera_routine)
    t.setDaemon(True)
    t.start()

    return stopper

camera_routine_stopper = launch_camera_routine()


def launch_waitkey_routine():
    stopper = threading.Event()

    def waitkey_routine():
        while not stopper.wait(timeout=0.1):

            key = cv2.waitKey(1) & 0xFF

            if key is 0xFF:
                pass
            else:
                if key == ord('q'):
                    global g_quit
                    g_quit = True
                elif key == ord('a'):
                    motor.stop()
                    motor.turn_left()
                elif key == ord('d'):
                    motor.stop()
                    motor.turn_right()
                elif key == ord('w'):
                    motor.stop()
                    motor.forward()
                elif key == ord('s'):
                    motor.stop()
                    motor.backward()
                elif ord('0') <= key < ord('9'):
                    global g_state
                    g_state = key - ord('0')
                else:
                    pass

                while key is not 0xFF:
                    key = cv2.waitKey(1) & 0xFF

        print('waitkey_routine stopped!')

    t = threading.Thread(target=waitkey_routine)
    t.setDaemon(True)
    t.start()

    return stopper

launch_waitkey_routine_stopper = launch_waitkey_routine()


def launch_center_proximity_sensor_routine(span=[-70, 70], steps=5):
    stopper = threading.Event()

    def center_proximity_sensor_routine():
        while not stopper.is_set():
            global g_center_proximity
            g_center_proximity = np.zeros([steps, 2]) + np.nan
            step_angle = (span[1] - span[0])/(steps-1)
            pwm_values = (np.array(list(range(span[0], span[1]+step_angle, step_angle))) + 90) / 18 + 2.5
            for i in range(steps):
                servo_pwm.ChangeDutyCycle(pwm_values[i])
                time.sleep(1)
                (g_center_proximity[i, 0], g_center_proximity[i, 1]) = hc_sr04_2.measure(delay=0.001, n_median=5,
                                                                                         quick_return=True)
            servo_pwm.ChangeDutyCycle(7.5)
            time.sleep(0.5)
            print(g_center_proximity)
            # print(g_proximity, 'std:', std)

        print('center_proximity_sensor_routine stopped!')

    t = threading.Thread(target=center_proximity_sensor_routine)
    t.setDaemon(True)
    t.start()

    return stopper

# g_center_proximity = None
# center_proximity_sensor_routine_stopper = launch_center_proximity_sensor_routine()

g_proximity = [0, 0, 0]
# proximity_sensor_routine_stopper = launch_proximity_sensor_routine()

g_frame = None
g_quit = False
g_state = 2


try:
    while not g_quit:
        # (_, frame) = camera.read()
        # (_, frame) = camera.read()
        # (_, frame) = camera.read()

        if g_state == 0 and g_frame is not None:
            frame = cv2.GaussianBlur(g_frame, (11, 11), 0)
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, HSV_RED['lower'], HSV_RED['upper'])
            (_, contours, _) = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            center = None
            if len(contours) > 0:
                # Find the max length of contours
                c = max(contours, key=cv2.contourArea)
                # Find the x, y, radius of given contours
                ((x, y), radius) = cv2.minEnclosingCircle(c)
                # Find the moments
                M = cv2.moments(c)

                try:
                    # mass center
                    center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
                    # process every frame
                    cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)
                    cv2.circle(frame, center, 5, (0, 0, 255), -1)

                    # turn right and turn left rule
                    if center[0] > View_Center + 10:
                        dt = center[0] / View_Center * 0.08
                        motor.turn_right(speed=0.8, t=dt)
                        motor.forward(speed=0.8, t=0.1)
                        print('R')
                        # time.sleep(0.4)
                    elif center[0] < View_Center - 10:
                        dt = (Frame_Width - center[0]) / View_Center * 0.08
                        motor.turn_left(speed=0.8, t=dt)
                        motor.forward(speed=0.8, t=0.1)
                        print('L')
                        # time.sleep(0.4)
                    # Forward and backward rule
                    elif radius < 80:
                        motor.forward(speed=1.2, t=0.4)
                        print('F')
                    elif radius > 80:
                        motor.backward()
                        g_state = 1
                        print('B')
                    else:
                        motor.stop()

                # if not find mass center
                except:
                    pass

            cv2.imshow("Frame", frame)

        elif g_state == 1:
            front_approx, _ = hc_sr04_2.measure(delay=0.001, n_median=5, quick_return=True)
            left_approx, _ = hc_sr04_3.measure(delay=0.001, n_median=5, quick_return=True)
            while not g_quit and (left_approx - front_approx) > -3.0:
                dt = 0.1 + (left_approx - front_approx) / 800.0
                motor.turn_right(speed=0.8, t=dt)
                time.sleep(dt)
                left_approx, _ = hc_sr04_3.measure(delay=0.001, n_median=5, quick_return=True)
                if left_approx > 300.0:
                    left_approx = 300.0
                print left_approx, front_approx, dt

            motor.turn_right(speed=0.8, t=0.1)
            g_state = 2
            print 'to state2'

        elif g_state == 2:
            left_approx, _ = hc_sr04_3.measure(delay=0.001, n_median=5, quick_return=True)
            motor.forward(speed=0.8, t=0.5)
            left_approx2, _ = hc_sr04_3.measure(delay=0.001, n_median=5, quick_return=True)
            skew = left_approx2 - left_approx
            dt = 0.1 + np.abs(skew) / 70.0

            print left_approx, left_approx2, skew, dt

            if skew < -0.2:
                motor.turn_right(speed=0.8, t=dt)
                motor.forward(speed=0.8, t=0.1)
                print 'R'
            elif skew > 0.2:
                motor.turn_left(speed=0.8, t=dt)
                motor.forward(speed=0.8, t=0.1)
                print 'L'
            else:
                motor.forward(speed=1.2, t=0.5)
                print 'F'
            # print(g_proximity)

            # def find_angle():
            #     i_min = 0
            #     d_min = g_proximity[2]
            #
            #     for i in range(30):
            #         motor.turn_right(speed=0.6, t=0.2)
            #         time.sleep(0.2)
            #         (d_min, i_min) = (g_proximity[2], i) if g_proximity[2] < d_min else (d_min, i_min)
            #         print g_proximity[2], i, d_min, i_min
            #
            # find_angle()

            time.sleep(1.0)


            # motor.forward(1.2, 0.2)

            # if d3 > 45:
            #     motor.turn_left(speed=0.6, t=0.1)
            #     motor.forward(speed=0.8, t=0.6)
            # elif d3 < 20:
            #     motor.turn_right(speed=0.6, t=0.1)
            #     motor.forward(speed=0.8, t=0.6)
            # else:
            #     motor.forward(speed=0.8, t=0.5)
            # state = 3
        else:
            pass

finally:
    camera_routine_stopper.set()
    # proximity_sensor_routine_stopper.set()
    # center_proximity_sensor_routine_stopper.set()
    motor.cleanup()
    camera.release()
    cv2.destroyAllWindows()
