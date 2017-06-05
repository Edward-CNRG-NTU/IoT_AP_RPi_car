import cv2
import time
import serial
import threading
import numpy as np
from motor_control import MotorControl

serial_port = serial.Serial('/dev/ttyACM0', 9600)

HSV_YELLOW = {'lower': (22, 90, 80), 'upper': (33, 255, 255)}
HSV_RED    = {'lower': (0, 100, 80), 'upper': (10, 255, 255)}
HSV_RED2   = {'lower': (170, 100, 80), 'upper': (179, 255, 255)}
HSV_GREEN  = {'lower': (40, 120, 66), 'upper': (45, 255, 255)}

Frame_Width  = 320
Frame_Height = 240
View_Center = Frame_Width / 2

camera = cv2.VideoCapture(0)
camera.set(cv2.CAP_PROP_FRAME_WIDTH,  Frame_Width)
camera.set(cv2.CAP_PROP_FRAME_HEIGHT, Frame_Height)

motor = MotorControl(dc_level=50, t=0.3)

# from hc_sr04_driver import HC_SR04
# hc_sr04_1 = HC_SR04(22, 32)
# hc_sr04_2 = HC_SR04(29, 31)
# hc_sr04_3 = HC_SR04(33, 37)

# import RPi.GPIO as GPIO
# GPIO.setmode(GPIO.BOARD)
# GPIO.setup(12, GPIO.OUT, initial=GPIO.LOW)
# servo_pwm = GPIO.PWM(12, 50)
# servo_pwm.start(7.5)


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
                print('key in:', chr(key))
                if key == ord('q'):
                    global g_quit
                    g_quit = True
                elif key == ord('a'):
                    motor.turn_left()
                elif key == ord('d'):
                    motor.turn_right()
                elif key == ord('w'):
                    motor.forward()
                elif key == ord('s'):
                    motor.backward()
                elif ord('0') <= key <= ord('9'):
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


def launch_external_sensor_routine():
    stopper = threading.Event()

    def external_sensor_routine():
        while not stopper.is_set():
            line = serial_port.readline()
            # print 'received:', line
            if line.startswith('#'):
                try:
                    data = np.array(line[1:-1].split(','), dtype='|S4').astype(np.float)
                except ValueError:
                    continue

                global g_proximity
                g_proximity = data[0:3]

                global g_wheel_count
                g_wheel_count = data[3:5]

                global g_external_sensor_condition
                # g_external_sensor_condition.acquire()
                g_external_sensor_condition.notifyAll()
                g_external_sensor_condition.release()

        print('external_sensor_routine stopped!')

    t = threading.Thread(target=external_sensor_routine)
    t.setDaemon(True)
    t.start()

    return stopper

external_sensor_routine_stopper = launch_external_sensor_routine()


def turn_right_controlled(angle):
    wheel_last = g_wheel_count
    while g_wheel_count[0] - wheel_last[0] < angle / 4.45:
        motor.turn_right(speed=1.0, t=0.1)


def turn_left_controlled(angle):
    wheel_last = g_wheel_count
    while g_wheel_count[1] - wheel_last[1] < angle / 4.45:
        motor.turn_left(speed=1.0, t=0.1)


def forward_controlled(distance):
    wheel_last = g_wheel_count
    diff_of_both = g_wheel_count - wheel_last
    while np.sum(diff_of_both) / 2 < distance / 0.0113:
        motor.forward(speed=1.2, t=0.5)
        diff_of_both = g_wheel_count - wheel_last
        diff_between = diff_of_both[0] - diff_of_both[1]
        print np.sum(diff_of_both), diff_between
        if diff_between > 0:
            motor.turn_left(speed=0.7, t=0.1 + np.abs(diff_between) * 0.005)
        elif diff_between < 0:
            motor.turn_right(speed=0.7, t=0.1 + np.abs(diff_between) * 0.005)


def cv2_find_center_and_radius(hsv_color_range, hsv_color_range_2=None):
    center = None
    radius = None
    frame = None

    if g_frame is not None and hsv_color_range is not None:
        frame = cv2.GaussianBlur(g_frame, (11, 11), 0)
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv_frame, hsv_color_range['lower'], hsv_color_range['upper'])
        if hsv_color_range_2 is not None:
            mask += cv2.inRange(hsv_frame, hsv_color_range_2['lower'], hsv_color_range_2['upper'])

        (_, contours, _) = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if len(contours) > 0:
            # Find the max length of contours
            c = max(contours, key=cv2.contourArea)
            # Find the x, y, radius of given contours
            ((x, y), radius) = cv2.minEnclosingCircle(c)

            try:
                # Find the moments
                M = cv2.moments(c)
                # mass center
                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
                # process every frame
                cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)
                cv2.circle(frame, center, 5, (0, 0, 255), -1)
            except KeyError:
                pass

    return center, radius, frame


def proximity_filtered(median=5):
    raw_data = np.zeros([median, g_proximity.shape[0]])
    for i in range(median):
        g_external_sensor_condition.acquire()
        g_external_sensor_condition.wait()
        raw_data[i] = g_proximity
        g_external_sensor_condition.release()

    return np.nanmedian(raw_data, axis=0)


g_frame = None
g_quit = False
g_state = 3
g_wheel_last = 0
g_proximity = np.array([0, 0, 0])
g_wheel_count = np.array([0, 0])
g_external_sensor_condition = threading.Condition()

time.sleep(1)

try:
    while not g_quit:

        if g_state == 0:

            center, radius, frame = cv2_find_center_and_radius(HSV_RED, HSV_RED2)

            cv2.imshow("Frame", frame)

            # turn right and turn left rule
            if center[0] > View_Center + 10:
                dt = center[0] / View_Center * 0.07
                motor.turn_right(speed=0.8, t=dt)
                motor.forward(speed=0.8, t=0.1)
                print('R')
            elif center[0] < View_Center - 10:
                dt = (Frame_Width - center[0]) / View_Center * 0.07
                motor.turn_left(speed=0.8, t=dt)
                motor.forward(speed=0.8, t=0.1)
                print('L')
            elif radius < 80:
                motor.forward(speed=1.2, t=0.35)
                print('F')
            elif radius > 80:
                motor.backward()
                g_state = 1
                print('to state 1, radius=', radius)
            else:
                motor.stop()

            if g_proximity[1] < 25:
                motor.backward()
                g_state = 1
                print('to state 1, front_proxi=', g_proximity[1])

        elif g_state == 1:

            time.sleep(0.2)
            front_approx = g_proximity[1]
            left_approx = g_proximity[0]
            while not g_quit and (left_approx - front_approx) > -3.0:
                dt = 0.1 + (left_approx - front_approx) / 800.0
                motor.turn_right(speed=0.8, t=dt)
                time.sleep(dt)
                left_approx = g_proximity[0]
                if left_approx > 300.0:
                    left_approx = 300.0
                print left_approx, front_approx, dt

            motor.turn_right(speed=0.8, t=0.1)
            g_state = 2
            g_wheel_last = g_wheel_count
            print 'to state2'

        elif g_state == 2:

            time.sleep(0.2)
            left_approx = g_proximity[0]
            motor.forward(speed=1.2, t=0.8)
            time.sleep(0.2)
            left_approx2 = g_proximity[0]
            skew = left_approx2 - left_approx
            skew = skew if -50 < skew < 50 else 50
            dt = 0.1 + np.abs(skew) / 70

            print left_approx, left_approx2, skew, dt

            if skew < -1:
                motor.turn_right(speed=0.8, t=dt)
                motor.forward(speed=0.8, t=0.1)
                print 'R'
            elif skew > 1:
                motor.turn_left(speed=0.8, t=dt)
                motor.forward(speed=0.8, t=0.1)
                print 'L'
            else:
                # motor.forward(speed=1.2, t=0.8)
                print 'F'

            center, radius, frame = cv2_find_center_and_radius(HSV_GREEN)

            if radius > 60:
                motor.forward(speed=0.8, t=1.5)
                g_state = 3
                print('to state 3, radius=', radius)

            if np.sum(g_wheel_count - g_wheel_last)/2 > 100:
                g_state = 3
                print('to state 3, travelled:', np.sum(g_wheel_count - g_wheel_last)/2)

            cv2.imshow("Frame", frame)

        elif g_state == 3:
            t1 = time.time()
            print proximity_filtered(3)
            print time.time()-t1

            turn_right_controlled(100)
            forward_controlled(2)

            g_state = 4
            print('to state 4')
        else:
            pass

finally:
    camera_routine_stopper.set()
    external_sensor_routine_stopper.set()
    motor.cleanup()
    camera.release()
    cv2.destroyAllWindows()
