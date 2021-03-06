import cv2
import time
import serial
import threading
import numpy as np
import motor_control


HSV_YELLOW = {'lower': (27, 110, 90), 'upper': (31, 255, 255)}
HSV_RED    = {'lower': (0, 100, 80), 'upper': (18, 255, 255)}
HSV_RED2   = {'lower': (170, 100, 80), 'upper': (179, 255, 255)}
HSV_GREEN  = {'lower': (40, 120, 66), 'upper': (45, 255, 255)}

Frame_Width  = 320
Frame_Height = 240
View_Center = Frame_Width / 2

camera = cv2.VideoCapture(0)
camera.set(cv2.CAP_PROP_FRAME_WIDTH,  Frame_Width)
camera.set(cv2.CAP_PROP_FRAME_HEIGHT, Frame_Height)

motor = motor_control.MotorControl(dc_level=70, t=0.3)

g_frame = None
g_quit = False
g_state = 5
g_wheel_last = 0
g_wheel_count = np.array([0, 0])
g_proximity = np.array([0, 0, 0])
g_external_sensor_condition = threading.Condition()
g_proximity_control = 15
g_obstacle_detected = False


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
        while not stopper.wait(timeout=0.05):
            global g_frame
            (_, g_frame) = camera.read()
            cv2.imshow("raw", g_frame)

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
        serial_port = serial.Serial('/dev/ttyACM0', 115200)
        serial_port.write('R')
        proximity_control = 0

        while not stopper.is_set():
            line = serial_port.readline()
            # print 'received:', line
            if line.startswith('#'):
                try:
                    data = np.array(line[1:-1].split(','), dtype='|S4').astype(np.float)
                except ValueError:
                    continue

                global g_external_sensor_condition
                g_external_sensor_condition.acquire()

                global g_proximity
                g_proximity = data[0:3]

                global g_wheel_count
                g_wheel_count = data[3:5]

                g_external_sensor_condition.notifyAll()
                g_external_sensor_condition.release()

            if g_proximity_control != proximity_control:
                serial_port.write(chr(g_proximity_control))
                proximity_control = g_proximity_control

        print('external_sensor_routine stopped!')

    t = threading.Thread(target=external_sensor_routine)
    t.setDaemon(True)
    t.start()

    return stopper

external_sensor_routine_stopper = launch_external_sensor_routine()


def launch_obstacle_detection_routine():
    stopper = threading.Event()

    def obstacle_detection_routine():
        global g_proximity_control
        while not stopper.wait(timeout=0.1):
            global g_obstacle_detected
            if np.any(filtered_proximity(3) < [5, 25, 5]):
                g_obstacle_detected = True
                print('obstacle detected:', g_proximity)
            else:
                g_obstacle_detected = False

                if g_proximity_control == 10:
                    g_proximity_control = 15
                elif g_proximity_control == 15:
                    g_proximity_control = 20
                elif g_proximity_control == 20:
                    g_proximity_control = 10
                else:
                    g_proximity_control = 15

        g_proximity_control = 15
        print('obstacle_detection_routine stopped!')

    t = threading.Thread(target=obstacle_detection_routine)
    t.setDaemon(True)
    t.start()

    return stopper

obstacle_detection_routine_stopper = None


def turn_right_controlled(angle):
    wheel_last = g_wheel_count
    count = angle / 4.45
    while not g_quit:
        if not g_obstacle_detected:
            time.sleep(0.05)
            if g_wheel_count[0] - wheel_last[0] < count:
                motor.turn_right(speed=0.8, t=0.2)
            elif g_wheel_count[0] - wheel_last[0] > count:
                motor.turn_left(speed=0.8, t=0.15)
                break
            else:
                break
        else:
            time.sleep(0.1)


def turn_left_controlled(angle):
    wheel_last = g_wheel_count
    count = angle / 4.45
    while not g_quit:
        if not g_obstacle_detected:
            time.sleep(0.05)
            if g_wheel_count[1] - wheel_last[1] < count:
                motor.turn_left(speed=0.8, t=0.2)
            elif g_wheel_count[1] - wheel_last[1] > count:
                motor.turn_right(speed=0.8, t=0.15)
                break
            else:
                break
        else:
            time.sleep(0.1)


def forward_controlled(distance):
    wheel_last = g_wheel_count
    diff_of_both = g_wheel_count - wheel_last
    while(not g_quit) and np.sum(diff_of_both) / 2 < distance / 0.0113:
        if not g_obstacle_detected:
            motor.forward(speed=1.0, t=0.3)
            # time.sleep(0.1)
            diff_of_both = g_wheel_count - wheel_last
            diff_between = diff_of_both[0] - diff_of_both[1]
            print np.sum(diff_of_both), diff_between
            if diff_between > 0:
                motor.turn_left(speed=0.7, t=0.1 + np.abs(diff_between) * 0.005)
            elif diff_between < 0:
                motor.turn_right(speed=0.7, t=0.1 + np.abs(diff_between) * 0.005)
        else:
            time.sleep(0.1)


def cv2_find_enclosing_circle(hsv_color_range, hsv_color_range_2=None):
    center = None
    radius = None
    frame = None

    if g_frame is not None and hsv_color_range is not None:
        frame = cv2.GaussianBlur(g_frame, (11, 11), 0)
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv_frame, hsv_color_range['lower'], hsv_color_range['upper'])
        frame = cv2.bitwise_and(frame, frame, mask=mask)
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
            except ZeroDivisionError:
                pass

    return center, radius, frame


def cv2_find_enclosing_rect(hsv_color_range, hsv_color_range_2=None):
    x = y = w = h = frame = None

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
            x, y, w, h = cv2.boundingRect(c)
            cv2.rectangle(frame, (int(x), int(y)), (int(x+w), int(y+h)), (0, 255, 255), 2)

    return x, y, w, h, frame


def filtered_proximity(median=5):
    raw_data = np.zeros([median, g_proximity.shape[0]])
    for i in range(median):
        g_external_sensor_condition.acquire()
        g_external_sensor_condition.wait()
        raw_data[i] = g_proximity
        g_external_sensor_condition.release()

    print raw_data
    return np.nanmedian(raw_data, axis=0)


time.sleep(1)

try:
    while not g_quit:

        if g_state == 0:

            center, radius, frame = cv2_find_enclosing_circle(HSV_RED, HSV_RED2)

            if center is not None:
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
                    motor.forward(speed=1.0, t=0.3)
                    print('F')
                elif radius > 80:
                    # motor.backward()
                    g_state = 1
                    print('radius=', radius)
                else:
                    motor.stop()

            proximity = filtered_proximity(3)
            if proximity[1] < 25:
                motor.backward()
                g_state = 1
                print('to state 1, front_proxi=', proximity[1])

        elif g_state == 1:

            proximity = filtered_proximity(5)
            front_proximity = proximity[1]
            while not g_quit and (proximity[0] - front_proximity) > -4.0:
                dt = 0.1 + np.abs(proximity[0] - front_proximity) / 750.0
                motor.turn_right(speed=0.8, t=dt)
                print proximity, dt
                proximity = filtered_proximity(5)

            motor.turn_right(speed=0.8, t=0.1)
            g_state = 2
            g_wheel_last = g_wheel_count
            print 'to state2'

        elif g_state == 2:

            proximity = filtered_proximity(3)
            motor.forward(speed=1.0, t=0.5)
            time.sleep(0.1)
            proximity2 = filtered_proximity(3)
            skew = proximity2[0] - proximity[0]
            if skew > 40:
                skew = 40
            elif skew < -40:
                skew = -40
            dt = 0.1 + np.abs(skew) / 120

            print proximity[0], proximity2[0], skew, dt

            if skew < 0:
                motor.turn_right(speed=0.7, t=dt)
                motor.forward(speed=0.7, t=0.1)
                print 'R'
            elif skew > 1:
                motor.turn_left(speed=0.7, t=dt)
                motor.forward(speed=0.7, t=0.1)
                print 'L'
            else:
                # motor.forward(speed=1.2, t=0.8)
                print 'F'

            if proximity2[0] < 15:
                motor.turn_right(speed=0.7, t=0.2)
                motor.forward(speed=0.7, t=0.4)
                motor.turn_left(speed=0.7, t=0.15)
                time.sleep(0.3)

            center, radius, frame = cv2_find_enclosing_circle(HSV_GREEN)

            if center is not None:
                if radius > 80:
                    motor.forward(speed=0.8, t=1.2)
                    g_state = 3
                    print('to state 3, radius=', radius)

            if np.sum(g_wheel_count - g_wheel_last)/2 > 260:
                g_state = 3
                print('to state 3, travelled:', np.sum(g_wheel_count - g_wheel_last)/2)

            cv2.imshow("Frame", frame)

        elif g_state == 3:

            turn_right_controlled(95)
            time.sleep(0.5)

            obstacle_detection_routine_stopper = launch_obstacle_detection_routine()

            forward_controlled(2.0)
            time.sleep(0.5)
            turn_right_controlled(85)
            time.sleep(0.5)
            forward_controlled(2.2)
            time.sleep(0.5)
            turn_right_controlled(90)
            time.sleep(0.5)
            forward_controlled(0.9)

            if obstacle_detection_routine_stopper is not None:
                obstacle_detection_routine_stopper.set()

            g_state = 0
            print('to state 0')

        elif g_state == 5:
            g_wheel_last = g_wheel_count
            g_state = 6

        elif g_state == 6:
            x, y, w, h, frame = cv2_find_enclosing_rect(HSV_RED, HSV_RED2)

            if frame is not None and x is not None and w is not None:
                cv2.imshow("Frame", frame)
                center = x + w / 2

                if center > View_Center + 10:
                    dt = center / View_Center * 0.07
                    motor.turn_right(speed=0.7, t=dt)
                    motor.forward(speed=0.8, t=0.1)
                    print('R')
                elif center < View_Center - 10:
                    dt = (Frame_Width - center) / View_Center * 0.07
                    motor.turn_left(speed=0.7, t=dt)
                    motor.forward(speed=0.8, t=0.1)
                    print('L')
                elif w > 30:
                    motor.forward(speed=1.0, t=0.5)
                    print('F')
                elif h < 90 and y < 100:
                    motor.forward(speed=1.0, t=0.5)
                    print('F')
                    g_state = 7
                else:
                    motor.stop()

            if np.sum(g_wheel_count - g_wheel_last)/2 > 180:
                g_state = 7
                print('to state 7, travelled:', np.sum(g_wheel_count - g_wheel_last) / 2)

            if filtered_proximity(5)[1] < 80:
                g_state = 7
                print('to state 7, travelled:', np.sum(g_wheel_count - g_wheel_last) / 2)
                forward_controlled(0.2)

            print g_proximity

        elif g_state == 7:
            turn_left_controlled(85)
            time.sleep(0.5)
            forward_controlled(3.2)
            time.sleep(0.5)
            turn_left_controlled(90)
            time.sleep(0.5)
            forward_controlled(2.1)
            time.sleep(0.5)
            turn_left_controlled(90)
            time.sleep(0.5)
            forward_controlled(3.0)
            time.sleep(0.5)
            turn_left_controlled(90)

        else:
            pass

finally:
    if obstacle_detection_routine_stopper is not None:
        obstacle_detection_routine_stopper.set()
    camera_routine_stopper.set()
    external_sensor_routine_stopper.set()
    motor.cleanup()
    camera.release()
    cv2.destroyAllWindows()
