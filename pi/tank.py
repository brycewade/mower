#!/usr/bin/env python3
import sys
import select
import re
from datetime import datetime
import ps3
import serial

JOYSTICK_NAME = "Sony Computer Entertainment Wireless Controller"
LEFT_JOYSTICK = 1
RIGHT_JOYSTICK = 4
L1_BUTTON = 4
R1_BUTTON = 5
X_BUTTON = 0
CIRCLE_BUTTON = 1
TRIANGLE_BUTTON = 2
SQUARE_BUTTON = 3
PS_BUTTON = 10
ARDUINO = "/dev/serial/by-id/usb-Arduino__www.arduino.cc__Arduino_Mega_2560_55838323735351102081-if00"


def start_serial():
    arduino = serial.Serial(ARDUINO, 115200, timeout=0)
    return arduino


def read_serial(arduino):
    return_message = arduino.read(32767)
    while return_message:
        try:
            print(return_message.decode(), end ="")
            return_message = arduino.read(32767)
        except UnicodeDecodeError:
            print("...undecoadable characters...")
            return_message = arduino.read(32767)


def read_from_arduino(arduino, start):
    message = ""
    search = '(?s){}(.*?)OK'.format(start)
    result = None
    while result is None:
        # print("Looking for {} in {}".format(search, message))
        new = arduino.read(32767)
        if new:
            print(new.decode(), end ="")
            message += new.decode()
            result = re.search(search, message)
    return result.group(1)


def wait_for_ok(arduino):
    message = ""
    search = 'OK'
    result = None
    while result is None:
        new = arduino.read(32767)
        if new:
            print(new.decode(), end ="")
            message += new.decode()
            result = re.search(search, message)


def send_to_arduino(arduino, message):
    message_bytes=message.encode() + b'\r'
    arduino.write(message_bytes)
    read_serial(arduino)


def record_position(arduino, output):
    send_to_arduino(arduino, "M114")
    location = read_from_arduino(arduino, "M114")
    lines = location.splitlines()
    output.write(lines[1])
    output.write("\n")


def hold_heading(arduino):
    send_to_arduino(arduino, "G1003")


def calibrate_compass(arduino):
    send_to_arduino(arduino, "G132")


def send_updated_directions(arduino, left, right):
    g_code = 'G1000 L{} R{}'.format(int(left/-128), int(right/-128))
    send_to_arduino(arduino, g_code)


def send_blade_change(arduino, string):
    g_code = 'G1001 {}'.format(string)
    send_to_arduino(arduino, g_code)


def check_keyboard(arduino):
    input = select.select([sys.stdin], [], [], 1)[0]
    if input:
        value = sys.stdin.readline().rstrip()
        send_to_arduino(arduino,value)


def play_file(arduino, file):
    input = open(file, 'r')
    for line in input.readlines():
        send_to_arduino(arduino, line)
        wait_for_ok(arduino)


def main():
    outfile = "/home/pi/maps/{}.txt".format(datetime.now().strftime("%Y%m%d%H%M%S"))
    output = open(outfile, "w")
    done = False
    previous_left = 0
    previous_right = 0
    previous_l1 = 0
    previous_r1 = 0
    left_blade_active= False
    right_blade_active = False
    joystick_index = -1
    blade_change = []
    
    arduino = start_serial()

    # Initialize the joysticks.
    joystick = ps3.Joystick()

    while not done:
        joystick.process_events()
        left = joystick.get_axis(LEFT_JOYSTICK)
        right = joystick.get_axis(RIGHT_JOYSTICK)
        x_button = joystick.get_button(X_BUTTON)
        tri_button = joystick.get_button(TRIANGLE_BUTTON)
        square_button = joystick.get_button(SQUARE_BUTTON)
        circle_button = joystick.get_button(CIRCLE_BUTTON)
        x_button = joystick.get_button(X_BUTTON)
        l1_button = joystick.get_button(L1_BUTTON)
        r1_button = joystick.get_button(R1_BUTTON)
        # check_keyboard(arduino)

        if x_button and not previous_x:
            print("Recording position")
            record_position(arduino, output)
        previous_x = x_button
        
        if tri_button and not previous_tri:
            print("Holding current heading")
            hold_heading(arduino)
        previous_tri = tri_button
        
        if square_button and not previous_square:
            print("Calibrating compass")
            calibrate_compass(arduino)
        previous_square = square_button
        
        if circle_button and not previous_circle:
            print("Playing file")
            play_file(arduino, "input.txt")
        previous_circle = circle_button


        if l1_button and not previous_l1:
            left_blade_active = not left_blade_active
            print("Toggling left blade: {}".format(left_blade_active))
            blade_change.append("L{}".format(left_blade_active * 255))
        previous_l1 = l1_button

        if r1_button and not previous_r1:
            right_blade_active = not right_blade_active
            print("Toggling right blade: {}".format(right_blade_active))
            blade_change.append("R{}".format(right_blade_active * 255))
        previous_r1 = r1_button
        
        if blade_change:
            send_blade_change(arduino, " ".join(blade_change))
            blade_change.clear()

        if left != previous_left or right != previous_right:
            previous_left = left
            previous_right = right
            # print("Updating directions {}, {}".format(left, right))
            send_updated_directions(arduino, left, right)
        
        if joystick.get_button(PS_BUTTON) == 1:
            print("Exiting.")
            done = True
        read_serial(arduino)
    output.close()

if __name__ == "__main__":
    # execute only if run as a script
    main()

