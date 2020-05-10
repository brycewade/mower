#!/usr/bin/env python3
import ps3
import serial

JOYSTICK_NAME = "Sony Computer Entertainment Wireless Controller"
LEFT_JOYSTICK = 1
RIGHT_JOYSTICK = 4
L1_BUTTON = 4
R1_BUTTON = 5
X_BUTTON = 0
TRIANGLE_BUTTON = 2
PS_BUTTON = 10
ARDUINO = "/dev/serial/by-id/usb-Arduino__www.arduino.cc__Arduino_Mega_2560_55838323735351102081-if00"


def start_serial():
    arduino = serial.Serial(ARDUINO, 115200, timeout=0)
    return arduino


def read_serial(arduino):
    return_message = arduino.read(32767)
    while return_message:
        print(return_message.decode(), end ="")
        return_message = arduino.read(32767)


def send_to_arduino(arduino, message):
    # print(" " + message)
    message_bytes=message.encode() + b'\r'
    # for i in message_bytes:
    #     print(i)
    arduino.write(message_bytes)
    read_serial(arduino)


def record_position(arduino):
    send_to_arduino(arduino, "M114")
    # location=read_from_arduino()


def hold_heading(arduino):
    send_to_arduino(arduino, "G1002")


def send_updated_directions(arduino, left, right):
    g_code = 'G1000 L{} R{}'.format(int(left/-128), int(right/-128))
    send_to_arduino(arduino, g_code)


def send_blade_change(arduino, string):
    g_code = 'G1001 {}'.format(string)
    send_to_arduino(arduino, g_code)


def main():
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
        x_button = joystick.get_button(X_BUTTON)
        l1_button = joystick.get_button(L1_BUTTON)
        r1_button = joystick.get_button(R1_BUTTON)
        if x_button and not previous_x:
            print("Recording position")
            record_position(arduino)
        previous_x = x_button
        
        if tri_button and not previous_tri:
            print("Holding current heading")
            hold_heading(arduino)
        previous_tri = tri_button


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

if __name__ == "__main__":
    # execute only if run as a script
    main()

