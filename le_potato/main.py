from inputs import get_gamepad
from inputs import devices

import serial



def main():
    ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)

    while True:
        events = get_gamepad()
        for event in events:
            if event.ev_type == 'button':
                if event.state == 0:
                    ser.write(b's')
                else:
                    if event.code == 'BTN_WEST':
                        ser.write(b'f')
                    elif event.code == 'BTN_SOUTH':
                        ser.write(b'b')
                    elif event.code == 'BTN_NORTH':
                        ser.write(b'l')
                    elif event.code == 'BTN_EAST':
                        ser.write(b'r')



if __name__ == "__main__":
    main()
