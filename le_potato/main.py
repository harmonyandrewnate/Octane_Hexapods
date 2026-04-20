from inputs import get_gamepad
from inputs import devices

import serial



def main():
    ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
    
    ser.write(b'\n\n\n\n\n\n')
    
    while True:
        events = get_gamepad()
        for event in events:
            if event.ev_type == 'button':
                if event.state == 0:
                    ser.write(b's\n')
                else:
                    if event.code == 'BTN_WEST':
                        ser.write(b'f\n')
                    elif event.code == 'BTN_SOUTH':
                        ser.write(b'b\n')
                    elif event.code == 'BTN_NORTH':
                        ser.write(b'l\n')
                    elif event.code == 'BTN_EAST':
                        ser.write(b'r\n')



if __name__ == "__main__":
    main()
