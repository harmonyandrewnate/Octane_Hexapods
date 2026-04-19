"""Simple example showing how to get gamepad events."""

from inputs import get_gamepad
from inputs import devices


def main():
    """When run, just print out the device names."""

    print("We have detected the following devices:\n")

    for device in devices:
        print(device)
    
    """Just print out some event infomation when the gamepad is used."""
    while 1:
        events = get_gamepad()
        for event in events:
            print(event.ev_type, event.code, event.state)


if __name__ == "__main__":
    main()
