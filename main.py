import time
import serial
from planning import motion_planning

def main():
    ser = serial.Serial(
        port='/dev/ttyACM0', 
        baudrate=115200, 
        bytesize=serial.EIGHTBITS,
        stopbits=serial.STOPBITS_ONE,
        parity=serial.PARITY_NONE,
        timeout=1
    )

    ser.reset_input_buffer()
    time.sleep(2)

    commands = motion_planning()

    try:
        for cmd in commands:
            msg = bytes(cmd)
            ser.write(b'\xAA' + msg + b'\x55')
            ser.flush()
            time.sleep(0.01)

    except KeyboardInterrupt:
        print("\nStopped.")

    finally:
        ser.close()


if __name__ == "__main__":
    main()