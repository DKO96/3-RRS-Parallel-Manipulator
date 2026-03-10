import time
import serial

def main():
    ser = serial.Serial(port='/dev/ttyACM0', baudrate=115200, timeout=1)
    ser.reset_input_buffer()
    time.sleep(3)

    while True:
        msg = b'\xDE\xAD\xBE\xEF'
        ser.write(msg)
        ser.flush()
        time.sleep(0.1)

        response = ser.read(len(msg))
        print("Sent:    ", msg.hex())
        print("Received:", response.hex())
        print()

        time.sleep(1)


if __name__ == "__main__":
    main()