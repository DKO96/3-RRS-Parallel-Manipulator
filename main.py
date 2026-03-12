import time
import serial
import numpy as np
from planning import motion_planning

def normalize_vector(v):
    norm = np.linalg.norm(v[:3])
    if norm == 0:
        return v[:3]
    return v[:3] / norm

def generate_demo():
    motor_commands = []

    poses = np.array([
        [0.0, 0.0, 1.0, 50.0],
        [0.0, 0.0, 1.0, 120.0],
        [0.4, 0.0, 0.9, 120.0],
        [0.0, 0.4, 0.9, 120.0],
        [-0.4, 0.0, 0.9, 120.0],
        [0.0, -0.4, 0.9, 120.0],
        [0.4, 0.0, 0.9, 120.0],
        [0.0, 0.4, 0.9, 120.0],
        [-0.4, 0.0, 0.9, 120.0],
        [0.0, -0.4, 0.9, 120.0],
        [0.4, 0.0, 0.9, 120.0],
        [0.0, 0.4, 0.9, 120.0],
        [-0.4, 0.0, 0.9, 120.0],
        [0.0, -0.4, 0.9, 120.0],
        [0.4, 0.0, 0.9, 120.0],
        [0.0, 0.4, 0.9, 120.0],
        [-0.4, 0.0, 0.9, 120.0],
        [0.0, -0.4, 0.9, 120.0],
        [0.4, 0.0, 0.9, 120.0],
        [0.0, 0.4, 0.9, 120.0],
        [-0.4, 0.0, 0.9, 120.0],
        [0.0, -0.4, 0.9, 120.0],
        [0.0, 0.0, 1.0, 120.0],
        [0.0, 0.0, 1.0, 50.0],
    ])
    
    for i in range(len(poses) - 1):
        commands = motion_planning(poses[i], poses[i + 1])
        motor_commands.extend(commands)

    return motor_commands

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

    # pose1 = np.array([0.0, 0.0, 1.0, 50.0])
    # pose2 = np.array([0.0, 0.0, 1.0, 120.0])

    # print(f"Pose 1: {normalize_vector(pose1[:3])}")
    # print(f"Pose 2: {normalize_vector(pose2[:3])}")
    
    # commands = motion_planning(pose1, pose2)

    commands = generate_demo()

    try:
        missed = 0
        for i, cmd in enumerate(commands):
            msg = bytes(cmd)
            ser.write(b'\xAA' + msg + b'\x55')
            ser.flush()

            # # read back serial msg
            # echo = ser.read(3)
            # if echo != msg:
            #     missed += 1
            #     print(f"Mismatch at command {i}: sent {msg.hex()} got {echo.hex() if echo else 'nothing'}")

            time.sleep(0.01)

        # print(f"Done. {missed} mismatches out of {len(commands)} commands.")    

    except KeyboardInterrupt:
        print("\nStopped.")

    finally:
        ser.close()


if __name__ == "__main__":
    main()