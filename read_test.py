import serial
import struct
import time

BUF_SIZE = 59

class GimbalGetAnglesExtReq:
    def __init__(self):
        self.timestamp = 0
        self.rollIMUangle = 0
        self.pitchIMUangle = 0
        self.yawIMUangle = 0
        self.rollIMUspeed = 0
        self.pitchIMUspeed = 0
        self.yawIMUspeed = 0
        self.rollStatorRotorAngle = 0
        self.pitchStatorRotorAngle = 0
        self.yawStatorRotorAngle = 0

def main():
    port = "/dev/ttyUSB1"
    baud_rate = 115200
    command = bytearray([0x3e, 0x3d, 0x00, 0x3d, 0x00])

    try:
        ser = serial.Serial(port, baud_rate, timeout=1)
        ser.flush()

        while True:
            # Write the command to the serial port
            # ser.write(command)
            # print("Sent command")

            # Read BUF_SIZE bytes from the serial port
            buf = ser.read(BUF_SIZE)

            if len(buf) != BUF_SIZE:
                print("Received incomplete data")
                continue

            # Display each byte in the buffer
            # for i, byte in enumerate(buf):
            #     print(f"{i} {hex(byte)}")

            # Extract pitch and yaw angles from the buffer
            pitch = struct.unpack_from('<h', buf, 24)[0]
            yaw = struct.unpack_from('<h', buf, 42)[0]

            # Store values in the structure and convert angles
            t = GimbalGetAnglesExtReq()
            t.pitchIMUangle = pitch
            t.yawIMUangle = yaw

            # Print the angles after conversion
            print(f"pitch : {t.pitchIMUangle * 0.02197}")
            print(f"yaw : {t.yawIMUangle * 0.02197}")

            time.sleep(0.01)


    except KeyboardInterrupt:
        print("Exiting...")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()

if __name__ == "__main__":
    main()
