import serial
import struct
import time
import logging

logging.basicConfig(level=logging.DEBUG)

class RPLidar:
    def __init__(self, port, baudrate=115200, timeout=3):
        self.ser = serial.Serial(port, baudrate=baudrate, timeout=timeout)
        self.ser.reset_input_buffer()
        logging.debug(f"Serial port {port} opened with baudrate {baudrate}")

    def send_command(self, cmd, payload=[]):
        sync_byte = 0xA5
        request = bytearray([sync_byte, cmd]) + bytearray(payload)
        self.ser.write(request)
        logging.debug(f"Command sent: {request.hex()}")

    def start_motor(self):
        self.send_command(0xF0, [0x02, 0x94])
        logging.debug("Motor start command issued")

    def stop_motor(self):
        self.send_command(0xF0, [0x00, 0x00])
        logging.debug("Motor stop command issued")

    def get_health(self):
        self.send_command(0x52)
        response = self.ser.read(10)
        logging.debug(f"Health Response: {response}")
        if len(response) < 7:
            logging.error("Incomplete health status response")
            return None
        if response[0] != 0xA5:
            logging.error("Invalid start byte in health status response")
            return None
        health_status = struct.unpack('<BBB', response[4:7])
        return health_status

    def read_scan(self):
        self.send_command(0x20)
        time.sleep(1)
        data = []
        while True:
            response = self.ser.read(5)
            logging.debug(f"Scan Data Chunk: {response}")
            if not response:
                break
            data.append(response)
        return data

    def close(self):
        self.ser.close()
        logging.debug("Serial port closed")

# Example usage
if __name__ == "__main__":
    lidar = RPLidar('/dev/ttyUSB0')
    try:
        lidar.start_motor()
        health = lidar.get_health()
        scans = lidar.read_scan()
        lidar.stop_motor()
    finally:
        lidar.close()
