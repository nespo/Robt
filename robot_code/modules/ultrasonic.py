import time, os, sys
# Adjust the sys.path to include the parent directory of robot_code
script_dir = os.path.dirname(__file__)
parent_dir = os.path.join(script_dir, '..', '..')
sys.path.append(os.path.abspath(parent_dir))
from robot_code.code.config import config
from servo import Servo
from pwm import PWM
from pin import Pin

class Ultrasonic:
    """A class to control an ultrasonic sensor with servo for scanning."""
    ANGLE_RANGE = 180
    STEP = 18
    PULSE_DELAY = 0.000015
    TRIG_PRE_DELAY = 0.01
    SERVO_SET_DELAY = 0.04

    def __init__(self, trig, echo, timeout=0.01):
        """
        Initializes the ultrasonic sensor with servo.

        :param trig: Trigger pin for the ultrasonic sensor.
        :param echo: Echo pin for the ultrasonic sensor.
        :param timeout: Timeout for echo signal.
        """
        self.timeout = timeout
        self.trig = trig
        self.echo = echo
        self.servo = Servo(PWM("P0"), offset=10)
        self.angle_distance = [0, 0]
        self.current_angle = 0
        self.max_angle = self.ANGLE_RANGE / 2
        self.min_angle = -self.ANGLE_RANGE / 2
        self.scan_list = []

    def get_distance(self):
        """Sends a pulse and calculates the distance based on the pulse's travel time."""
        self.trig.low()
        time.sleep(self.TRIG_PRE_DELAY)
        self.trig.high()
        time.sleep(self.PULSE_DELAY)
        self.trig.low()

        pulse_end = pulse_start = time.time()
        while self.echo.value() == 0:
            pulse_start = time.time()
            if pulse_start - pulse_end > self.timeout:
                return -1  # Timeout
        while self.echo.value() == 1:
            pulse_end = time.time()
            if pulse_end - pulse_start > self.timeout:
                return -2  # Timeout

        duration = pulse_end - pulse_start
        distance = round(duration * 340 / 2 * 100, 2)  # Speed of sound at 20°C in air is 340 m/s
        return distance

    def get_distance_at(self, angle):
        """Moves the servo to a specific angle and measures the distance."""
        self.servo.set_angle(angle)
        time.sleep(self.SERVO_SET_DELAY)  # Wait for the servo to reach the position
        distance = self.get_distance()
        self.angle_distance = [angle, distance]
        return distance

    def get_status_at(self, angle, ref1=35, ref2=10):
        """
        Gets the distance at a specific angle and returns a status based on reference distances.

        :param angle: Angle at which to measure the distance.
        :param ref1: Reference distance for status.
        :param ref2: Second reference distance for a different status.
        :return: Status based on the measured distance.
        """
        dist = self.get_distance_at(angle)
        if dist > ref1 or dist == -2:
            return 2  # Clear or timeout
        elif dist > ref2:
            return 1  # Warning
        else:
            return 0  # Danger/close

    def scan_step(self, ref):
        """Performs a scanning step and updates the scanning list based on obstacles detection."""
        if self.current_angle >= self.max_angle:
            self.current_angle = self.max_angle
            us_step = -self.STEP
        elif self.current_angle <= self.min_angle:
            self.current_angle = self.min_angle
            us_step = self.STEP
        else:
            us_step = self.STEP if self.current_angle < self.max_angle else -self.STEP

        self.current_angle += us_step
        status = self.get_status_at(self.current_angle, ref1=ref)

        self.scan_list.append(status)
        if self.current_angle in [self.min_angle, self.max_angle]:
            if us_step < 0:
                self.scan_list.reverse()
            temp_scan_list = self.scan_list.copy()
            self.scan_list = []  # Reset scan list for next full scan
            return temp_scan_list
        return False
