# motor.py
import RPi.GPIO as GPIO

class Motor:
    def __init__(self, pwm_pin, dir_pin, is_reversed=False):
        self.pwm_pin = pwm_pin
        self.dir_pin = dir_pin
        self.is_reversed = is_reversed

        GPIO.setup(self.pwm_pin, GPIO.OUT)
        GPIO.setup(self.dir_pin, GPIO.OUT)

        self.pwm = GPIO.PWM(self.pwm_pin, 100)  # Set frequency to 100 Hz
        self.pwm.start(0)

    def set_power(self, power):
        direction = GPIO.LOW if power >= 0 else GPIO.HIGH
        if self.is_reversed:
            direction = not direction

        GPIO.output(self.dir_pin, direction)
        
        # Adjust power to be within PWM range and apply
        power = abs(power)
        power = max(min(100, power), 0)  # Constrain between 0 and 100
        self.pwm.ChangeDutyCycle(power)
