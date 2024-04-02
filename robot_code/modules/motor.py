import RPi.GPIO as GPIO
from pin import Pin  # Make sure to import the Pin class
from pwm import PWM  # Make sure to import the PWM class

class Motor:
    def __init__(self, pin_pwm, pin_dir, reverse=False):
        # Convert pin identifiers to GPIO pins
        self.pwm_pin = PWM(pin_pwm)
        self.dir_pin = Pin(pin_dir, Pin.OUT)
        self.is_reversed = reverse

    def set_power(self, power):
        # Set direction based on power sign and reverse flag
        direction = GPIO.HIGH if (power > 0) ^ self.is_reversed else GPIO.LOW
        self.dir_pin.value(direction)
        
        # Adjust power to be within 0-100% for PWM
        power = abs(power)
        power = max(min(100, power), 0)  # Constrain between 0 and 100
        self.pwm_pin.pulse_width_percent(power)
