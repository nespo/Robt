# speed.py
import RPi.GPIO as GPIO
import time, math
import threading

class Speed:
    def __init__(self, pin):
        self.pin = pin
        GPIO.setup(self.pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

        self.speed = 0
        self.timer = threading.Thread(target=self.measure_speed)
        self.running = False

    def measure_speed(self):
        count = 0
        last_state = False
        start_time = time.time()

        while self.running:
            current_state = GPIO.input(self.pin)
            if current_state != last_state:
                count += 1
                last_state = current_state

            if time.time() - start_time >= 1:  # Every 1 second
                # Calculate speed, example calculation, adjust formula as needed
                self.speed = count / 20.0  # Placeholder calculation
                count = 0
                start_time = time.time()

    def start(self):
        self.running = True
        self.timer.start()

    def stop(self):
        self.running = False
        self.timer.join()

    def get_speed(self):
        return self.speed
