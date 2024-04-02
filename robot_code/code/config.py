# config.py

config = {
    "motors": {
        "left_front": {"pin_pwm": "P13", "pin_dir": "D4", "reverse": False},
        "right_front": {"pin_pwm": "P12", "pin_dir": "D5", "reverse": False},
        "left_rear": {"pin_pwm": "P8", "pin_dir": "D11", "reverse": False},
        "right_rear": {"pin_pwm": "P9", "pin_dir": "D15", "reverse": False}
    },
    "ultrasonic_servo_offset": 0,
}
