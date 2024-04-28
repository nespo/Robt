class MockGPIO:
    BCM = 'BCM'
    OUT = 'OUT'
    IN = 'IN'
    HIGH = 'HIGH'
    LOW = 'LOW'
    RISING = 'RISING'   # For rising edge detection
    FALLING = 'FALLING' # For falling edge detection
    BOTH = 'BOTH'       # For both rising and falling edge detection
    PUD_UP = 'PUD_UP'   # Pull-up resistor
    PUD_DOWN = 'PUD_DOWN' # Pull-down resistor

    @staticmethod
    def setmode(mode):
        print(f"GPIO mode set to {mode}")

    @staticmethod
    def setup(pin, mode, pull_up_down=None):
        print(f"Pin {pin} set up as {mode}, with pull-up/down {pull_up_down}")

    @staticmethod
    def add_event_detect(pin, edge_type, callback=None, bouncetime=200):
        print(f"Event detect added to pin {pin} for {edge_type} edge with bounce time {bouncetime}")

    @staticmethod
    def remove_event_detect(pin):
        print(f"Event detect removed from pin {pin}")

    @staticmethod
    def output(pin, state):
        print(f"Pin {pin} set to {state}")

    @staticmethod
    def cleanup():
        print("Cleaning up GPIO pins")
