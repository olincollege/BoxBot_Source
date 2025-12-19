import time
from gpiozero import PWMOutputDevice, OutputDevice
from typing import List

# Servo config
PWM_FREQ = 50
DUTYCYCLE = 3

class StepperL298N4Pin:
    """
    Controls a 4-pin stepper motor using the L298N driver and gpiozero.
    
    Uses the full step drive sequence.
    """
    # Define the 4-step sequence (Full Step Mode)
    SEQUENCE: List[List[int]] = [
        [1, 0, 1, 0],
        [0, 1, 1, 0],
        [0, 1, 0, 1],
        [1, 0, 0, 1]
    ]

    def __init__(self, pin1: int, pin2: int, pin3: int, pin4: int, delay: float = 0.003):
        """
        Initializes the stepper motor.
        
        :param pin1, pin2, pin3, pin4: The BCM pin numbers connected to the L298N driver inputs.
        :param delay: Time delay between steps (controls motor speed).
        """
        # Initialize gpiozero OutputDevice for each pin
        # We allow initial_value=False to ensure they start low/off
        self.devices: List[OutputDevice] = [
            OutputDevice(pin1, initial_value=False),
            OutputDevice(pin2, initial_value=False),
            OutputDevice(pin3, initial_value=False),
            OutputDevice(pin4, initial_value=False)
        ]
        
        self.delay: float = delay
        self.index: int = 0

    def step(self, direction: str):
        """
        Performs one step in the specified direction ("forward" or "backward").
        """
        # Determine the next index in the sequence
        if direction == "forward":
            self.index = (self.index + 1) % 4
        elif direction == "backward":
            self.index = (self.index - 1) % 4
        else:
            return  # Do nothing for invalid direction

        # Get the sequence array for the current step
        seq: List[int] = self.SEQUENCE[self.index]

        # Apply the values to the physical pins
        for device, val in zip(self.devices, seq):
            # device.value accepts 0 or 1 (or True/False)
            device.value = val

        # Pause to control the stepping speed
        time.sleep(self.delay)

    def hold(self):
        """
        Turns off all coils to stop the motor and prevent power drain.
        """
        for device in self.devices:
            device.off()

    def cleanup(self):
        """
        Releases the GPIO pins.
        """
        for device in self.devices:
            device.close()

class ContinousServo:
    def __init__(self, pin1, pin2):
        # Create servo objects using the generic PWMOutputDevice
        # gpiozero uses normalized values (0.0 to 1.0)
        self.servo1 = PWMOutputDevice(pin1, frequency=PWM_FREQ)
        self.servo2 = PWMOutputDevice(pin2, frequency=PWM_FREQ)

        # Start servos (equivalent to .start(DUTYCYCLE))
        self.servo1.value = DUTYCYCLE / 100
        self.servo2.value = DUTYCYCLE / 100

        self.stop()
        
    def lift(self):
        # 5% duty cycle -> 0.05
        self.servo1.value = 0.05
        time.sleep(0.1) # Small delay to prevent peak current overlap 
        self.servo2.value = 0.05

    def lower(self):
        # 10% duty cycle -> 0.10
        self.servo1.value = 0.10
        time.sleep(0.1) # Small delay to prevent peak current overlap 
        self.servo2.value = 0.10

    def stop(self):
        # 15% duty cycle -> 0.15
        self.servo1.value = 0.15
        time.sleep(0.1) # Small delay to prevent peak current overlap 
        self.servo2.value = 0.15

    def shutdown(self):
        # Close the connections to release the pins
        self.servo1.close()
        self.servo2.close()
        print("GPIO cleanup complete.")