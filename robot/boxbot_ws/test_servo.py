# import RPi.GPIO as GPIO
# import time

# GPIO.setmode(GPIO.BCM)

# GPIO.setup(24, GPIO.OUT)

# p = GPIO.PWM(24, 50)

# p.start(3)

# p.ChangeDutyCycle(5)
# time.sleep(3)
# p.ChangeDutyCycle(50) # may need to be adjusted 
# time.sleep(6)
# p.ChangeDutyCycle(10)
# time.sleep(3)

from gpiozero import PWMOutputDevice
from time import sleep

# In gpiozero, pin numbering defaults to BCM
# We initialize pin 24 with a frequency of 50Hz
p = PWMOutputDevice(24, frequency=50)

# gpiozero uses a 0.0 to 1.0 scale for duty cycle (value / 100)
p.value = 0.03  # Equivalent to 3%

p.value = 0.05  # Equivalent to 5%
sleep(3)

p.value = 0.50  # Equivalent to 50%
sleep(6)

p.value = 0.10  # Equivalent to 10%
sleep(3)

