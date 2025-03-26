import Jetson.GPIO as GPIO
import time

# Disable warnings
GPIO.setwarnings(False)

# Define output pin for your Jetson board
output_pins = {
    'JETSON_XAVIER': 18,
    'JETSON_NANO': 33,
    'JETSON_NX': 33,
    'CLARA_AGX_XAVIER': 18,
    'JETSON_TX2_NX': 32,
    'JETSON_ORIN': 18,
    'JETSON_ORIN_NX': 33,
    'JETSON_ORIN_NANO': 15
}

output_pin = output_pins.get(GPIO.model, None)
if output_pin is None:
    raise Exception("PWM not supported on this board.")

# Cleanup in case GPIO is already in use
GPIO.cleanup()
GPIO.setmode(GPIO.BOARD)

# Make sure the pin is correctly set up
print(f"Setting up GPIO {output_pin} as OUTPUT...")
GPIO.setup(output_pin, GPIO.OUT)
time.sleep(1)  # Give it a second to settle

# Start PWM with a moderate frequency (PWM still works even when toggling HIGH/LOW)
pwm = GPIO.PWM(output_pin, 440)  # 440 Hz tone
pwm.start(50)  # 50% duty cycle

try:
    print("Testing buzzer. Ctrl+C to stop.")
    while True:
        print(f"Setting GPIO {output_pin} HIGH (silent if active-low, buzzing if active-high)")
        GPIO.output(output_pin, GPIO.HIGH)
        time.sleep(5)
        
        print(f"Setting GPIO {output_pin} LOW (buzzing if active-low, silent if active-high)")
        GPIO.output(output_pin, GPIO.LOW)
        time.sleep(5)

except KeyboardInterrupt:
    print("\nTest stopped by user.")
finally:
    print("Cleaning up GPIO...")
    pwm.stop()
    GPIO.cleanup()

