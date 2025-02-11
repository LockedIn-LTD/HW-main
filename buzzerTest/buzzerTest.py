import Jetson.GPIO as GPIO
import numpy as np
import time

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

moonlight_sonata_frequencies = [
    138.59, 207.65, 277.18, 207.65, 138.59, 207.65, 233.08, 207.65,  # C#2, G#2, C#3, G#2, C#2, G#2, B2, G#2
    146.83, 220.00, 293.66, 220.00, 146.83, 220.00, 246.94, 220.00,  # D2, A2, D3, A2, D2, A2, B2, A2
    164.81, 246.94, 329.63, 246.94, 164.81, 246.94, 277.18, 246.94,  # E2, B2, E3, B2, E2, B2, C#3, B2
    130.81, 195.99, 261.63, 195.99, 130.81, 195.99, 220.00, 195.99,  # C2, G2, C3, G2, C2, G2, A2, G2
    138.59, 207.65, 277.18, 207.65, 138.59, 207.65, 233.08, 207.65,  # C#2, G#2, C#3, G#2, C#2, G#2, B2, G#2
    261.63, 246.94, 220.00, 207.65, 220.00, 246.94, 261.63, 293.66,  # Right-hand melody: C4, B3, A3, G#3, A3, B3, C4, D4
    311.13, 349.23, 391.99, 440.00, 466.16, 440.00, 391.99, 349.23   # Melody continues: D#4, F4, G4, A4, A#4, A4, G4, F4
]

mario_theme_frequencies = [
    659, 659, 10, 659, 10, 523, 659, 10, 784, 10, 392, 10, 523, 10, 392, 10, 330,
    10, 440, 494, 466, 440, 392, 659, 784, 880, 698, 784, 10, 523, 587, 494
]



output_pin = output_pins.get(GPIO.model, None)
if output_pin is None:
    raise Exception('PWM not supported on this board')


def generate_c_major_frequencies(octaves=2, base_octave=4, a440=440.0):
    # Define the semitone offsets of the C major scale relative to C
    c_major_intervals = [0, 2, 4, 5, 7, 9, 11]  # C, D, E, F, G, A, B
    
    # Calculate C4 frequency from A440
    c4_frequency = a440 * 2**(-9/12)  # C4 is 9 semitones below A4 (440 Hz)

    # Generate frequencies for the given number of octaves
    frequencies = []
    for octave in range(base_octave, base_octave + octaves):
        for interval in c_major_intervals:
            note_frequency = c4_frequency * 2**(interval/12) * 2**(octave-4)
            frequencies.append(round(note_frequency, 2))  # Round for readability
    
    return frequencies


def main():
    # Pin Setup:
    # Board pin-numbering scheme
    GPIO.setmode(GPIO.BOARD)
    # set pin as an output pin with optional initial state of HIGH
    GPIO.setup(output_pin, GPIO.OUT, initial=GPIO.LOW)
    p = GPIO.PWM(output_pin, 50)
    val = 100
    p.start(75)
    

    print("PWM running. Press CTRL+C to exit.")
    try:
        while True:
            #freq = generate_c_major_frequencies(octaves=3)
            for val in mario_theme_frequencies:
                time.sleep(0.125)
                print(val)
                p.ChangeFrequency(val)
            
    finally:
        p.stop()
        GPIO.cleanup()

if __name__ == '__main__':
    main()