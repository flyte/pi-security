import pifacedigitalio
from time import sleep

input_pin = 1
output_pin = 0

if __name__ == "__main__":
    pf = pifacedigitalio.PiFaceDigital()
    triggered = False
    while True:
        if pf.input_pins[input_pin].value == 0 and not triggered:
            print("Movement!")
            pf.output_pins[output_pin].turn_on()
            triggered = True
        elif pf.input_pins[input_pin].value == 1 and triggered:
            print("Reset")
            pf.output_pins[output_pin].turn_off()
            triggered = False
    sleep(0.1)
