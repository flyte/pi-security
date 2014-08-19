import pifacedigitalio
import argparse
from time import sleep

if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("input", type=int)

    args = p.parse_args()
    pf = pifacedigitalio.PiFaceDigital()
    
    print(pf.input_pins[args.input].value)
