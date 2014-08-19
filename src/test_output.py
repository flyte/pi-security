import pifacedigitalio
import argparse
from time import sleep

if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("output", type=int)
    p.add_argument("--on_period", type=float, default=0.2)
    p.add_argument("--off_period", type=float, default=1.0)
    p.add_argument("--loops", type=int, default=1)

    args = p.parse_args()
    pf = pifacedigitalio.PiFaceDigital()

    try:
        i = 0
        while i < args.loops:
            print("******** Loop %s ********" % (i+1))
            print("Output %s ON" % i)
            pf.output_pins[args.output].turn_on()

            print("Sleeping %s" % args.on_period)
            sleep(args.on_period)

            print("Output %s OFF" % i)
            pf.output_pins[args.output].turn_off()

            print("Sleeping %s" % args.off_period)
            sleep(args.off_period)
            i += 1
    except KeyboardInterrupt:
        pf.output_pins[args.output].turn_off()
    except Exception as e:
        print("Got exception:")
        raise
