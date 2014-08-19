import zmq
import pifacedigitalio
import argparse
from time import sleep

if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("input", type=int)
    p.add_argument("prefix", type=str)
    p.add_argument("port", type=int)
    p.add_argument("--delay", type=float, default=0.1)
    args  = p.parse_args()

    pf = pifacedigitalio.PiFaceDigital()
    context = zmq.Context()
    socket = context.socket(zmq.PUB)
    socket.bind("tcp://*:%s" % args.port)

    last_value = None
    while True:
        value = pf.input_pins[args.input].value
        if value != last_value:
            msg = "%s%s" % (args.prefix, value)
            print(msg)
            socket.send_string(msg)
            last_value = value
        sleep(args.delay)
