import pifacedigitalio
from subprocess import Popen

cmd = "/opt/vc/bin/raspistill -vf -w 640 -h 480 -o mypic.jpg -t 0".split()

if __name__ == "__main__":
    pf = pifacedigitalio.PiFaceDigital()
    cap_proc = None
    try:
        while True:
            movement = pf.input_pins[0].value == 0
            if movement and not cap_proc:
                cap_proc = Popen(cmd)
            elif not movement:
                if cap_proc:
                    cap_proc.terminate()
                cap_proc = None
        sleep(0.1)
    finally:
        if cap_proc:
            cap_proc.kill()
