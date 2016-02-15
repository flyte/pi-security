import argparse
from time import sleep

import paho.mqtt.client as mqtt
import pifacedigitalio as pfdio
from transitions import Machine


CHANNEL_PREFIX = "home/garage/"
SET_SUFFIX = "/set"
SECURITY_CHAN = "security"
SWITCHES = (
    ("socket1", 0, False),
    ("socket2", 1, False),
    ("siren", 2, False),
    ("buzzer", 3, False)
)
SWITCH_PINS = {chan: output for chan, output, _ in SWITCHES}
SWITCH_INVERT = {chan: invert for chan, _, invert in SWITCHES}
INPUTS = (
    ("tamper", 0, True),
    ("motion", 1, True)
)
INPUT_PINS = {chan: pin for chan, pin, _ in INPUTS}
INPUT_INVERT = {chan: invert for chan, _, invert in INPUTS}
PAYLOAD_ON = "ON"
PAYLOAD_OFF = "OFF"


def on_connect(client, userdata, flags, rc):
    print("Connected with result code "+str(rc))
    for chan, _, _ in SWITCHES:
        client.subscribe("{}{}{}".format(CHANNEL_PREFIX, chan, SET_SUFFIX))
    set_all_states()


def set_all_states():
    for config in SWITCHES:
        publish_output_state(*config)
    for config in INPUTS:
        publish_input_state(*config)


def get_input_state(inpt, invert):
    return bool(pf.input_pins[inpt].value) != invert


def get_output_state(output, invert):
    return bool(pf.output_pins[output].value) != invert


def publish_input_state(chan, inpt, invert):
    state = get_input_state(inpt, invert)
    publish_state(chan, state)


def publish_output_state(chan, output, invert):
    state = get_output_state(output, invert)
    publish_state(chan, state)


def publish_state(chan, state):
    payload = PAYLOAD_ON if state else PAYLOAD_OFF
    client.publish(
        "{}{}".format(CHANNEL_PREFIX, chan), payload=payload, retain=True)
    print "{} state set to {}".format(chan, payload)


def on_msg(client, userdata, msg):
    chan = msg.topic[len(CHANNEL_PREFIX):-len(SET_SUFFIX)]
    try:
        output = SWITCH_PINS[chan]
    except KeyError:
        print "{} isn't a channel we care about.".format(chan)
    if msg.payload == PAYLOAD_ON:
        pf.output_pins[output].turn_on()
        publish_state(chan, True)
    elif msg.payload == PAYLOAD_OFF:
        pf.output_pins[output].turn_off()
        publish_state(chan, False)
    print "{} {}".format(msg.topic, msg.payload)


class Security(Machine):
    states = "disarmed armed caution sounding".split()
    transitions = [
        "disarm * disarmed".split(),
        "arm disarmed armed".split(),
        "warn armed caution".split(),
        ["sound", ["armed", "caution"], "sounding"],
        ["relax", ["caution", "sounding"], "armed"]
    ]
    initial = "disarmed"
    caution_timeout = None
    sounding_timeout = None

    def __init__(self, stop_event, piface, mqtt_client):
        self._stop_event = stop_event
        self._mqtt = mqtt_client
        Machine.__init__(
            self,
            states=self.states,
            initial=self.initial
        )
        for t in self.transitions:
            self.add_transition(*t, after="update_mqtt")

        # Register our interest in detected movement
        input_listener = pfdio.InputEventListener(chip=piface)
        edge = (pfdio.IODIR_FALLING_EDGE if INPUT_INVERT["motion"]
                else pfdio.IODIR_RISING_EDGE)
        input_listener.register(
            INPUT_PINS["motion"], edge, self.movement)

    def movement(self):
        """ Called when there is movement detected. """
        if self.state == "disarmed":
            print "Movement detected but we're in the 'disarmed' state."
            return
        if self.state == "armed":
            print "Movement detected and we're armed!"

    def update_mqtt(self):
        """ Update mqtt with the current state. """
        self._mqtt.publish(
            "{}{}".format(CHANNEL_PREFIX, SECURITY_CHAN),
            payload=self.state,
            retain=True)


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("user")
    p.add_argument("password")
    p.add_argument("host")
    p.add_argument("--port", type=int, default=1883)
    args = p.parse_args()

    pf = pfdio.PiFaceDigital()

    client = mqtt.Client()
    client.on_connect = on_connect
    client.on_message = on_msg
    client.username_pw_set(args.user, args.password)
    client.connect(args.host, args.port, 60)
    client.loop_start()

    last_states = {inpt: None for _, inpt, _ in INPUTS}
    try:
        while True:
            for chan, inpt, invert in INPUTS:
                state = get_input_state(inpt, invert)
                if state != last_states[inpt]:
                    publish_state(chan, state)
                    last_states[inpt] = state
            sleep(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        client.loop_stop()
