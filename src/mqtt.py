import argparse
from time import sleep

import paho.mqtt.client as mqtt
import pifacedigitalio


CHANNEL_PREFIX = "home/garage/"
SET_SUFFIX = "/set"
SWITCHES = (
    ("socket1", 0, False),
    ("socket2", 1, False),
    ("siren", 2, False),
    ("buzzer", 3, False)
)
SWITCHES_CHAN_TO_OUTPUT = {chan: output for chan, output, _ in SWITCHES}
INPUTS = (
    ("tamper", 0, True),
    ("motion", 1, True)
)
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
    client.publish("{}{}".format(CHANNEL_PREFIX, chan), payload=payload, retain=True)
    print "{} state set to {}".format(chan, payload)


def on_msg(client, userdata, msg):
    chan = msg.topic[len(CHANNEL_PREFIX):-len(SET_SUFFIX)]
    try:
        output = SWITCHES_CHAN_TO_OUTPUT[chan]
    except KeyError:
        print "{} isn't a channel we care about.".format(chan)
    if msg.payload == PAYLOAD_ON:
        pf.output_pins[output].turn_on()
        publish_state(chan, True)
    elif msg.payload == PAYLOAD_OFF:
        pf.output_pins[output].turn_off()
        publish_state(chan, False)
    print "{} {}".format(msg.topic, msg.payload)


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("user")
    p.add_argument("password")
    p.add_argument("host")
    p.add_argument("--port", type=int, default=1883)
    args = p.parse_args()

    pf = pifacedigitalio.PiFaceDigital()

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
