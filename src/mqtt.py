import argparse
from time import sleep
from threading import Timer

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
MOVEMENT_TO_WARNING_SECS = 4
WARNING_TO_SOUNDING_SECS = 10
SOUNDING_SECS = 20


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
    states = "disarmed armed warning sounding".split()
    transitions = [
        "disarm * disarmed".split(),
        "arm disarmed armed".split(),
        "warn armed warning".split(),
        ["sound", ["armed", "warning"], "sounding"],
        ["relax", ["warning", "sounding"], "armed"]
    ]
    initial = "disarmed"
    _warning_timer = None
    _sounding_timer = None
    _sounding_stop_timer = None

    def __init__(self, stop_event, piface, mqtt_client):
        self._stop_event = stop_event
        self._pf = piface
        self._mqtt = mqtt_client
        Machine.__init__(
            self,
            states=self.states,
            initial=self.initial
        )
        for t in self.transitions:
            self.add_transition(*t, after="update_mqtt_state")

        # Register our interest in detected movement
        input_listener = pfdio.InputEventListener(chip=piface)
        edge = (pfdio.IODIR_FALLING_EDGE if INPUT_INVERT["motion"]
                else pfdio.IODIR_RISING_EDGE)
        input_listener.register(
            INPUT_PINS["motion"], edge, self.movement)

    def movement(self):
        """ Called when there is movement detected. """
        if self.is_disarmed():
            print "Movement detected but we're in the 'disarmed' state."
            return
        if self.is_armed():
            print "Movement detected and we're armed! Starting warning timer.."
            # Start the warning timer
            self._warning_timer = Timer(MOVEMENT_TO_WARNING_SECS, self.warn)
            self._warning_timer.start()

    def no_movement(self):
        """ Called when the movement stops. """
        try:
            self._warning_timer.cancel()
            self._warning_timer = None
            print "Stopped warning timer."
        except AttributeError:
            # _warning_timer must have been None
            print "No warning timer to stop."
            pass

    def update_mqtt_state(self):
        """ Update mqtt with the current state. """
        print "Changed to state %s" % self.state
        self._mqtt.publish(
            "{}{}".format(CHANNEL_PREFIX, SECURITY_CHAN),
            payload=self.state,
            retain=True)

    def on_enter_warning(self):
        """ Sound the buzzer and start the sounding timer. """
        # self._pf.output_pins[SWITCH_PINS["buzzer"]].turn_on()
        self._mqtt.publish(
            "{}{}".format(CHANNEL_PREFIX, "buzzer"),
            payload=PAYLOAD_ON,
            retain=True)
        self._sounding_timer = Timer(WARNING_TO_SOUNDING_SECS, self.sound)
        self._sounding_timer.start()

    def on_exit_warning(self):
        """ Stop sounding the buzzer. """
        # self._pf.output_pins[SWITCH_PINS["buzzer"]].turn_off()
        self._mqtt.publish(
            "{}{}".format(CHANNEL_PREFIX, "buzzer"),
            payload=PAYLOAD_OFF,
            retain=True)
        try:
            self._sounding_timer.cancel()
            self._sounding_timer = None
        except AttributeError:
            # _sounding_timer must have been None
            pass

    def on_enter_sounding(self):
        """ Sound the siren. """
        # self._pf.output_pins[SWITCH_PINS["siren"]].turn_on()
        self._mqtt.publish(
            "{}{}".format(CHANNEL_PREFIX, "siren"),
            payload=PAYLOAD_ON,
            retain=True)
        self._sounding_stop_timer = Timer(SOUNDING_SECS, self.relax)
        self._sounding_stop_timer.start()

    def on_exit_sounding(self):
        """ Stop sounding the siren. """
        # self._pf.output_pins[SWITCH_PINS["siren"]].turn_off()
        self._mqtt.publish(
            "{}{}".format(CHANNEL_PREFIX, "siren"),
            payload=PAYLOAD_OFF,
            retain=True)
        try:
            self._sounding_off_timer.cancel()
            self._sounding_off_timer = None
        except AttributeError:
            # _sounding_off_timer must have been None
            pass


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
