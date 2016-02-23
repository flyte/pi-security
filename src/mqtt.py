import argparse
from time import sleep
from threading import Timer
from functools import partial

import paho.mqtt.client as mqtt
import pifacedigitalio as pfdio
from transitions import Machine


CHANNEL_PREFIX = "home/garage/"
SET_SUFFIX = "/set"
SECURITY_CHAN = "alarm"
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
MOVEMENT_TO_WARNING_SECS = 5
WARNING_TO_SOUNDING_SECS = 20
SOUNDING_SECS = 120
RECONNECT_DELAY_SECS = 2
DEFAULT_MQTT_PORT = 1883


def on_connect(client, userdata, flags, rc):
    print "Connected with result code " + str(rc)
    for chan, _, _ in SWITCHES:
        client.subscribe(
            "{}{}{}".format(CHANNEL_PREFIX, chan, SET_SUFFIX), qos=1)
    client.subscribe(
        "{}{}{}".format(CHANNEL_PREFIX, SECURITY_CHAN, SET_SUFFIX), qos=1)
    set_all_states()

    client.subscribe("{}{}{}".format(CHANNEL_PREFIX, "movement", SET_SUFFIX))
    client.subscribe("{}{}{}".format(CHANNEL_PREFIX, "no_movement", SET_SUFFIX))


def on_disconnect(client, userdata, rc):
    print "Disconnected from MQTT server with code: %s" % rc
    while rc != 0:
        sleep(RECONNECT_DELAY_SECS)
        rc = client.reconnect()


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

    if chan == "movement":
        security.movement()
    elif chan == "no_movement":
        security.no_movement()

    if chan == SECURITY_CHAN:
        if msg.payload == "armed":
            print "Arming security..."
            security.arm()
        elif msg.payload == "disarmed":
            print "Disarming security..."
            security.disarm()
    else:
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


def change_output(pin, state):
    if state:
        pf.output_pins[pin].turn_on()
    else:
        pf.output_pins[pin].turn_off()


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
    _warning_timer = Timer(None, None)
    _sounding_timer = Timer(None, None)
    _sounding_stop_timer = Timer(None, None)

    def __init__(self, mqtt_client, buzzer_start,
                 buzzer_stop, siren_start, siren_stop):
        self._mqtt = mqtt_client
        self._buzzer_start = buzzer_start
        self._buzzer_stop = buzzer_stop
        self._siren_start = siren_start
        self._siren_stop = siren_stop
        Machine.__init__(
            self,
            states=self.states,
            initial=self.initial
        )
        for t in self.transitions:
            self.add_transition(*t, after="update_mqtt_state")
        self.update_mqtt_state()

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
        was_running = self._warning_timer.is_alive()
        self._warning_timer.cancel()
        if was_running:
            print "Cancelled warning timer."
        else:
            print "Warning timer was not running."

    def update_mqtt_state(self):
        """ Update mqtt with the current state. """
        print "Changed to state %s" % self.state
        self._mqtt.publish(
            "{}{}".format(CHANNEL_PREFIX, SECURITY_CHAN),
            payload=self.state,
            retain=True)

    def _cancel_all_timers(self):
        """ Cancel all timers. """
        for timer in (self._warning_timer,
                      self._sounding_timer,
                      self._sounding_stop_timer):
            timer.cancel()

    def on_enter_warning(self):
        """ Sound the buzzer and start the sounding timer. """
        self._buzzer_start()
        self._mqtt.publish(
            "{}{}".format(CHANNEL_PREFIX, "buzzer"),
            payload=PAYLOAD_ON,
            retain=True)
        self._sounding_timer = Timer(WARNING_TO_SOUNDING_SECS, self.sound)
        self._sounding_timer.start()

    def on_exit_warning(self):
        """ Stop sounding the buzzer. """
        self._buzzer_stop()
        self._mqtt.publish(
            "{}{}".format(CHANNEL_PREFIX, "buzzer"),
            payload=PAYLOAD_OFF,
            retain=True)
        self._cancel_all_timers()

    def on_enter_sounding(self):
        """ Sound the siren. """
        self._siren_start()
        self._mqtt.publish(
            "{}{}".format(CHANNEL_PREFIX, "siren"),
            payload=PAYLOAD_ON,
            retain=True)
        self._sounding_stop_timer = Timer(SOUNDING_SECS, self.relax)
        self._sounding_stop_timer.start()

    def on_exit_sounding(self):
        """ Stop sounding the siren. """
        self._siren_stop()
        self._mqtt.publish(
            "{}{}".format(CHANNEL_PREFIX, "siren"),
            payload=PAYLOAD_OFF,
            retain=True)
        self._cancel_all_timers()


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("user")
    p.add_argument("password")
    p.add_argument("host")
    p.add_argument("--port", type=int, default=DEFAULT_MQTT_PORT)
    args = p.parse_args()

    pf = pfdio.PiFaceDigital()

    client = mqtt.Client(client_id="garage-security", clean_session=False)
    client.on_connect = on_connect
    client.on_message = on_msg
    client.username_pw_set(args.user, args.password)
    client.connect(args.host, args.port, 60)
    client.loop_start()

    buzzer_start = partial(change_output, SWITCH_PINS["buzzer"], True)
    buzzer_stop = partial(change_output, SWITCH_PINS["buzzer"], False)
    siren_start = partial(change_output, SWITCH_PINS["siren"], True)
    siren_stop = partial(change_output, SWITCH_PINS["siren"], False)
    security = Security(
        client, buzzer_start, buzzer_stop, siren_start, siren_stop)

    last_states = {inpt: None for _, inpt, _ in INPUTS}
    try:
        while True:
            for chan, inpt, invert in INPUTS:
                state = get_input_state(inpt, invert)
                if state != last_states[inpt]:
                    publish_state(chan, state)
                    last_states[inpt] = state
                    if chan == "motion":
                        if state:
                            security.movement()
                        else:
                            security.no_movement()
            sleep(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        client.loop_stop()
