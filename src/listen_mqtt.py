import argparse

import paho.mqtt.client as mqtt


CHANNEL_PREFIX = "home/garage/"
CHANNELS = (
    "socket1",
    "socket2",
    "siren",
    "buzzer",
    "motion"
)


def on_connect(client, userdata, flags, rc):
    print("Connected with result code "+str(rc))
    for chan in CHANNELS:
        client.subscribe("{}{}".format(CHANNEL_PREFIX, chan))


def on_msg(client, userdata, msg):
    print "{} {}".format(msg.topic, msg.payload)


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("user")
    p.add_argument("password")
    p.add_argument("host")
    p.add_argument("--port", type=int, default=1883)
    args = p.parse_args()

    client = mqtt.Client()
    client.on_connect = on_connect
    client.on_message = on_msg
    client.username_pw_set(args.user, args.password)
    client.connect(args.host, args.port, 60)

    client.loop_forever()
