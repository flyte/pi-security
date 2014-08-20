import json
import zmq
import sys

CONFIG_FILE = "config.json"

if __name__ == "__main__":
    inputs = {}
    outputs = {}
    config = json.load(open(CONFIG_FILE))

    context = zmq.Context()
    poller = zmq.Poller()

    # Set up input sockets
    for host in config["inputs"]:
        s = context.socket(zmq.SUB)
        s.connect(host["uri"])
        s.setsockopt_string(zmq.SUBSCRIBE, host["prefix"]) 
        poller.register(s, zmq.POLLIN)
        host["socket"] = s
        inputs[host["name"]] = host

    # Set up output sockets
    for host in config["outputs"]:
        s = context.socket(zmq.PUSH)
        s.connect(host["uri"])
        host["socket"] = s
        outputs[host["name"]] = host

    while True:
        try:
            socks = dict(poller.poll())
        except KeyboardInterrupt:
            print "\nBye!"
            sys.exit()

        for m in config["maps"]:
            in_host = inputs[m["input"]]
            if in_host["socket"] in socks:
                msg_in = in_host["socket"].recv_string()
                msg_in = msg_in[len(in_host["prefix"]):]
                print "RX from %s: %s" % (in_host["name"], msg_in)
                out_host = outputs[m["output"]]
                msg_out = out_host["prefix"] + msg_in
                print "TX to %s: %s" % (out_host["name"], msg_out)
                out_host["socket"].send_string(msg_out)
