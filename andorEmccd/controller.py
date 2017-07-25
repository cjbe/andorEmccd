#!/usr/bin/env python3.5
import argparse
import sys
import time

from artiq.protocols.pc_rpc import simple_server_loop
from artiq.tools import verbosity_args, simple_network_args, init_logger
from .andorEmccd import AndorEmccd


def get_argparser():
    parser = argparse.ArgumentParser()
    simple_network_args(parser, 4010)
    verbosity_args(parser)
    parser.add_argument("--temp", default=-80, type=int)
    return parser


def main():
    args = get_argparser().parse_args()
    init_logger(args)

    def ping(self):
        return True
    AndorEmccd.ping = ping
    dev = AndorEmccd()

    try:
        dev.set_temperature(args.temp)
        print("Waiting for camera to cool down to {} C ...".format(args.temp))
        while dev.get_temperature() > args.temp + 10:
            time.sleep(1)
        print("Camera cool")
        simple_server_loop({"camera": dev}, args.bind, args.port)
    finally:
        dev.close()

if __name__ == "__main__":
    main()