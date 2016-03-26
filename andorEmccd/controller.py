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
    return parser


def main():
    args = get_argparser().parse_args()
    init_logger(args)

    def ping(self):
        return True
    AndorEmccd.ping = ping
    dev = AndorEmccd()

    try:
        dev.set_temperature(-80)
        print("Waiting for camera to cool down")
        while dev.get_temperature() > -70:
            time.sleep(1)
        print("Camera cool")
        simple_server_loop({"camera": dev}, args.bind, args.port)
    finally:
        dev.close()

if __name__ == "__main__":
    main()