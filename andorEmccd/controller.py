#!/usr/bin/env python3.5
import argparse
import sys

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

    dev = AndorEmccd()
    simple_server_loop({"camera": dev}, args.bind, args.port)


if __name__ == "__main__":
    main()