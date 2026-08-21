#!/usr/bin/env python3

import argparse
import pathlib

from gnuradio import blocks
from gnuradio import gr
from gnuradio import iqtaxi


class M300RxCapture(gr.top_block):
    def __init__(self, args):
        super().__init__("M300 IQTAXI RX Capture")

        sample_count = max(1, round(args.sample_rate * args.seconds))
        source = iqtaxi.rx_source(
            "M300_XDMA",
            args.addr,
            args.sample_rate,
            args.center_freq,
            args.gain,
            args.work_samples,
        )
        head = blocks.head(gr.sizeof_gr_complex, sample_count)
        sink = blocks.file_sink(gr.sizeof_gr_complex, str(args.output), False)
        self.connect(source, head, sink)


def parse_args():
    parser = argparse.ArgumentParser(description="Capture M300 RX samples as GNU Radio CF32")
    parser.add_argument("--addr", default="/dev/xdma0")
    parser.add_argument("--sample-rate", type=float, default=61.44e6)
    parser.add_argument("--center-freq", type=float, default=2.4e9)
    parser.add_argument("--gain", type=float, default=20.0)
    parser.add_argument("--seconds", type=float, default=1.0)
    parser.add_argument("--work-samples", type=int, default=4096)
    parser.add_argument("--output", type=pathlib.Path, default=pathlib.Path("m300_rx.cf32"))
    return parser.parse_args()


def main():
    args = parse_args()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    flowgraph = M300RxCapture(args)
    flowgraph.run()
    samples = max(1, round(args.sample_rate * args.seconds))
    print(f"captured {samples} CF32 samples to {args.output}")


if __name__ == "__main__":
    main()
