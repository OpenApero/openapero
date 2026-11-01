#!/usr/bin/env python3
"""Plot a graph-benchmark dataset's final-gain ranges per algorithm, objective and replanning mode.

The command-line entry point of :mod:`open_apero_graph_benchmark.plot_final_gain_ranges`.
"""

import sys

from open_apero_graph_benchmark.plot_final_gain_ranges import main

if __name__ == "__main__":
    sys.exit(main())
