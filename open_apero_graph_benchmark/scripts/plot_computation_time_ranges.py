#!/usr/bin/env python3
"""Plot an a-priori graph-benchmark dataset's computation-time ranges on a log scale.

The command-line entry point of :mod:`open_apero_graph_benchmark.plot_computation_time_ranges`.
"""

import sys

from open_apero_graph_benchmark.plot_computation_time_ranges import main

if __name__ == "__main__":
    sys.exit(main())
