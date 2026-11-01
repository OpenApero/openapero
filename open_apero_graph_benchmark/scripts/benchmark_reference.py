#!/usr/bin/env python3
"""Write, or check, the reference.json that stands in for a dataset's result files.

The command-line entry point of :mod:`open_apero_graph_benchmark.benchmark_reference`, which
documents the format and the options.
"""

import sys

from open_apero_graph_benchmark.benchmark_reference import main

if __name__ == "__main__":
    sys.exit(main())
