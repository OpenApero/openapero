#!/usr/bin/env python3
"""Re-run the paper's graph benchmark on the stored graph instances and check it reproduces.

The command-line entry point of :mod:`open_apero_graph_benchmark.verify_benchmark_results`.
"""

import sys

from open_apero_graph_benchmark.verify_benchmark_results import main

if __name__ == "__main__":
    sys.exit(main())
