#!/usr/bin/env python3
"""Generate a clustered spatial gain model for the graph benchmark.

The command-line entry point of :mod:`open_apero_graph_benchmark.generate_clustered_gain_model`.
"""

import sys

from open_apero_graph_benchmark.generate_clustered_gain_model import main

if __name__ == "__main__":
    sys.exit(main())
