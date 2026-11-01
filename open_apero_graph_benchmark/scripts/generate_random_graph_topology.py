#!/usr/bin/env python3
"""Generate a random graph topology for the graph benchmark.

The command-line entry point of :mod:`open_apero_graph_benchmark.generate_random_graph_topology`.
"""

import sys

from open_apero_graph_benchmark.generate_random_graph_topology import main

if __name__ == "__main__":
    sys.exit(main())
