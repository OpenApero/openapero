#!/usr/bin/env python3
"""Render a graph-benchmark graph instance as a gain-colored PDF.

The command-line entry point of :mod:`open_apero_graph_benchmark.render_graph`.
"""

import sys

from open_apero_graph_benchmark.render_graph import main

if __name__ == "__main__":
    sys.exit(main())
