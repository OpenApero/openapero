#!/usr/bin/env python3
"""Compare two graph-benchmark result trees for everything except measured time.

The command-line entry point of :mod:`open_apero_graph_benchmark.compare_result_trees`, which
documents what is compared. Exit status is 0 when the trees agree, 1 otherwise.
"""

import sys

from open_apero_graph_benchmark.compare_result_trees import main

if __name__ == "__main__":
    sys.exit(main())
