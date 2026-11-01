#!/usr/bin/env python3
"""Compare a paper_sweep.py data root against the paper's active-perception table.

The command-line entry point of :mod:`open_apero_ros.compare_paper_sweep`.
"""

import sys

from open_apero_ros.compare_paper_sweep import main

if __name__ == "__main__":
    sys.exit(main())
