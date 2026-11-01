#!/usr/bin/env python3
"""Compare normalized planner gain across active-perception tasks.

The command-line entry point of :mod:`open_apero_ros.plot_normalized_gain`.
"""

import sys

from open_apero_ros.plot_normalized_gain import main

if __name__ == "__main__":
    sys.exit(main())
