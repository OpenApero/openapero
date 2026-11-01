#!/usr/bin/env python3
"""Write the CPU-time performance overview for recorded experiment runs.

The command-line entry point of :mod:`open_apero_ros.evaluate_experiment`.
"""

import sys

from open_apero_ros.evaluate_experiment import main

if __name__ == "__main__":
    sys.exit(main())
