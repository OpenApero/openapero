#!/usr/bin/env python3
"""Generate and visualize random collectible points for PointsEvaluator.

The command-line entry point of :mod:`open_apero_ros.generate_points`.
"""

import sys

from open_apero_ros.generate_points import main

if __name__ == "__main__":
    sys.exit(main())
