#!/usr/bin/env python3
"""Plot the performance curves of the planners on one experiment scene.

The command-line entry point of :mod:`open_apero_ros.plot_performance_curves`.
"""

import sys

from open_apero_ros.plot_performance_curves import main

if __name__ == "__main__":
    sys.exit(main())
