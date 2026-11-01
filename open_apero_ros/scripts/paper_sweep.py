#!/usr/bin/env python3
"""Run the active-perception paper sweep: scenes x planners x tasks x trials.

The command-line entry point of :mod:`open_apero_ros.paper_sweep`.
"""

import sys

from open_apero_ros.paper_sweep import main

if __name__ == "__main__":
    sys.exit(main())
