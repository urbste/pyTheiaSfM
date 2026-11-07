"""Allow ``python -m pytheia`` to run the command line interface."""

import sys

from pytheia.cli import main

if __name__ == "__main__":
    sys.exit(main())
