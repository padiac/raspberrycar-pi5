#!/usr/bin/env python3
"""Every name used in this codebase resolves to something.

Here because of a specific escape. Moving the telemetry assembly into its
own file left it reading a name that was a global in the file it came from
and nothing at all in the file it went to. Python compiles that happily --
a NameError is a runtime event -- the whole suite passed, the car came up,
and it served five hundreds a second to its own dashboard.

The deeper lesson was that nothing in the suite called telemetry(), and
that is worth fixing properly. This is the cheap half, and it runs in a
tenth of a second over every file.

Run directly: python3 tests/test_names.py
"""

import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
CODE = os.path.dirname(HERE)
CHECKER = os.path.join(os.path.dirname(CODE), "tools", "undefined.py")

result = subprocess.run([sys.executable, CHECKER, CODE],
                        capture_output=True, text=True)
print((result.stdout or result.stderr).strip())
if result.returncode != 0:
    print()
    print("the names above are used but never defined or imported")
    sys.exit(1)
print()
print("every name resolves")
