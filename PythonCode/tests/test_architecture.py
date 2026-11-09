#!/usr/bin/env python3
"""The conventions in CONVENTIONS.md, enforced rather than described.

A convention that is only written down drifts, and the drift is invisible
until somebody tries to change something and finds they cannot. Everything
in here is a rule that can be checked mechanically, so it is checked on
every run instead of being hoped for.

Run directly: python3 tests/test_architecture.py
"""

import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
CODE = os.path.dirname(HERE)
CHECKER = os.path.join(os.path.dirname(CODE), "tools", "architecture.py")

result = subprocess.run([sys.executable, CHECKER, CODE],
                        capture_output=True, text=True)
print((result.stdout or result.stderr).strip())
if result.returncode != 0:
    print()
    print("Each line above breaks one of three rules:")
    print("  - dependencies point one way, lowest layer first")
    print("  - an import that is never used is a lie about what a module needs")
    print("  - a device contract implementation says which hardware it is")
    sys.exit(1)
print()
print("the architecture rules hold")
