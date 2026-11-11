#!/usr/bin/env python3
"""Run every test suite, and say plainly whether the lot passed.

There is no pytest here on purpose. These are checks a robot can run on
itself with nothing installed, printed in an order that reads as an argument
rather than as a table of dots -- which matters when the thing being checked
is a machine that can drive into a wall.

    python3 tests/run_all.py          # from PythonCode, or from anywhere
"""

import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
SUITES = ["test_layers.py", "test_tree.py", "test_link.py", "test_follow.py",
          "test_housekeeping.py", "test_vfh.py", "test_turning.py", "test_patrol.py", "test_hal.py",
          "test_estimator.py", "test_blob.py", "test_reacquire.py",
          "test_aim.py", "test_telemetry.py",
          "test_names.py", "test_architecture.py"]

failed = []
for name in SUITES:
    print(f"\n{'=' * 62}\n{name}\n{'=' * 62}")
    result = subprocess.run([sys.executable, str(HERE / name)])
    if result.returncode != 0:
        failed.append(name)

print()
if failed:
    print(f"FAILED: {', '.join(failed)}")
    sys.exit(1)
print(f"all {len(SUITES)} suites passed")
