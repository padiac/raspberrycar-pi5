"""Where the detector's tunables are declared.

Separate from detector.py only because importing that pulls in the whole
inference stack, and something as cheap as reading a confidence threshold
should not cost a model load. The detector imports this, not the reverse.
"""

from settings import Spec, group

LOOKING = group(
    "vision",
    detect_confidence=Spec(
        0.4, bounds=(0.05, 0.95),
        help="Below this a box is ignored. Lower sees more and invents "
             "more; higher is cleaner and misses more."),
)
