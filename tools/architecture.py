#!/usr/bin/env python3
"""Check the rules the codebase is supposed to follow.

A convention that is only written down drifts, and the drift is invisible
until somebody tries to change something and finds they cannot. These are
the conventions that can be checked mechanically, so they are.

Three of them:

**Dependencies point one way.** Layers are listed in LAYERS, lowest first,
and a module may import from its own layer or any layer below it, never
above. This is what keeps the hardware layer ignorant of what the robot is
trying to do -- the property that makes a driver swappable at all. The
moment perception imports from behaviors, the two are one thing wearing two
names.

**Imports are used.** An import nobody uses is at best noise and at worst a
lie about what a module depends on, which is exactly what the first rule is
trying to read.

**Devices say what they are.** Anything implementing a hardware contract
carries a label, because the startup log naming the driver that filled each
role is how a swap is confirmed to have happened.

    python3 tools/architecture.py PythonCode
"""

import ast
import os
import sys

#: Lowest first. A module may import its own layer and anything below.
LAYERS = [
    ("settings", "calibration"),        # choices and measurements
    ("diagnostics",),                   # logging; depends on settings only
    ("hal",),                           # pins, buses, the camera
    ("perception",),                    # turning sensors into a world
    ("control",),                       # what is allowed, where to point
    ("behaviors",),                     # what to try
    ("robot",),                         # the loop between them
    ("web",),                           # how a human reaches it
]

#: Anything here is outside the rules: tests exercise every layer by
#: definition, and tools are not part of the running car.
EXEMPT = ("tests", "tools")


def layer_of(module):
    head = module.split(".")[0]
    for index, names in enumerate(LAYERS):
        if head in names:
            return index
    return None


def module_name(path, root):
    relative = os.path.relpath(path, root)
    parts = relative[:-3].split(os.sep)
    if parts[-1] == "__init__":
        parts = parts[:-1]
    return ".".join(parts)


def imports_of(tree):
    """(imported module, line) for every import in the file."""
    found = []
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            for alias in node.names:
                found.append((alias.name, alias.asname or alias.name, node.lineno))
        elif isinstance(node, ast.ImportFrom):
            if node.level:
                continue            # relative: same package, same layer
            for alias in node.names:
                found.append((node.module or "", alias.asname or alias.name,
                              node.lineno))
    return found


def used_names(tree):
    names = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Name):
            names.add(node.id)
        elif isinstance(node, ast.Attribute):
            target = node
            while isinstance(target, ast.Attribute):
                target = target.value
            if isinstance(target, ast.Name):
                names.add(target.id)
    # A name mentioned in a docstring or a string annotation is still a use
    # as far as this check cares; being generous here costs nothing and a
    # false positive costs somebody an afternoon.
    for node in ast.walk(tree):
        if isinstance(node, ast.Constant) and isinstance(node.value, str):
            for word in node.value.replace("(", " ").replace(".", " ").split():
                names.add(word.strip(",:'\"[]{}"))
    return names


def check_file(path, root, problems):
    with open(path, encoding="utf-8") as handle:
        text = handle.read()
    tree = ast.parse(text, path)
    name = module_name(path, root)
    if name.split(".")[0] in EXEMPT:
        return

    mine = layer_of(name)
    used = used_names(tree)
    lines = text.splitlines()

    for imported, bound, line in imports_of(tree):
        theirs = layer_of(imported)
        if mine is not None and theirs is not None and theirs > mine:
            problems.append(
                "%s:%d  %s imports %s -- a layer above it"
                % (path, line, ".".join(LAYERS[mine]), imported))
        root_name = bound.split(".")[0]
        if root_name not in used and "noqa" not in lines[line - 1]:
            problems.append("%s:%d  imports %s and never uses it"
                            % (path, line, bound))


def check_labels(root, problems):
    """Every device contract implementation carries a label."""
    contracts = {"RangeSensor", "ProximitySensor", "FloorSensor", "PanMount",
                 "PowerMonitor", "FrameSource"}
    for folder, _dirs, files in os.walk(os.path.join(root, "hal")):
        if "__pycache__" in folder:
            continue
        for filename in sorted(files):
            if not filename.endswith(".py"):
                continue
            path = os.path.join(folder, filename)
            with open(path, encoding="utf-8") as handle:
                tree = ast.parse(handle.read(), path)
            for node in ast.walk(tree):
                if not isinstance(node, ast.ClassDef):
                    continue
                bases = {b.id for b in node.bases if isinstance(b, ast.Name)}
                if not bases & contracts:
                    continue
                labelled = any(
                    isinstance(item, ast.Assign)
                    and any(getattr(t, "id", None) == "label" for t in item.targets)
                    for item in node.body)
                if not labelled:
                    problems.append(
                        "%s:%d  %s implements a device contract with no label"
                        % (path, node.lineno, node.name))


def main(roots):
    problems = []
    for root in roots:
        for folder, _dirs, files in os.walk(root):
            if "__pycache__" in folder:
                continue
            for filename in sorted(files):
                if filename.endswith(".py"):
                    check_file(os.path.join(folder, filename), root, problems)
        check_labels(root, problems)

    for problem in problems:
        print(problem)
    print("\n%d problem(s)" % len(problems))
    return 1 if problems else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:] or ["."]))
