#!/usr/bin/env python3
"""Find names a module uses but never defines or imports.

A standing hole in this project's checks, found the hard way: moving the
telemetry assembly into its own file left it reading DRIVETRAINS, which
was a global in the file it came from and is nothing at all in the file it
went to. It compiled -- a NameError is a runtime event -- the whole test
suite passed, and the car came up and served five hundreds a second.

The suite passed because nothing in it calls telemetry(), which is the
real lesson, but the cheap half of the fix is to stop a name that cannot
resolve from ever reaching the Pi. This is the cheap half.

Deliberately simple and a little pessimistic: it does not understand
comprehension scopes in detail, star imports, or names injected at
runtime. It reports what it cannot account for, and anything it is wrong
about is listed in ALLOWED below with a reason.

    python3 tools/undefined.py PythonCode
"""

import ast
import builtins
import os
import sys

#: Names that are legitimately not defined where they are used.
ALLOWED = {
    "__file__", "__name__", "__doc__", "__all__", "__init__",
}


class Scope:
    def __init__(self, parent=None):
        self.parent = parent
        self.names = set()

    def add(self, name):
        self.names.add(name)

    def known(self, name):
        if name in self.names:
            return True
        return self.parent.known(name) if self.parent else False


class Checker(ast.NodeVisitor):
    def __init__(self, path):
        self.path = path
        self.module = Scope()
        self.scope = self.module
        self.problems = []
        # Two passes over a module: everything defined at module level is
        # visible to every function in it, whatever order they appear in.
        self.deferred = []

    def run(self, tree):
        for node in tree.body:
            self._declare(node)
        self.visit(tree)
        for func, scope in self.deferred:
            self.scope = scope
            for node in func.body:
                self._declare(node)
            for node in func.body:
                self.visit(node)
        return self.problems

    def _declare(self, node):
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef)):
            self.scope.add(node.name)
        elif isinstance(node, (ast.Import, ast.ImportFrom)):
            for alias in node.names:
                self.scope.add((alias.asname or alias.name).split(".")[0])
        elif isinstance(node, ast.Assign):
            for target in node.targets:
                self._bind(target)
        elif isinstance(node, (ast.AnnAssign, ast.AugAssign)):
            self._bind(node.target)
        elif isinstance(node, (ast.For, ast.AsyncFor)):
            self._bind(node.target)
            for child in node.body + node.orelse:
                self._declare(child)
        elif isinstance(node, (ast.If, ast.While)):
            for child in node.body + node.orelse:
                self._declare(child)
        elif isinstance(node, (ast.With, ast.AsyncWith)):
            for item in node.items:
                if item.optional_vars is not None:
                    self._bind(item.optional_vars)
            for child in node.body:
                self._declare(child)
        elif isinstance(node, ast.Try):
            for handler in node.handlers:
                if handler.name:
                    self.scope.add(handler.name)
                for child in handler.body:
                    self._declare(child)
            for child in node.body + node.orelse + node.finalbody:
                self._declare(child)

    def _bind(self, target):
        if isinstance(target, ast.Name):
            self.scope.add(target.id)
        elif isinstance(target, (ast.Tuple, ast.List)):
            for element in target.elts:
                self._bind(element)

    def visit_FunctionDef(self, node):
        inner = Scope(self.scope)
        for arg in (node.args.posonlyargs + node.args.args
                    + node.args.kwonlyargs):
            inner.add(arg.arg)
        if node.args.vararg:
            inner.add(node.args.vararg.arg)
        if node.args.kwarg:
            inner.add(node.args.kwarg.arg)
        for default in node.args.defaults + [d for d in node.args.kw_defaults if d]:
            self.visit(default)
        for decorator in node.decorator_list:
            self.visit(decorator)
        self.deferred.append((node, inner))

    visit_AsyncFunctionDef = visit_FunctionDef

    def visit_ClassDef(self, node):
        for decorator in node.decorator_list:
            self.visit(decorator)
        for base in node.bases:
            self.visit(base)
        inner = Scope(self.scope)
        outer, self.scope = self.scope, inner
        for child in node.body:
            self._declare(child)
        for child in node.body:
            self.visit(child)
        self.scope = outer

    def visit_Lambda(self, node):
        inner = Scope(self.scope)
        for arg in node.args.args:
            inner.add(arg.arg)
        outer, self.scope = self.scope, inner
        self.visit(node.body)
        self.scope = outer

    def _comprehension(self, node):
        inner = Scope(self.scope)
        outer, self.scope = self.scope, inner
        for generator in node.generators:
            self.visit(generator.iter)
            self._bind(generator.target)
            for condition in generator.ifs:
                self.visit(condition)
        for field in ("elt", "key", "value"):
            child = getattr(node, field, None)
            if child is not None:
                self.visit(child)
        self.scope = outer

    visit_ListComp = _comprehension
    visit_SetComp = _comprehension
    visit_GeneratorExp = _comprehension
    visit_DictComp = _comprehension

    def visit_Name(self, node):
        if not isinstance(node.ctx, ast.Load):
            self._bind(node)
            return
        name = node.id
        if name in ALLOWED or hasattr(builtins, name):
            return
        if not self.scope.known(name):
            self.problems.append((self.path, node.lineno, name))


def check(path):
    with open(path, encoding="utf-8") as handle:
        tree = ast.parse(handle.read(), path)
    return Checker(path).run(tree)


def main(roots):
    problems = []
    for root in roots:
        for folder, _dirs, files in os.walk(root):
            if "__pycache__" in folder or "/venv" in folder:
                continue
            for name in sorted(files):
                if name.endswith(".py"):
                    problems += check(os.path.join(folder, name))
    seen = set()
    for path, line, name in problems:
        key = (path, name)
        if key in seen:
            continue
        seen.add(key)
        print("%s:%d  undefined name %r" % (path, line, name))
    print("\n%d undefined name(s)" % len(seen))
    return 1 if seen else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:] or ["."]))
