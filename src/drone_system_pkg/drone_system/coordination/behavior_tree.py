# Small behavior-tree helpers for patrol / chase / return mode picking.

from typing import Callable, Iterable


class BTNode:
    def run(self, ctx) -> bool:
        raise NotImplementedError


class Condition(BTNode):
    def __init__(self, fn: Callable):
        self._fn = fn

    def run(self, ctx) -> bool:
        return bool(self._fn(ctx))


class Action(BTNode):
    def __init__(self, fn: Callable):
        self._fn = fn

    def run(self, ctx) -> bool:
        self._fn(ctx)
        return True


class Sequence(BTNode):
    def __init__(self, children: Iterable[BTNode]):
        self._children = list(children)

    def run(self, ctx) -> bool:
        for c in self._children:
            if not c.run(ctx):
                return False
        return True


class Selector(BTNode):
    def __init__(self, children: Iterable[BTNode]):
        self._children = list(children)

    def run(self, ctx) -> bool:
        for c in self._children:
            if c.run(ctx):
                return True
        return False
