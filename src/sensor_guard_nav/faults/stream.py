"""Deterministic stateful faults for arbitrary frame streams."""

from __future__ import annotations

from collections import deque
from copy import deepcopy
from dataclasses import dataclass, field
from typing import Generic, TypeVar

T = TypeVar("T")


@dataclass(slots=True)
class FrameFreeze(Generic[T]):
    """Repeat the first frame received while enabled."""

    enabled: bool = False
    _frozen: T | None = field(default=None, init=False, repr=False)

    def apply(self, frame: T) -> T:
        if not self.enabled:
            self._frozen = None
            return deepcopy(frame)
        if self._frozen is None:
            self._frozen = deepcopy(frame)
        return deepcopy(self._frozen)

    def reset(self) -> None:
        self._frozen = None


class FrameDelay(Generic[T]):
    """Return a frame from exactly ``delay_frames`` calls ago.

    During warm-up, the oldest available frame is returned.
    """

    def __init__(self, delay_frames: int) -> None:
        if delay_frames < 0:
            raise ValueError("delay_frames must be non-negative")
        self.delay_frames = delay_frames
        self._queue: deque[T] = deque(maxlen=delay_frames + 1)

    def apply(self, frame: T) -> T:
        self._queue.append(deepcopy(frame))
        if len(self._queue) <= self.delay_frames:
            return deepcopy(self._queue[0])
        return deepcopy(self._queue.popleft())

    def reset(self) -> None:
        self._queue.clear()
