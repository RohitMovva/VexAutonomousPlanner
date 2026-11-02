"""Run the path optimizer in its own process, for the GUI.

A run takes tens of seconds of pure-Python search; on a thread it would hold the GIL
and stall the window. A spawned process keeps the window responsive, can be asked to
stop early (the search then returns the best route it has) and can be killed outright.

No Qt here: the child imports only this module and the optimizer. The GUI polls
``OptimizerJob.poll`` from a timer.
"""

from __future__ import annotations

import multiprocessing as mp
import queue
import traceback
from typing import List, Optional, Sequence, Tuple

from motion_profiling_v2 import motion_profile_generator as mpg
from utilities import path_optimizer as po
from utilities import route_serializer as rs

PROGRESS, DONE, ERROR = "progress", "done", "error"


def _run(route_text, action_names, constraints, options, out_queue, stop_event):
    try:
        route = rs.parse_route(route_text, action_names)
        result = po.optimize_route(
            route,
            action_names,
            constraints,
            options,
            progress=lambda message: out_queue.put((PROGRESS, message)),
            should_stop=stop_event.is_set,
        )
        out_queue.put((DONE, result))
    except BaseException as exc:  # reported to the GUI, which shows it
        out_queue.put((ERROR, "".join(traceback.format_exception(exc))))


class OptimizerJob:
    """One optimizer run in a child process."""

    def __init__(
        self,
        route_text: str,
        action_names: Sequence[str],
        constraints: mpg.Constraints,
        options: po.OptimizerOptions,
    ):
        # spawn, not fork: forking a process that has Qt running is unsafe, and spawn
        # is what Windows (and the PyInstaller build) does anyway.
        ctx = mp.get_context("spawn")
        self._queue = ctx.Queue()
        self._stop = ctx.Event()
        self._process = ctx.Process(
            target=_run,
            args=(route_text, list(action_names), constraints, options, self._queue, self._stop),
            daemon=True,
        )
        self.finished = False

    def start(self) -> None:
        self._process.start()

    def request_stop(self) -> None:
        """Finish now with the best route found so far."""
        self._stop.set()

    def kill(self) -> None:
        self.finished = True
        if self._process.is_alive():
            self._process.terminate()
        self._process.join(timeout=1)

    def poll(self) -> List[Tuple[str, object]]:
        """Messages since the last poll: (PROGRESS, text), then one (DONE, result) or
        (ERROR, text) at the end."""
        messages = []
        while True:
            try:
                messages.append(self._queue.get_nowait())
            except queue.Empty:
                break
        if any(kind in (DONE, ERROR) for kind, _ in messages):
            self.finished = True
            self._process.join(timeout=1)
        elif not self.finished and not self._process.is_alive() and self._process.exitcode is not None:
            # Died without reporting; one last drain in case the result raced the exit.
            try:
                messages.append(self._queue.get(timeout=0.5))
            except queue.Empty:
                messages.append((ERROR, f"the optimizer process exited with code {self._process.exitcode}"))
            self.finished = True
        return messages

    @property
    def exit_code(self) -> Optional[int]:
        return self._process.exitcode
