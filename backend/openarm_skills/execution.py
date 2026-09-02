"""Thread-safe Skill execution orchestration independent of ROS imports."""

from __future__ import annotations

import threading
import time
from concurrent.futures import ThreadPoolExecutor, as_completed
from datetime import datetime
from typing import Callable, Literal, Protocol
from uuid import UUID, uuid4

from pydantic import Field

from .models import (
    GripperAction,
    MoveAction,
    ParallelAction,
    Skill,
    StrictModel,
    WaitAction,
    utc_now,
)
from .repository import SkillRepository


class ExecutionCancelledError(RuntimeError):
    pass


class CancellationToken:
    def __init__(self, *events: threading.Event) -> None:
        self._events = events

    def is_set(self) -> bool:
        return any(event.is_set() for event in self._events)

    def wait(self, timeout: float) -> bool:
        deadline = time.monotonic() + timeout
        while not self.is_set():
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                return False
            self._events[0].wait(min(remaining, 0.05))
        return True

    def raise_if_cancelled(self) -> None:
        if self.is_set():
            raise ExecutionCancelledError("execution cancelled")

    def combined_with(self, *events: threading.Event) -> "CancellationToken":
        return CancellationToken(*self._events, *events)


class MotionRuntime(Protocol):
    def move(self, action: MoveAction, cancellation: CancellationToken) -> None: ...

    def gripper(
        self, action: GripperAction, cancellation: CancellationToken
    ) -> None: ...


ExecutionStatus = Literal[
    "queued", "running", "succeeded", "failed", "cancelled"
]


class SkillExecution(StrictModel):
    id: UUID = Field(default_factory=uuid4)
    skill_id: UUID
    skill_name: str
    status: ExecutionStatus = "queued"
    current_path: str | None = None
    current_action_id: UUID | None = None
    current_action_type: str | None = None
    error: str | None = None
    created_at: datetime = Field(default_factory=utc_now)
    started_at: datetime | None = None
    finished_at: datetime | None = None


StepCallback = Callable[[str, MoveAction | GripperAction | WaitAction | ParallelAction], None]


class SkillExecutor:
    def __init__(self, runtime: MotionRuntime) -> None:
        self.runtime = runtime
        self._arm_locks = {"left": threading.Lock(), "right": threading.Lock()}

    def execute(
        self,
        skill: Skill,
        cancellation: CancellationToken,
        on_step: StepCallback,
    ) -> None:
        self._execute_actions(skill.actions, "actions", cancellation, on_step)

    def _execute_actions(
        self,
        actions,
        path: str,
        cancellation: CancellationToken,
        on_step: StepCallback,
    ) -> None:
        for index, action in enumerate(actions):
            cancellation.raise_if_cancelled()
            action_path = f"{path}[{index}]"
            on_step(action_path, action)
            if isinstance(action, MoveAction):
                self._with_arm_lock(
                    action.arm,
                    cancellation,
                    lambda: self.runtime.move(action, cancellation),
                )
            elif isinstance(action, GripperAction):
                self._with_arm_lock(
                    action.arm,
                    cancellation,
                    lambda: self.runtime.gripper(action, cancellation),
                )
            elif isinstance(action, WaitAction):
                if cancellation.wait(action.duration):
                    raise ExecutionCancelledError("execution cancelled during wait")
            elif isinstance(action, ParallelAction):
                self._execute_parallel(action, action_path, cancellation, on_step)
            else:
                raise RuntimeError(f"unsupported action type: {type(action).__name__}")

    def _with_arm_lock(
        self,
        arm: str,
        cancellation: CancellationToken,
        callback: Callable[[], None],
    ) -> None:
        lock = self._arm_locks[arm]
        while not lock.acquire(timeout=0.05):
            cancellation.raise_if_cancelled()
        try:
            cancellation.raise_if_cancelled()
            callback()
        finally:
            lock.release()

    def _execute_parallel(
        self,
        action: ParallelAction,
        path: str,
        cancellation: CancellationToken,
        on_step: StepCallback,
    ) -> None:
        sibling_stop = threading.Event()
        branch_token = cancellation.combined_with(sibling_stop)

        def run_branch(index: int) -> None:
            branch = action.branches[index]
            self._execute_actions(
                branch.actions,
                f"{path}.branches[{index}].actions",
                branch_token,
                on_step,
            )

        first_error: Exception | None = None
        with ThreadPoolExecutor(max_workers=len(action.branches)) as pool:
            futures = [pool.submit(run_branch, index) for index in range(len(action.branches))]
            for future in as_completed(futures):
                try:
                    future.result()
                except Exception as exc:
                    if first_error is None:
                        first_error = exc
                    sibling_stop.set()
        if first_error is not None:
            raise first_error


class ExecutionManager:
    def __init__(self, repository: SkillRepository, runtime: MotionRuntime) -> None:
        self.repository = repository
        self.executor = SkillExecutor(runtime)
        self._records: dict[UUID, SkillExecution] = {}
        self._cancel_events: dict[UUID, threading.Event] = {}
        self._threads: dict[UUID, threading.Thread] = {}
        self._lock = threading.Lock()

    def start(self, skill_id: UUID) -> SkillExecution:
        skill = self.repository.get(skill_id)
        return self.start_skill(skill)

    def start_skill(self, skill: Skill) -> SkillExecution:
        """Start a validated Skill, including an intentionally transient one."""

        record = SkillExecution(skill_id=skill.id, skill_name=skill.name)
        cancel_event = threading.Event()
        worker = threading.Thread(
            target=self._run,
            args=(record.id, skill, cancel_event),
            name=f"skill-execution-{record.id}",
            daemon=True,
        )
        with self._lock:
            self._records[record.id] = record
            self._cancel_events[record.id] = cancel_event
            self._threads[record.id] = worker
        worker.start()
        return self.get(record.id)

    def _run(
        self, execution_id: UUID, skill: Skill, cancel_event: threading.Event
    ) -> None:
        self._update(
            execution_id,
            status="running",
            started_at=utc_now(),
        )
        token = CancellationToken(cancel_event)
        try:
            self.executor.execute(
                skill,
                token,
                lambda path, action: self._update(
                    execution_id,
                    current_path=path,
                    current_action_id=action.action_id,
                    current_action_type=action.type,
                ),
            )
            token.raise_if_cancelled()
        except ExecutionCancelledError as exc:
            self._update(
                execution_id,
                status="cancelled",
                error=str(exc),
                finished_at=utc_now(),
            )
        except Exception as exc:
            self._update(
                execution_id,
                status="failed",
                error=str(exc),
                finished_at=utc_now(),
            )
        else:
            self._update(
                execution_id,
                status="succeeded",
                finished_at=utc_now(),
            )

    def _update(self, execution_id: UUID, **changes) -> None:
        with self._lock:
            current = self._records[execution_id]
            self._records[execution_id] = current.model_copy(update=changes)

    def get(self, execution_id: UUID) -> SkillExecution:
        with self._lock:
            record = self._records.get(execution_id)
            if record is None:
                raise KeyError(str(execution_id))
            return record.model_copy(deep=True)

    def list(self) -> list[SkillExecution]:
        with self._lock:
            records = list(self._records.values())
        return sorted(
            (record.model_copy(deep=True) for record in records),
            key=lambda record: record.created_at,
            reverse=True,
        )

    def cancel(self, execution_id: UUID) -> SkillExecution:
        with self._lock:
            record = self._records.get(execution_id)
            event = self._cancel_events.get(execution_id)
            if record is None or event is None:
                raise KeyError(str(execution_id))
            if record.status in {"succeeded", "failed", "cancelled"}:
                return record.model_copy(deep=True)
            event.set()
            return record.model_copy(deep=True)

    def shutdown(self) -> None:
        with self._lock:
            events = list(self._cancel_events.values())
            threads = list(self._threads.values())
        for event in events:
            event.set()
        for thread in threads:
            thread.join(timeout=3.0)
