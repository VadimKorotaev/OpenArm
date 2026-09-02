import threading

from openarm_skills.execution import CancellationToken, SkillExecutor
from openarm_skills.models import Skill


class BarrierRuntime:
    def __init__(self) -> None:
        self.barrier = threading.Barrier(2)
        self.calls: list[tuple[str, str]] = []
        self.lock = threading.Lock()

    def move(self, action, cancellation) -> None:
        cancellation.raise_if_cancelled()
        self.barrier.wait(timeout=1.0)
        with self.lock:
            self.calls.append(("move", action.arm))

    def gripper(self, action, cancellation) -> None:
        cancellation.raise_if_cancelled()
        self.barrier.wait(timeout=1.0)
        with self.lock:
            self.calls.append(("gripper", action.arm))


def world_move(arm: str) -> dict:
    return {
        "type": "move",
        "arm": arm,
        "target": {
            "reference": {"kind": "world", "frame_id": "world"},
            "pose": {
                "position": {"x": 0.2, "y": 0.1, "z": 0.4},
                "orientation": {"w": 1.0},
            },
        },
    }


def test_parallel_branches_execute_concurrently() -> None:
    runtime = BarrierRuntime()
    executor = SkillExecutor(runtime)
    skill = Skill(
        name="parallel runtime test",
        actions=[
            {
                "type": "parallel",
                "branches": [
                    {"actions": [world_move("left")]},
                    {
                        "actions": [
                            {"type": "gripper", "arm": "right", "opening": 0.5}
                        ]
                    },
                ],
            }
        ],
    )
    paths: list[str] = []

    executor.execute(
        skill,
        CancellationToken(threading.Event()),
        lambda path, action: paths.append(path),
    )

    assert set(runtime.calls) == {("move", "left"), ("gripper", "right")}
    assert any("branches[0]" in path for path in paths)
    assert any("branches[1]" in path for path in paths)
