from uuid import uuid4

import pytest
from pydantic import ValidationError

from openarm_skills.models import (
    GripperAction,
    MoveAction,
    ParallelAction,
    SkillCreate,
    WaitAction,
)


def world_move(arm: str = "left") -> dict:
    return {
        "type": "move",
        "arm": arm,
        "target": {
            "reference": {"kind": "world", "frame_id": "world"},
            "pose": {
                "position": {"x": 0.25, "y": 0.15, "z": 0.46},
                "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
            },
        },
    }


def marker_move(arm: str = "right") -> dict:
    action = world_move(arm)
    action["target"]["reference"] = {
        "kind": "marker",
        "frame_id": "aruco_marker_1",
    }
    return action


def test_complete_skill_parses_all_action_types() -> None:
    skill = SkillCreate.model_validate(
        {
            "name": "Bimanual marker demo",
            "description": "Parallel pose motion followed by a wait.",
            "actions": [
                {
                    "type": "parallel",
                    "branches": [
                        {"actions": [world_move("left")]},
                        {
                            "actions": [
                                marker_move("right"),
                                {"type": "gripper", "arm": "right", "opening": 0.5},
                            ]
                        },
                    ],
                },
                {"type": "wait", "duration": 0.25},
            ],
        }
    )

    assert skill.schema_version == 1
    assert isinstance(skill.actions[0], ParallelAction)
    assert isinstance(skill.actions[0].branches[0].actions[0], MoveAction)
    assert isinstance(skill.actions[0].branches[1].actions[1], GripperAction)
    assert isinstance(skill.actions[1], WaitAction)


def test_json_round_trip_preserves_discriminated_actions() -> None:
    original = SkillCreate(name="round trip", actions=[marker_move()])
    restored = SkillCreate.model_validate_json(original.model_dump_json())

    assert restored == original
    assert restored.actions[0].target.reference.frame_id == "aruco_marker_1"


@pytest.mark.parametrize(
    "reference",
    [
        {"kind": "world", "frame_id": "map"},
        {"kind": "marker", "frame_id": "world"},
        {"kind": "marker", "frame_id": "/leading_slash"},
    ],
)
def test_reference_kind_and_tf_frame_must_match(reference: dict) -> None:
    action = world_move()
    action["target"]["reference"] = reference

    with pytest.raises(ValidationError):
        SkillCreate(name="bad frame", actions=[action])


def test_quaternion_must_be_normalized() -> None:
    action = world_move()
    action["target"]["pose"]["orientation"]["w"] = 2.0

    with pytest.raises(ValidationError, match="quaternion must be normalized"):
        SkillCreate(name="bad quaternion", actions=[action])


def test_parallel_branches_cannot_claim_same_arm() -> None:
    with pytest.raises(ValidationError, match="both claim left arm"):
        SkillCreate(
            name="resource conflict",
            actions=[
                {
                    "type": "parallel",
                    "branches": [
                        {"actions": [world_move("left")]},
                        {
                            "actions": [
                                {"type": "gripper", "arm": "left", "opening": 1.0}
                            ]
                        },
                    ],
                }
            ],
        )


def test_actions_reject_unknown_fields_and_ranges() -> None:
    action = world_move()
    action["velocity_scale"] = 1.1
    action["unexpected"] = True

    with pytest.raises(ValidationError):
        SkillCreate(name="invalid move", actions=[action])

    with pytest.raises(ValidationError):
        SkillCreate(
            name="invalid gripper",
            actions=[{"type": "gripper", "arm": "right", "opening": -0.1}],
        )


def test_action_ids_must_be_unique_across_nested_tree() -> None:
    duplicate = str(uuid4())
    left = world_move("left")
    right = marker_move("right")
    left["action_id"] = duplicate
    right["action_id"] = duplicate

    with pytest.raises(ValidationError, match="action_id values must be unique"):
        SkillCreate(
            name="duplicate IDs",
            actions=[
                {
                    "type": "parallel",
                    "branches": [
                        {"actions": [left]},
                        {"actions": [right]},
                    ],
                }
            ],
        )
