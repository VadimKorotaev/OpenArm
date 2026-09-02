"""Versioned domain model for persisted OpenArm skills.

A move target is stored as a transform from ``reference.frame_id`` to the end
effector. For a marker-relative target, ``pose`` therefore represents the
saved ``T_marker_ee``. The runtime will resolve it to ``T_world_target`` using
the marker's current TF immediately before execution.
"""

from __future__ import annotations

import math
from datetime import datetime, timezone
from typing import Annotated, Literal, Union
from uuid import UUID, uuid4

from pydantic import BaseModel, ConfigDict, Field, StringConstraints
from pydantic import field_validator, model_validator


Arm = Literal["left", "right"]
SkillName = Annotated[
    str, StringConstraints(strip_whitespace=True, min_length=1, max_length=100)
]
Description = Annotated[str, StringConstraints(strip_whitespace=True, max_length=2000)]
FrameId = Annotated[
    str,
    StringConstraints(
        strip_whitespace=True,
        min_length=1,
        max_length=200,
        pattern=r"^[A-Za-z][A-Za-z0-9_/]*$",
    ),
]


def utc_now() -> datetime:
    return datetime.now(timezone.utc)


class StrictModel(BaseModel):
    model_config = ConfigDict(extra="forbid", allow_inf_nan=False)


class Vector3(StrictModel):
    x: float
    y: float
    z: float


class Quaternion(StrictModel):
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    w: float = 1.0

    @model_validator(mode="after")
    def require_unit_quaternion(self) -> "Quaternion":
        norm = math.sqrt(self.x**2 + self.y**2 + self.z**2 + self.w**2)
        if norm < 1e-12:
            raise ValueError("quaternion must not be zero")
        if abs(norm - 1.0) > 1e-3:
            raise ValueError(f"quaternion must be normalized; norm is {norm:.6f}")
        # Remove harmless floating-point drift while preserving strict validation.
        self.x /= norm
        self.y /= norm
        self.z /= norm
        self.w /= norm
        return self


class CartesianPose(StrictModel):
    position: Vector3
    orientation: Quaternion = Field(default_factory=Quaternion)


class PoseReference(StrictModel):
    kind: Literal["world", "marker"]
    frame_id: FrameId

    @model_validator(mode="after")
    def frame_matches_kind(self) -> "PoseReference":
        if self.kind == "world" and self.frame_id != "world":
            raise ValueError("a world reference must use frame_id='world'")
        if self.kind == "marker" and self.frame_id == "world":
            raise ValueError("a marker reference must name a non-world TF frame")
        return self


class PoseTarget(StrictModel):
    reference: PoseReference
    pose: CartesianPose


class MarkerState(StrictModel):
    frame_id: FrameId
    pose: CartesianPose


class CapturePoseRequest(StrictModel):
    reference: PoseReference = Field(
        default_factory=lambda: PoseReference(kind="world", frame_id="world")
    )
    velocity_scale: float = Field(default=0.10, gt=0.0, le=1.0)
    acceleration_scale: float = Field(default=0.10, gt=0.0, le=1.0)
    planning_time: float = Field(default=5.0, ge=0.1, le=60.0)
    position_tolerance: float = Field(default=0.002, gt=0.0, le=0.05)
    orientation_tolerance: float = Field(default=0.01, gt=0.0, le=0.5)


class NudgeRequest(StrictModel):
    """Small world-frame Cartesian offset used for manual positioning."""

    translation: Vector3 = Field(
        default_factory=lambda: Vector3(x=0.0, y=0.0, z=0.0)
    )
    rotation_rpy: Vector3 = Field(
        default_factory=lambda: Vector3(x=0.0, y=0.0, z=0.0)
    )
    velocity_scale: float = Field(default=0.08, gt=0.0, le=1.0)
    acceleration_scale: float = Field(default=0.08, gt=0.0, le=1.0)
    planning_time: float = Field(default=5.0, ge=0.1, le=60.0)

    @model_validator(mode="after")
    def require_small_nonzero_offset(self) -> "NudgeRequest":
        values = (
            self.translation.x,
            self.translation.y,
            self.translation.z,
            self.rotation_rpy.x,
            self.rotation_rpy.y,
            self.rotation_rpy.z,
        )
        if all(abs(value) < 1e-12 for value in values):
            raise ValueError("at least one nudge component must be non-zero")
        if max(abs(value) for value in values[:3]) > 0.05:
            raise ValueError("translation nudge is limited to 0.05 m per request")
        if max(abs(value) for value in values[3:]) > math.radians(15.0):
            raise ValueError("rotation nudge is limited to 15 degrees per request")
        return self


class ActionBase(StrictModel):
    action_id: UUID = Field(default_factory=uuid4)


class MoveAction(ActionBase):
    type: Literal["move"] = "move"
    arm: Arm
    target: PoseTarget
    velocity_scale: float = Field(default=0.10, gt=0.0, le=1.0)
    acceleration_scale: float = Field(default=0.10, gt=0.0, le=1.0)
    planning_time: float = Field(default=5.0, ge=0.1, le=60.0)
    position_tolerance: float = Field(default=0.002, gt=0.0, le=0.05)
    orientation_tolerance: float = Field(default=0.01, gt=0.0, le=0.5)


class GripperAction(ActionBase):
    type: Literal["gripper"] = "gripper"
    arm: Arm
    # Normalized abstraction: 0.0 is closed, 1.0 is fully open. The ROS
    # adapter will map this value to the configured finger joint limits.
    opening: float = Field(ge=0.0, le=1.0)
    max_effort: float = Field(default=0.0, ge=0.0)
    timeout: float = Field(default=5.0, gt=0.0, le=60.0)


class WaitAction(ActionBase):
    type: Literal["wait"] = "wait"
    duration: float = Field(gt=0.0, le=3600.0)


class ActionSequence(StrictModel):
    actions: list["SkillAction"] = Field(min_length=1, max_length=100)


class ParallelAction(ActionBase):
    type: Literal["parallel"] = "parallel"
    branches: list[ActionSequence] = Field(min_length=2, max_length=8)

    @model_validator(mode="after")
    def branches_must_not_compete_for_an_arm(self) -> "ParallelAction":
        claimed: dict[str, int] = {}
        for index, branch in enumerate(self.branches, start=1):
            branch_arms: set[str] = set()
            for action in branch.actions:
                branch_arms.update(claimed_arms(action))
            for arm in branch_arms:
                previous = claimed.get(arm)
                if previous is not None:
                    raise ValueError(
                        f"parallel branches {previous} and {index} both claim {arm} arm"
                    )
                claimed[arm] = index
        return self


SkillAction = Annotated[
    Union[MoveAction, GripperAction, WaitAction, ParallelAction],
    Field(discriminator="type"),
]

ActionSequence.model_rebuild(_types_namespace={"SkillAction": SkillAction})
ParallelAction.model_rebuild(_types_namespace={"SkillAction": SkillAction})


def claimed_arms(action: SkillAction) -> set[str]:
    if isinstance(action, (MoveAction, GripperAction)):
        return {action.arm}
    if isinstance(action, WaitAction):
        return set()
    arms: set[str] = set()
    for branch in action.branches:
        for child in branch.actions:
            arms.update(claimed_arms(child))
    return arms


def iter_actions(actions: list[SkillAction]):
    for action in actions:
        yield action
        if isinstance(action, ParallelAction):
            for branch in action.branches:
                yield from iter_actions(branch.actions)


def action_depth(action: SkillAction) -> int:
    if not isinstance(action, ParallelAction):
        return 1
    return 1 + max(
        action_depth(child)
        for branch in action.branches
        for child in branch.actions
    )


class SkillCreate(StrictModel):
    schema_version: Literal[1] = 1
    name: SkillName
    description: Description = ""
    actions: list[SkillAction] = Field(min_length=1, max_length=100)

    @model_validator(mode="after")
    def validate_action_tree(self) -> "SkillCreate":
        flattened = list(iter_actions(self.actions))
        ids = [action.action_id for action in flattened]
        if len(ids) != len(set(ids)):
            raise ValueError("action_id values must be unique within a skill")
        if max(action_depth(action) for action in self.actions) > 5:
            raise ValueError("parallel action nesting is limited to five levels")
        return self


class Skill(SkillCreate):
    id: UUID = Field(default_factory=uuid4)
    created_at: datetime = Field(default_factory=utc_now)
    updated_at: datetime = Field(default_factory=utc_now)

    @field_validator("created_at", "updated_at")
    @classmethod
    def timestamps_must_have_timezone(cls, value: datetime) -> datetime:
        if value.tzinfo is None or value.utcoffset() is None:
            raise ValueError("timestamps must include a timezone")
        return value

    @model_validator(mode="after")
    def updated_not_before_created(self) -> "Skill":
        if self.updated_at < self.created_at:
            raise ValueError("updated_at must not be earlier than created_at")
        return self


class SkillUpdate(StrictModel):
    name: SkillName | None = None
    description: Description | None = None
    actions: list[SkillAction] | None = Field(default=None, min_length=1, max_length=100)

    @model_validator(mode="after")
    def require_at_least_one_change(self) -> "SkillUpdate":
        if not self.model_fields_set:
            raise ValueError("at least one field must be supplied")
        null_fields = [
            field for field in self.model_fields_set if getattr(self, field) is None
        ]
        if null_fields:
            raise ValueError(
                "update fields cannot be null: " + ", ".join(sorted(null_fields))
            )
        if self.actions is not None:
            probe = SkillCreate(name="validation probe", actions=self.actions)
            self.actions = probe.actions
        return self
