"""FastAPI application exposing persistent Skill CRUD operations."""

from __future__ import annotations

import os
from contextlib import asynccontextmanager
from pathlib import Path
from typing import Callable, Literal
from uuid import UUID

from fastapi import FastAPI, HTTPException, Query, Response, status
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles
from pydantic import ValidationError

from .execution import ExecutionManager, MotionRuntime, SkillExecution
from .models import (
    CapturePoseRequest,
    CartesianPose,
    MarkerState,
    MoveAction,
    NudgeRequest,
    PoseReference,
    PoseTarget,
    Skill,
    SkillCreate,
    SkillUpdate,
    Vector3,
)
from .repository import (
    MarkerNotFoundError,
    MarkerRepository,
    SkillNotFoundError,
    SkillRepository,
)
from .transforms import quaternion_from_rpy, quaternion_multiply


BACKEND_DIRECTORY = Path(__file__).resolve().parent.parent
PROJECT_ROOT = BACKEND_DIRECTORY.parent
DEFAULT_DATABASE = PROJECT_ROOT / "data" / "openarm_skills.db"
UI_DIRECTORY = BACKEND_DIRECTORY / "static"


def create_app(
    database_path: str | Path | None = None,
    *,
    ros_enabled: bool | None = None,
    runtime_factory: Callable[[], MotionRuntime] | None = None,
) -> FastAPI:
    selected_path = Path(
        database_path or os.environ.get("OPENARM_SKILLS_DB", DEFAULT_DATABASE)
    )
    repository = SkillRepository(selected_path)
    marker_repository = MarkerRepository(selected_path)
    if ros_enabled is None:
        ros_enabled = os.environ.get("OPENARM_ROS_ENABLED", "1") not in {
            "0",
            "false",
            "False",
        }
    runtime: MotionRuntime | None = None
    execution_manager: ExecutionManager | None = None

    @asynccontextmanager
    async def lifespan(application: FastAPI):
        nonlocal runtime, execution_manager
        repository.initialize()
        application.state.repository = repository
        if runtime_factory is not None:
            runtime = runtime_factory()
        elif ros_enabled:
            # Import lazily so CRUD/unit tests do not require a sourced ROS
            # environment and production fails clearly during startup instead.
            from .ros_runtime import RosMotionRuntime

            runtime = RosMotionRuntime()
        if runtime is not None:
            start = getattr(runtime, "start", None)
            if start is not None:
                start()
            # Rebroadcast saved marker TFs before serving requests, so a Skill
            # saved against a marker still resolves after an API restart.
            set_marker = getattr(runtime, "set_marker", None)
            if set_marker is not None:
                for marker in marker_repository.list():
                    set_marker(marker.frame_id, marker.pose)
            execution_manager = ExecutionManager(repository, runtime)
        application.state.runtime = runtime
        application.state.marker_repository = marker_repository
        application.state.execution_manager = execution_manager
        try:
            yield
        finally:
            if execution_manager is not None:
                execution_manager.shutdown()
            if runtime is not None:
                stop = getattr(runtime, "stop", None)
                if stop is not None:
                    stop()

    application = FastAPI(
        title="OpenArm Skills Constructor API",
        version="0.1.0",
        lifespan=lifespan,
    )

    @application.get("/health")
    def health() -> dict[str, object]:
        return {
            "status": "ok",
            "database": str(repository.database_path),
            "skills": repository.count(),
            "ros_enabled": runtime is not None,
            "ros_ready": bool(runtime is not None and getattr(runtime, "ready", True)),
        }

    def require_execution_manager() -> ExecutionManager:
        if execution_manager is None:
            raise HTTPException(status_code=503, detail="ROS runtime is disabled")
        return execution_manager

    def require_runtime_method(name: str):
        if runtime is None or not hasattr(runtime, name):
            raise HTTPException(
                status_code=503, detail=f"ROS runtime method {name} is unavailable"
            )
        return getattr(runtime, name)

    @application.post(
        "/api/skills",
        response_model=Skill,
        status_code=status.HTTP_201_CREATED,
    )
    def create_skill(draft: SkillCreate) -> Skill:
        return repository.create(draft)

    @application.get("/api/skills", response_model=list[Skill])
    def list_skills(
        limit: int = Query(default=100, ge=1, le=500),
        offset: int = Query(default=0, ge=0),
    ) -> list[Skill]:
        return repository.list(limit=limit, offset=offset)

    @application.get("/api/skills/{skill_id}", response_model=Skill)
    def get_skill(skill_id: UUID) -> Skill:
        try:
            return repository.get(skill_id)
        except SkillNotFoundError as exc:
            raise HTTPException(status_code=404, detail="skill not found") from exc

    @application.patch("/api/skills/{skill_id}", response_model=Skill)
    def update_skill(skill_id: UUID, patch: SkillUpdate) -> Skill:
        try:
            return repository.update(skill_id, patch)
        except SkillNotFoundError as exc:
            raise HTTPException(status_code=404, detail="skill not found") from exc

    @application.delete(
        "/api/skills/{skill_id}", status_code=status.HTTP_204_NO_CONTENT
    )
    def delete_skill(skill_id: UUID) -> Response:
        try:
            repository.delete(skill_id)
        except SkillNotFoundError as exc:
            raise HTTPException(status_code=404, detail="skill not found") from exc
        return Response(status_code=status.HTTP_204_NO_CONTENT)

    @application.post(
        "/api/skills/{skill_id}/actions/capture-pose",
        response_model=Skill,
    )
    def capture_pose_action(
        skill_id: UUID,
        request: CapturePoseRequest,
        arm: Literal["left", "right"] = Query(),
    ) -> Skill:
        try:
            skill = repository.get(skill_id)
        except SkillNotFoundError as exc:
            raise HTTPException(status_code=404, detail="skill not found") from exc
        try:
            pose = require_runtime_method("current_pose")(arm, request.reference)
        except HTTPException:
            raise
        except Exception as exc:
            raise HTTPException(status_code=409, detail=str(exc)) from exc
        action = MoveAction(
            arm=arm,
            target=PoseTarget(reference=request.reference, pose=pose),
            velocity_scale=request.velocity_scale,
            acceleration_scale=request.acceleration_scale,
            planning_time=request.planning_time,
            position_tolerance=request.position_tolerance,
            orientation_tolerance=request.orientation_tolerance,
        )
        return repository.update(
            skill_id,
            SkillUpdate(actions=[*skill.actions, action]),
        )

    @application.post(
        "/api/skills/{skill_id}/executions",
        response_model=SkillExecution,
        status_code=status.HTTP_202_ACCEPTED,
    )
    def execute_skill(skill_id: UUID) -> SkillExecution:
        manager = require_execution_manager()
        try:
            return manager.start(skill_id)
        except SkillNotFoundError as exc:
            raise HTTPException(status_code=404, detail="skill not found") from exc

    @application.get("/api/executions", response_model=list[SkillExecution])
    def list_executions() -> list[SkillExecution]:
        return require_execution_manager().list()

    @application.get("/api/executions/{execution_id}", response_model=SkillExecution)
    def get_execution(execution_id: UUID) -> SkillExecution:
        try:
            return require_execution_manager().get(execution_id)
        except KeyError as exc:
            raise HTTPException(status_code=404, detail="execution not found") from exc

    @application.post(
        "/api/executions/{execution_id}/cancel",
        response_model=SkillExecution,
        status_code=status.HTTP_202_ACCEPTED,
    )
    def cancel_execution(execution_id: UUID) -> SkillExecution:
        try:
            return require_execution_manager().cancel(execution_id)
        except KeyError as exc:
            raise HTTPException(status_code=404, detail="execution not found") from exc

    @application.get(
        "/api/robot/arms/{arm}/pose",
        response_model=PoseTarget,
    )
    def current_arm_pose(
        arm: Literal["left", "right"],
        reference_kind: Literal["world", "marker"] = "world",
        frame_id: str = "world",
    ) -> PoseTarget:
        try:
            reference = PoseReference(kind=reference_kind, frame_id=frame_id)
        except ValidationError as exc:
            raise HTTPException(status_code=422, detail=exc.errors()) from exc
        try:
            pose = require_runtime_method("current_pose")(arm, reference)
        except HTTPException:
            raise
        except Exception as exc:
            raise HTTPException(status_code=409, detail=str(exc)) from exc
        return PoseTarget(reference=reference, pose=pose)

    @application.post(
        "/api/robot/arms/{arm}/nudge",
        response_model=SkillExecution,
        status_code=status.HTTP_202_ACCEPTED,
    )
    def nudge_arm(
        arm: Literal["left", "right"], request: NudgeRequest
    ) -> SkillExecution:
        world_reference = PoseReference(kind="world", frame_id="world")
        try:
            current = require_runtime_method("current_pose")(arm, world_reference)
        except HTTPException:
            raise
        except Exception as exc:
            raise HTTPException(status_code=409, detail=str(exc)) from exc

        delta_rotation = quaternion_from_rpy(
            request.rotation_rpy.x,
            request.rotation_rpy.y,
            request.rotation_rpy.z,
        )
        target = CartesianPose(
            position=Vector3(
                x=current.position.x + request.translation.x,
                y=current.position.y + request.translation.y,
                z=current.position.z + request.translation.z,
            ),
            # Apply the delta in world axes, matching the translation controls.
            orientation=quaternion_multiply(delta_rotation, current.orientation),
        )
        transient = Skill(
            name=f"Manual nudge · {arm}",
            description="Transient manual Cartesian positioning command",
            actions=[
                MoveAction(
                    arm=arm,
                    target=PoseTarget(reference=world_reference, pose=target),
                    velocity_scale=request.velocity_scale,
                    acceleration_scale=request.acceleration_scale,
                )
            ],
        )
        return require_execution_manager().start_skill(transient)

    @application.put("/api/markers/{frame_id}", response_model=MarkerState)
    def set_marker(frame_id: str, pose: CartesianPose) -> MarkerState:
        try:
            reference = PoseReference(kind="marker", frame_id=frame_id)
        except ValidationError as exc:
            raise HTTPException(status_code=422, detail=exc.errors()) from exc
        marker = require_runtime_method("set_marker")(reference.frame_id, pose)
        marker_repository.save(reference.frame_id, pose)
        return marker

    @application.get("/api/markers", response_model=list[MarkerState])
    def list_markers() -> list[MarkerState]:
        return require_runtime_method("list_markers")()

    @application.delete(
        "/api/markers/{frame_id}", status_code=status.HTTP_204_NO_CONTENT
    )
    def delete_marker(frame_id: str) -> Response:
        try:
            require_runtime_method("delete_marker")(frame_id)
        except KeyError as exc:
            raise HTTPException(status_code=404, detail="marker not found") from exc
        try:
            marker_repository.delete(frame_id)
        except MarkerNotFoundError:
            # The runtime is authoritative; tolerate a marker that was only
            # ever broadcast in-process.
            pass
        return Response(status_code=status.HTTP_204_NO_CONTENT)

    @application.get("/", include_in_schema=False)
    def web_ui() -> FileResponse:
        return FileResponse(UI_DIRECTORY / "index.html")

    application.mount("/static", StaticFiles(directory=UI_DIRECTORY), name="static")

    return application


app = create_app()
