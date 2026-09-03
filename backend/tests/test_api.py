import asyncio
import math
from contextlib import asynccontextmanager
from pathlib import Path

import httpx
from fastapi import FastAPI

from openarm_skills.app import create_app
from openarm_skills.models import CartesianPose, Quaternion, Vector3


class FakeRuntime:
    def __init__(self) -> None:
        self.ready = False
        self.calls: list[tuple[str, str]] = []
        self.move_actions = []
        self.markers: dict[str, CartesianPose] = {}

    def start(self) -> None:
        self.ready = True

    def stop(self) -> None:
        self.ready = False

    def move(self, action, cancellation) -> None:
        cancellation.raise_if_cancelled()
        self.calls.append(("move", action.arm))
        self.move_actions.append(action)

    def gripper(self, action, cancellation) -> None:
        cancellation.raise_if_cancelled()
        self.calls.append(("gripper", action.arm))

    def current_pose(self, arm, reference) -> CartesianPose:
        return CartesianPose(
            position=Vector3(x=0.25, y=0.15 if arm == "left" else -0.15, z=0.46),
            orientation=Quaternion(),
        )

    def set_marker(self, frame_id, pose):
        from openarm_skills.models import MarkerState

        self.markers[frame_id] = pose
        return MarkerState(frame_id=frame_id, pose=pose)

    def list_markers(self):
        from openarm_skills.models import MarkerState

        return [
            MarkerState(frame_id=frame_id, pose=pose)
            for frame_id, pose in sorted(self.markers.items())
        ]

    def delete_marker(self, frame_id):
        if self.markers.pop(frame_id, None) is None:
            raise KeyError(frame_id)


@asynccontextmanager
async def api_client(app: FastAPI):
    transport = httpx.ASGITransport(app=app)
    async with app.router.lifespan_context(app):
        async with httpx.AsyncClient(
            transport=transport, base_url="http://testserver"
        ) as client:
            yield client


def move_action() -> dict:
    return {
        "type": "move",
        "arm": "left",
        "target": {
            "reference": {"kind": "world", "frame_id": "world"},
            "pose": {
                "position": {"x": 0.245, "y": 0.154, "z": 0.463},
                "orientation": {"w": 1.0},
            },
        },
    }


def create_payload(name: str = "Left arm demo") -> dict:
    return {
        "name": name,
        "description": "Persistent CRUD test",
        "actions": [move_action(), {"type": "wait", "duration": 0.1}],
    }


def test_crud_and_persistence_across_app_restart(tmp_path: Path) -> None:
    async def scenario() -> None:
        database = tmp_path / "skills.db"
        first_app = create_app(database, ros_enabled=False)

        async with api_client(first_app) as client:
            response = await client.post("/api/skills", json=create_payload())
            assert response.status_code == 201
            created = response.json()
            skill_id = created["id"]

            assert (await client.get("/health")).json()["skills"] == 1
            saved = await client.get(f"/api/skills/{skill_id}")
            assert saved.json()["name"] == "Left arm demo"

            response = await client.patch(
                f"/api/skills/{skill_id}",
                json={"name": "Renamed demo"},
            )
            assert response.status_code == 200
            assert response.json()["name"] == "Renamed demo"
            assert len(response.json()["actions"]) == 2

        second_app = create_app(database, ros_enabled=False)
        async with api_client(second_app) as client:
            skills = (await client.get("/api/skills")).json()
            assert [skill["name"] for skill in skills] == ["Renamed demo"]

            assert (await client.delete(f"/api/skills/{skill_id}")).status_code == 204
            assert (await client.get(f"/api/skills/{skill_id}")).status_code == 404
            assert (await client.delete(f"/api/skills/{skill_id}")).status_code == 404

    asyncio.run(scenario())


def test_api_rejects_invalid_skill_and_empty_patch(tmp_path: Path) -> None:
    async def scenario() -> None:
        app = create_app(tmp_path / "invalid.db", ros_enabled=False)
        async with api_client(app) as client:
            invalid = create_payload()
            invalid["actions"][0]["target"]["pose"]["orientation"] = {"w": 2.0}
            response = await client.post("/api/skills", json=invalid)
            assert response.status_code == 422

            created = (await client.post("/api/skills", json=create_payload())).json()
            empty = await client.patch(f"/api/skills/{created['id']}", json={})
            null = await client.patch(
                f"/api/skills/{created['id']}", json={"name": None}
            )
            assert empty.status_code == 422
            assert null.status_code == 422

    asyncio.run(scenario())


def test_list_pagination(tmp_path: Path) -> None:
    async def scenario() -> None:
        app = create_app(tmp_path / "pagination.db", ros_enabled=False)
        async with api_client(app) as client:
            for index in range(3):
                response = await client.post(
                    "/api/skills", json=create_payload(f"Skill {index}")
                )
                assert response.status_code == 201

            first_page = await client.get("/api/skills?limit=2")
            second_page = await client.get("/api/skills?limit=2&offset=2")
            invalid_page = await client.get("/api/skills?limit=0")
            assert len(first_page.json()) == 2
            assert len(second_page.json()) == 1
            assert invalid_page.status_code == 422

    asyncio.run(scenario())


async def wait_for_terminal_execution(client: httpx.AsyncClient, execution_id: str) -> dict:
    for _ in range(100):
        record = (await client.get(f"/api/executions/{execution_id}")).json()
        if record["status"] in {"succeeded", "failed", "cancelled"}:
            return record
        await asyncio.sleep(0.02)
    raise AssertionError("execution did not reach a terminal state")


def test_execute_skill_and_read_current_pose(tmp_path: Path) -> None:
    async def scenario() -> None:
        runtime = FakeRuntime()
        app = create_app(
            tmp_path / "execution.db",
            runtime_factory=lambda: runtime,
        )
        async with api_client(app) as client:
            health = (await client.get("/health")).json()
            assert health["ros_enabled"] is True
            assert health["ros_ready"] is True

            created = (await client.post("/api/skills", json=create_payload())).json()
            response = await client.post(
                f"/api/skills/{created['id']}/executions"
            )
            assert response.status_code == 202
            execution = await wait_for_terminal_execution(
                client, response.json()["id"]
            )
            assert execution["status"] == "succeeded"
            assert execution["error"] is None
            assert runtime.calls == [("move", "left")]

            pose = (await client.get("/api/robot/arms/left/pose")).json()
            assert pose["reference"] == {"kind": "world", "frame_id": "world"}
            assert pose["pose"]["position"]["x"] == 0.25

    asyncio.run(scenario())


def test_cancel_waiting_execution(tmp_path: Path) -> None:
    async def scenario() -> None:
        runtime = FakeRuntime()
        app = create_app(
            tmp_path / "cancel.db",
            runtime_factory=lambda: runtime,
        )
        async with api_client(app) as client:
            payload = {
                "name": "Cancellable wait",
                "actions": [{"type": "wait", "duration": 5.0}],
            }
            created = (await client.post("/api/skills", json=payload)).json()
            started = await client.post(
                f"/api/skills/{created['id']}/executions"
            )
            execution_id = started.json()["id"]
            assert (
                await client.post(f"/api/executions/{execution_id}/cancel")
            ).status_code == 202
            execution = await wait_for_terminal_execution(client, execution_id)
            assert execution["status"] == "cancelled"

    asyncio.run(scenario())


def test_marker_management_and_capture_pose(tmp_path: Path) -> None:
    async def scenario() -> None:
        runtime = FakeRuntime()
        app = create_app(
            tmp_path / "capture.db",
            runtime_factory=lambda: runtime,
        )
        async with api_client(app) as client:
            marker_pose = {
                "position": {"x": 0.3, "y": 0.0, "z": 0.25},
                "orientation": {"w": 1.0},
            }
            created_marker = await client.put(
                "/api/markers/aruco_marker_1", json=marker_pose
            )
            assert created_marker.status_code == 200
            assert len((await client.get("/api/markers")).json()) == 1

            current = await client.get(
                "/api/robot/arms/left/pose",
                params={
                    "reference_kind": "marker",
                    "frame_id": "aruco_marker_1",
                },
            )
            assert current.status_code == 200
            assert current.json()["reference"] == {
                "kind": "marker",
                "frame_id": "aruco_marker_1",
            }

            skill = (
                await client.post(
                    "/api/skills",
                    json={
                        "name": "captured marker pose",
                        "actions": [{"type": "wait", "duration": 0.1}],
                    },
                )
            ).json()
            capture = await client.post(
                f"/api/skills/{skill['id']}/actions/capture-pose?arm=left",
                json={
                    "reference": {
                        "kind": "marker",
                        "frame_id": "aruco_marker_1",
                    }
                },
            )
            assert capture.status_code == 200
            captured = capture.json()
            assert len(captured["actions"]) == 2
            assert captured["actions"][1]["type"] == "move"
            assert captured["actions"][1]["target"]["reference"]["kind"] == "marker"

            assert (
                await client.delete("/api/markers/aruco_marker_1")
            ).status_code == 204
            assert (
                await client.delete("/api/markers/aruco_marker_1")
            ).status_code == 404

    asyncio.run(scenario())


def test_markers_survive_an_api_restart(tmp_path: Path) -> None:
    async def scenario() -> None:
        database = tmp_path / "markers.db"
        marker_pose = {
            "position": {"x": 0.3, "y": 0.05, "z": 0.25},
            "orientation": {"w": 1.0},
        }

        first_runtime = FakeRuntime()
        first_app = create_app(database, runtime_factory=lambda: first_runtime)
        async with api_client(first_app) as client:
            assert (
                await client.put("/api/markers/aruco_marker_1", json=marker_pose)
            ).status_code == 200

        # A fresh runtime starts with no in-memory markers at all.
        second_runtime = FakeRuntime()
        assert second_runtime.markers == {}
        second_app = create_app(database, runtime_factory=lambda: second_runtime)
        async with api_client(second_app) as client:
            markers = (await client.get("/api/markers")).json()
            assert [marker["frame_id"] for marker in markers] == ["aruco_marker_1"]
            assert markers[0]["pose"]["position"]["y"] == 0.05
            # The TF is broadcast again, not merely listed from the database.
            assert "aruco_marker_1" in second_runtime.markers

            assert (
                await client.delete("/api/markers/aruco_marker_1")
            ).status_code == 204

        third_runtime = FakeRuntime()
        third_app = create_app(database, runtime_factory=lambda: third_runtime)
        async with api_client(third_app) as client:
            assert (await client.get("/api/markers")).json() == []
            assert third_runtime.markers == {}

    asyncio.run(scenario())


def test_manual_nudge_executes_transient_world_move(tmp_path: Path) -> None:
    async def scenario() -> None:
        runtime = FakeRuntime()
        app = create_app(tmp_path / "nudge.db", runtime_factory=lambda: runtime)
        async with api_client(app) as client:
            response = await client.post(
                "/api/robot/arms/right/nudge",
                json={"translation": {"x": 0.01, "y": 0.0, "z": 0.0}},
            )
            assert response.status_code == 202
            execution = await wait_for_terminal_execution(client, response.json()["id"])
            assert execution["status"] == "succeeded"
            assert execution["skill_name"] == "Manual nudge · right"
            action = runtime.move_actions[-1]
            assert action.arm == "right"
            assert action.target.reference.frame_id == "world"
            assert action.target.pose.position.x == 0.26

            rotation = await client.post(
                "/api/robot/arms/right/nudge",
                json={"rotation_rpy": {"x": math.radians(5), "y": 0.0, "z": 0.0}},
            )
            assert rotation.status_code == 202
            rotation_execution = await wait_for_terminal_execution(
                client, rotation.json()["id"]
            )
            assert rotation_execution["status"] == "succeeded"
            rotated_action = runtime.move_actions[-1]
            assert math.isclose(
                rotated_action.target.pose.orientation.x,
                math.sin(math.radians(2.5)),
            )
            assert math.isclose(
                rotated_action.target.pose.orientation.w,
                math.cos(math.radians(2.5)),
            )

            zero = await client.post(
                "/api/robot/arms/right/nudge",
                json={"translation": {"x": 0.0, "y": 0.0, "z": 0.0}},
            )
            too_large = await client.post(
                "/api/robot/arms/right/nudge",
                json={"translation": {"x": 0.051, "y": 0.0, "z": 0.0}},
            )
            assert zero.status_code == 422
            assert too_large.status_code == 422

    asyncio.run(scenario())


def test_web_ui_and_static_assets_are_served(tmp_path: Path) -> None:
    async def scenario() -> None:
        app = create_app(tmp_path / "ui.db", ros_enabled=False)
        async with api_client(app) as client:
            page = await client.get("/")
            script = await client.get("/static/app.js")
            stylesheet = await client.get("/static/styles.css")
            assert page.status_code == 200
            assert "OpenArm Skill Studio" in page.text
            assert 'id="marker-roll"' in page.text
            assert 'id="marker-pitch"' in page.text
            assert 'id="marker-yaw"' in page.text
            assert 'id="marker-qx"' not in page.text
            assert 'id="nudge-angle-step"' in page.text
            assert 'data-nudge-rotation-axis="x"' in page.text
            assert 'data-nudge-rotation-axis="y"' in page.text
            assert 'data-nudge-rotation-axis="z"' in page.text
            marker_option = '<option value="marker">marker</option>'
            world_option = '<option value="world">world</option>'
            assert page.text.index(marker_option) < page.text.index(world_option)
            assert 'id="capture-frame" value="aruco_marker_1"' in page.text
            disabled_capture_frame = (
                'id="capture-frame" value="aruco_marker_1" '
                'aria-label="TF frame маркера" disabled'
            )
            assert disabled_capture_frame not in page.text
            assert script.status_code == 200
            assert "nudge_openarm" in script.text
            assert "quaternionFromEulerDegrees" in script.text
            assert "eulerDegreesFromQuaternion" in script.text
            assert 'section("Ориентация RPY, °"' in script.text
            assert 'section("Ориентация quaternion"' not in script.text
            assert "loadCurrentMovePose" in script.text
            assert "reference_kind: selection.kind" in script.text
            assert stylesheet.status_code == 200

    asyncio.run(scenario())
