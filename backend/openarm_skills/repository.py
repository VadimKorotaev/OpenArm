"""SQLite persistence for validated Skill and marker records."""

from __future__ import annotations

import sqlite3
from pathlib import Path
from uuid import UUID

from .models import CartesianPose, MarkerState, Skill, SkillCreate, SkillUpdate, utc_now


class SkillNotFoundError(LookupError):
    pass


class MarkerNotFoundError(LookupError):
    pass


class SqliteStore:
    """Shared connection handling and schema migrations for one database file."""

    def __init__(self, database_path: str | Path) -> None:
        self.database_path = Path(database_path).expanduser().resolve()

    def _connect(self) -> sqlite3.Connection:
        connection = sqlite3.connect(self.database_path, timeout=5.0)
        connection.row_factory = sqlite3.Row
        connection.execute("PRAGMA foreign_keys = ON")
        connection.execute("PRAGMA busy_timeout = 5000")
        connection.execute("PRAGMA journal_mode = WAL")
        return connection

    def initialize(self) -> None:
        self.database_path.parent.mkdir(parents=True, exist_ok=True)
        with self._connect() as connection:
            connection.execute(
                """
                CREATE TABLE IF NOT EXISTS schema_migrations (
                    version INTEGER PRIMARY KEY,
                    applied_at TEXT NOT NULL
                )
                """
            )
            row = connection.execute(
                "SELECT COALESCE(MAX(version), 0) AS version FROM schema_migrations"
            ).fetchone()
            version = int(row["version"])
            if version < 1:
                connection.execute(
                    """
                    CREATE TABLE skills (
                        id TEXT PRIMARY KEY,
                        name TEXT NOT NULL,
                        schema_version INTEGER NOT NULL,
                        payload TEXT NOT NULL,
                        created_at TEXT NOT NULL,
                        updated_at TEXT NOT NULL
                    )
                    """
                )
                connection.execute(
                    "CREATE INDEX skills_updated_at_idx ON skills(updated_at DESC)"
                )
                connection.execute(
                    "INSERT INTO schema_migrations(version, applied_at) VALUES (?, ?)",
                    (1, utc_now().isoformat()),
                )
            if version < 2:
                # Marker TFs outlive the ROS runtime process. Without this a
                # saved marker-relative Skill breaks after an API restart,
                # because its reference frame would no longer be broadcast.
                connection.execute(
                    """
                    CREATE TABLE markers (
                        frame_id TEXT PRIMARY KEY,
                        payload TEXT NOT NULL,
                        updated_at TEXT NOT NULL
                    )
                    """
                )
                connection.execute(
                    "INSERT INTO schema_migrations(version, applied_at) VALUES (?, ?)",
                    (2, utc_now().isoformat()),
                )


class SkillRepository(SqliteStore):
    @staticmethod
    def _from_row(row: sqlite3.Row) -> Skill:
        return Skill.model_validate_json(row["payload"])

    def create(self, draft: SkillCreate) -> Skill:
        now = utc_now()
        skill = Skill(
            **draft.model_dump(),
            created_at=now,
            updated_at=now,
        )
        with self._connect() as connection:
            connection.execute(
                """
                INSERT INTO skills(
                    id, name, schema_version, payload, created_at, updated_at
                ) VALUES (?, ?, ?, ?, ?, ?)
                """,
                (
                    str(skill.id),
                    skill.name,
                    skill.schema_version,
                    skill.model_dump_json(),
                    skill.created_at.isoformat(),
                    skill.updated_at.isoformat(),
                ),
            )
        return skill

    def list(self, limit: int = 100, offset: int = 0) -> list[Skill]:
        with self._connect() as connection:
            rows = connection.execute(
                """
                SELECT payload
                FROM skills
                ORDER BY updated_at DESC, id ASC
                LIMIT ? OFFSET ?
                """,
                (limit, offset),
            ).fetchall()
        return [self._from_row(row) for row in rows]

    def get(self, skill_id: UUID) -> Skill:
        with self._connect() as connection:
            row = connection.execute(
                "SELECT payload FROM skills WHERE id = ?", (str(skill_id),)
            ).fetchone()
        if row is None:
            raise SkillNotFoundError(str(skill_id))
        return self._from_row(row)

    def update(self, skill_id: UUID, patch: SkillUpdate) -> Skill:
        current = self.get(skill_id)
        values = current.model_dump()
        # Filter only the top-level PATCH fields. Pydantic's recursive
        # ``exclude_unset`` would also remove default discriminators such as
        # ``type='move'`` from server-created nested actions.
        patch_values = patch.model_dump()
        values.update(
            {field: patch_values[field] for field in patch.model_fields_set}
        )
        values["updated_at"] = utc_now()
        updated = Skill.model_validate(values)
        with self._connect() as connection:
            cursor = connection.execute(
                """
                UPDATE skills
                SET name = ?, schema_version = ?, payload = ?, updated_at = ?
                WHERE id = ?
                """,
                (
                    updated.name,
                    updated.schema_version,
                    updated.model_dump_json(),
                    updated.updated_at.isoformat(),
                    str(skill_id),
                ),
            )
        if cursor.rowcount != 1:
            raise SkillNotFoundError(str(skill_id))
        return updated

    def delete(self, skill_id: UUID) -> None:
        with self._connect() as connection:
            cursor = connection.execute(
                "DELETE FROM skills WHERE id = ?", (str(skill_id),)
            )
        if cursor.rowcount != 1:
            raise SkillNotFoundError(str(skill_id))

    def count(self) -> int:
        with self._connect() as connection:
            row = connection.execute("SELECT COUNT(*) AS count FROM skills").fetchone()
        return int(row["count"])


class MarkerRepository(SqliteStore):
    """Durable copy of the marker TFs the ROS runtime broadcasts.

    The runtime stays the source of truth while the process is alive; this
    repository only survives restarts and reseeds the runtime on startup.
    """

    def save(self, frame_id: str, pose: CartesianPose) -> MarkerState:
        marker = MarkerState(frame_id=frame_id, pose=pose)
        with self._connect() as connection:
            connection.execute(
                """
                INSERT INTO markers(frame_id, payload, updated_at)
                VALUES (?, ?, ?)
                ON CONFLICT(frame_id) DO UPDATE SET
                    payload = excluded.payload,
                    updated_at = excluded.updated_at
                """,
                (marker.frame_id, marker.model_dump_json(), utc_now().isoformat()),
            )
        return marker

    def list(self) -> list[MarkerState]:
        with self._connect() as connection:
            rows = connection.execute(
                "SELECT payload FROM markers ORDER BY frame_id ASC"
            ).fetchall()
        return [MarkerState.model_validate_json(row["payload"]) for row in rows]

    def delete(self, frame_id: str) -> None:
        with self._connect() as connection:
            cursor = connection.execute(
                "DELETE FROM markers WHERE frame_id = ?", (frame_id,)
            )
        if cursor.rowcount != 1:
            raise MarkerNotFoundError(frame_id)
