"""Dialogue context manager: per-session conversation state (Étape 4)."""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Any


@dataclass
class DialogueSession:
    session_id: str
    last_activity: float = field(default_factory=time.monotonic)
    last_intent: dict[str, Any] | None = None
    pending_slot: str = ""
    pending_candidates: list[str] = field(default_factory=list)
    turn_count: int = 0
    waiting_confirmation: bool = False
    pending_action: dict[str, Any] | None = None  # action held for confirmation


class DialogueContextManager:
    SESSION_TIMEOUT_S: float = 120.0

    def __init__(self) -> None:
        self._sessions: dict[str, DialogueSession] = {}

    def get_or_create(self, session_id: str) -> DialogueSession:
        self._purge_expired()
        if session_id not in self._sessions:
            self._sessions[session_id] = DialogueSession(session_id=session_id)
        return self._sessions[session_id]

    def update(self, session: DialogueSession) -> None:
        session.last_activity = time.monotonic()
        session.turn_count += 1
        self._sessions[session.session_id] = session

    def close(self, session_id: str) -> None:
        self._sessions.pop(session_id, None)

    def _purge_expired(self) -> None:
        now = time.monotonic()
        expired = [
            sid
            for sid, s in self._sessions.items()
            if now - s.last_activity > self.SESSION_TIMEOUT_S
        ]
        for sid in expired:
            del self._sessions[sid]
