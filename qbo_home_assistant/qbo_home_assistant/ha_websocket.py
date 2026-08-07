"""
ha_websocket.py — Low-level asyncio WebSocket client for Home Assistant.

Adapted from marialib.ha_websocket.  Key difference: a persistent receive
loop multiplexes incoming frames so that event subscriptions and one-shot
requests can coexist on the same connection without race conditions.

Architecture:
  connect()         — open WS, DO NOT start recv_loop yet
  authenticate()    — consume auth_required / send token / read auth_ok,
                      THEN start _recv_loop as an asyncio Task
  _recv_loop()      — dispatches every subsequent frame either to a pending
                      request Future (by message id) or to registered event
                      listener callbacks
  _request(payload) — assign a fresh id, register a Future, send, await result
"""

import asyncio
import json
import logging
from typing import Any, Callable, Dict, List, Optional

import websockets

_WEBSOCKETS_VERSION = getattr(websockets, "__version__", "unknown")


class HAWebSocket:
    """Asyncio WebSocket client with concurrent request/event multiplexing."""

    def __init__(self, url: str, token: str) -> None:
        self.url = url
        self.token = token
        self._ws = None
        self._msg_id: int = 0
        # maps message id → Future waiting for the result
        self._pending: Dict[int, asyncio.Future] = {}
        # async callbacks invoked for every "type": "event" frame
        self._event_listeners: List[Callable] = []
        self._recv_task: Optional[asyncio.Task] = None
        self._logger = logging.getLogger(__name__)

    # ── Connection ────────────────────────────────────────────────────────────

    async def connect(self) -> bool:
        """Open the WebSocket.  Does NOT start the receive loop yet."""
        self._logger.info(
            "Connecting to %s  (websockets lib version: %s)", self.url, _WEBSOCKETS_VERSION
        )
        try:
            self._ws = await websockets.connect(self.url)
            self._logger.info("WebSocket opened → %s", self.url)
            return True
        except Exception as exc:
            self._logger.error(
                "WebSocket connect failed [%s]: %s", type(exc).__name__, exc
            )
            if "loop" in str(exc).lower():
                self._logger.error(
                    ">>> websockets %s is incompatible with Python 3.10+.  "
                    "Run: pip3 install --upgrade websockets",
                    _WEBSOCKETS_VERSION,
                )
            return False

    async def authenticate(self) -> Dict[str, Any]:
        """
        Perform the HA WebSocket handshake:
          1. Read auth_required  (first frame, before recv_loop is running)
          2. Send access_token
          3. Read auth_ok / auth_invalid
          4. Start _recv_loop so subsequent frames are multiplexed
        Returns the auth result dict.
        """
        raw = await self._ws.recv()
        first = json.loads(raw)
        self._logger.debug("HA handshake frame 1: %s", first)
        if first.get("type") != "auth_required":
            raise RuntimeError(f"Expected auth_required, got: {first}")

        self._logger.debug("Sending auth token (length=%d)", len(self.token))
        await self._ws.send(json.dumps({"type": "auth", "access_token": self.token}))

        raw = await self._ws.recv()
        result = json.loads(raw)
        self._logger.debug("HA handshake frame 2: %s", result)
        if result.get("type") == "auth_ok":
            self._logger.info("HA authentication OK (HA version %s)", result.get("ha_version"))
        else:
            self._logger.error(
                "HA authentication REJECTED — token invalid or expired. "
                "Full response: %s", result
            )

        # Start the multiplexing loop only after auth is complete
        self._recv_task = asyncio.ensure_future(self._recv_loop())
        return result

    async def disconnect(self) -> None:
        if self._recv_task:
            self._recv_task.cancel()
            try:
                await self._recv_task
            except asyncio.CancelledError:
                pass
            self._recv_task = None
        if self._ws:
            await self._ws.close()
            self._ws = None
            self._logger.info("WebSocket closed")

    # ── Receive loop (frame multiplexer) ──────────────────────────────────────

    async def _recv_loop(self) -> None:
        """Route incoming frames to the right Future or event listeners."""
        try:
            async for raw in self._ws:
                msg = json.loads(raw)
                msg_type = msg.get("type")
                msg_id = msg.get("id")

                if msg_type == "event":
                    for cb in list(self._event_listeners):
                        asyncio.ensure_future(cb(msg))
                elif msg_id is not None and msg_id in self._pending:
                    fut = self._pending.pop(msg_id)
                    if not fut.done():
                        fut.set_result(msg)
                # "pong" and unknown frames are silently ignored
        except asyncio.CancelledError:
            pass
        except Exception as exc:
            self._logger.error("recv_loop terminated: %s", exc)
        finally:
            # Unblock any callers still waiting for a response
            for fut in self._pending.values():
                if not fut.done():
                    fut.set_exception(ConnectionError("WebSocket closed"))
            self._pending.clear()

    # ── Request helper ────────────────────────────────────────────────────────

    def _next_id(self) -> int:
        self._msg_id += 1
        return self._msg_id

    async def _request(self, payload: Dict[str, Any], timeout: float = 15.0) -> Dict[str, Any]:
        """Send a message and wait for the HA response with the same id."""
        if not self._ws:
            raise ConnectionError("WebSocket is not connected")
        msg_id = self._next_id()
        payload["id"] = msg_id
        loop = asyncio.get_running_loop()
        fut: asyncio.Future = loop.create_future()
        self._pending[msg_id] = fut
        await self._ws.send(json.dumps(payload))
        return await asyncio.wait_for(fut, timeout=timeout)

    # ── States ────────────────────────────────────────────────────────────────

    async def get_states(self) -> List[Dict[str, Any]]:
        """Return the current state of every entity."""
        resp = await self._request({"type": "get_states"})
        return resp.get("result", [])

    async def get_state(self, entity_id: str) -> Optional[Dict[str, Any]]:
        """Return the current state dict for one entity, or None if not found."""
        for state in await self.get_states():
            if state.get("entity_id") == entity_id:
                return state
        return None

    # ── Entity / area / floor registries ─────────────────────────────────────

    async def get_entities(self) -> List[Dict[str, Any]]:
        """Return the full entity registry (includes area_id, friendly_name…)."""
        resp = await self._request({"type": "config/entity_registry/list"})
        return resp.get("result", [])

    async def get_areas(self) -> List[Dict[str, Any]]:
        """Return the area (room/zone) registry."""
        resp = await self._request({"type": "config/area_registry/list"})
        return resp.get("result", [])

    async def get_floors(self) -> List[Dict[str, Any]]:
        """Return the floor registry."""
        resp = await self._request({"type": "config/floor_registry/list"})
        return resp.get("result", [])

    # ── Service calls ─────────────────────────────────────────────────────────

    async def call_service(
        self,
        domain: str,
        service: str,
        service_data: Optional[Dict[str, Any]] = None,
        target: Optional[Dict[str, Any]] = None,
    ) -> Dict[str, Any]:
        """
        Call any Home Assistant service.

        Examples:
            call_service("light", "turn_on", target={"entity_id": "light.salon"})
            call_service("light", "turn_on",
                         service_data={"brightness_pct": 60},
                         target={"entity_id": "light.salon"})
            call_service("climate", "set_temperature",
                         service_data={"temperature": 21},
                         target={"entity_id": "climate.bureau"})
        """
        payload: Dict[str, Any] = {
            "type": "call_service",
            "domain": domain,
            "service": service,
        }
        if service_data:
            payload["service_data"] = service_data
        if target:
            payload["target"] = target
        return await self._request(payload)

    # ── Event subscriptions ───────────────────────────────────────────────────

    async def subscribe_events(self, event_type: Optional[str] = None) -> Dict[str, Any]:
        """
        Ask HA to push events of a given type.
        Pass event_type=None to receive all events (high volume).
        Typical use: event_type="state_changed"
        """
        payload: Dict[str, Any] = {"type": "subscribe_events"}
        if event_type:
            payload["event_type"] = event_type
        return await self._request(payload)

    def add_event_listener(self, callback: Callable) -> None:
        """Register an async callback(raw_event_dict) for every event frame."""
        if callback not in self._event_listeners:
            self._event_listeners.append(callback)

    def remove_event_listener(self, callback: Callable) -> None:
        try:
            self._event_listeners.remove(callback)
        except ValueError:
            pass

    # ── Context manager ───────────────────────────────────────────────────────

    async def __aenter__(self) -> "HAWebSocket":
        await self.connect()
        await self.authenticate()
        return self

    async def __aexit__(self, *_) -> None:
        await self.disconnect()
