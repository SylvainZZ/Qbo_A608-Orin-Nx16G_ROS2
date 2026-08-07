"""
ha_client.py — High-level Home Assistant client helpers.

Adapted from marialib.ha_client — BMML / BuildingGenerator logic removed,
Qbo-specific additions:
  • HaClient  — persistent authenticated connection with reconnection support
  • session() — one-shot async context manager (for quick fire-and-forget calls)

Relationship with ha_websocket:
  HAWebSocket handles the raw WS protocol (framing, multiplexing).
  HaClient adds lifecycle management (connect/disconnect, reconnection guard)
  and the higher-level helpers used by HaBridgeNode.
"""

import asyncio
import logging
from contextlib import asynccontextmanager
from typing import Any, Callable, Dict, List, Optional

from .ha_websocket import HAWebSocket


class HaClient:
    """
    Persistent, authenticated connection to Home Assistant.

    Typical use inside HaBridgeNode:
        client = HaClient(ws_url, token)
        if await client.connect():
            await client.subscribe_state_changes(my_callback)
            # … connection stays open until recv_loop ends
        await client.disconnect()
    """

    def __init__(self, ws_url: str, token: str) -> None:
        self._ws_url = ws_url
        self._token = token
        self._ws: Optional[HAWebSocket] = None
        self._connected: bool = False
        self._logger = logging.getLogger(__name__)

    # ── Lifecycle ─────────────────────────────────────────────────────────────

    async def connect(self) -> bool:
        """Open and authenticate the WebSocket.  Returns True on success."""
        self._ws = HAWebSocket(self._ws_url, self._token)
        if not await self._ws.connect():
            return False
        auth = await self._ws.authenticate()
        self._connected = auth.get("type") == "auth_ok"
        if not self._connected:
            self._logger.error("HA authentication rejected")
        return self._connected

    async def disconnect(self) -> None:
        if self._ws:
            try:
                await self._ws.disconnect()
            except Exception:
                self._logger.exception("Error during HA disconnect")
        self._connected = False

    @property
    def is_connected(self) -> bool:
        return self._connected

    @property
    def ws(self) -> Optional[HAWebSocket]:
        """Direct access to the underlying HAWebSocket (e.g. to await _recv_task)."""
        return self._ws

    # ── State queries ─────────────────────────────────────────────────────────

    async def get_states(self) -> List[Dict[str, Any]]:
        """Return the current state of every entity."""
        return await self._ws.get_states()

    async def get_state(self, entity_id: str) -> Optional[Dict[str, Any]]:
        """Return the current state dict for one entity, or None if not found."""
        return await self._ws.get_state(entity_id)

    async def get_entities(self) -> List[Dict[str, Any]]:
        """Return the full entity registry (area_id, friendly_name, …)."""
        return await self._ws.get_entities()

    async def get_areas(self) -> List[Dict[str, Any]]:
        """Return the area (room/zone) registry."""
        return await self._ws.get_areas()

    async def get_floors(self) -> List[Dict[str, Any]]:
        """Return the floor registry."""
        return await self._ws.get_floors()

    # ── Service calls ─────────────────────────────────────────────────────────

    async def call_service(
        self,
        domain: str,
        service: str,
        entity_id: Optional[str] = None,
        service_data: Optional[Dict[str, Any]] = None,
    ) -> Dict[str, Any]:
        """
        Call a HA service.  entity_id is wrapped into the target dict.

        Examples:
            await client.call_service("light", "turn_off", "light.salon")
            await client.call_service("cover", "set_cover_position",
                                      "cover.volet_cuisine",
                                      {"position": 50})
            await client.call_service("climate", "set_temperature",
                                      "climate.bureau",
                                      {"temperature": 20.5})
        """
        target = {"entity_id": entity_id} if entity_id else None
        return await self._ws.call_service(domain, service, service_data, target)

    # ── Real-time state change subscription ──────────────────────────────────

    async def subscribe_state_changes(self, callback: Callable) -> None:
        """
        Register *callback* and ask HA to push state_changed events.

        callback signature:  async def on_event(raw_event_dict: dict) -> None
        The raw_event_dict has the shape:
            {"type": "event", "event": {"event_type": "state_changed",
             "data": {"entity_id": "…", "new_state": {…}, "old_state": {…}}}}
        """
        self._ws.add_event_listener(callback)
        await self._ws.subscribe_events("state_changed")
        self._logger.info("Subscribed to state_changed events")

    # ── Connectivity probe ────────────────────────────────────────────────────

    @staticmethod
    async def ping(ws_url: str, token: str) -> bool:
        """Return True if HA is reachable and the token is valid."""
        ws = HAWebSocket(ws_url, token)
        try:
            if not await ws.connect():
                return False
            result = await ws.authenticate()
            return result.get("type") == "auth_ok"
        except Exception:
            return False
        finally:
            await ws.disconnect()


# ── One-shot session context (mirrors marialib.ha_client.session) ─────────────

@asynccontextmanager
async def session(ws_url: str, token: str):
    """
    Async context manager for a single-use HA session.

    Usage:
        async with ha_client.session(url, token) as client:
            states = await client.get_states()
    """
    client = HaClient(ws_url, token)
    if not await client.connect():
        raise RuntimeError("Could not connect / authenticate with Home Assistant")
    try:
        yield client
    finally:
        await client.disconnect()
