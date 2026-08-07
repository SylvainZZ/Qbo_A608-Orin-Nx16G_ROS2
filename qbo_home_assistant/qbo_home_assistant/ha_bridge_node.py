"""
ha_bridge_node.py — Main ROS2 node bridging Home Assistant and Qbo.

Design
------
asyncio runs in a dedicated background thread (_bg_thread).
ROS2 service handlers block on asyncio.run_coroutine_threadsafe(...).result()
so they wait for HA's reply before returning — standard ROS2 service behaviour.
rclpy publishers are thread-safe; they are called directly from the asyncio
thread for real-time state_changed events.

Topics published (relative to node namespace)
----------------------------------------------
  ~/entity_state   [qbo_ha_interfaces/EntityState]       each state_changed event
  ~/connection     [qbo_ha_interfaces/HaConnectionStatus] latched, updated on every
                                                          connect/disconnect cycle

Services advertised
-------------------
  ~/call_service       [qbo_ha_interfaces/CallHaService]
  ~/get_entity_state   [qbo_ha_interfaces/GetEntityState]
  ~/get_all_states     [qbo_ha_interfaces/GetAllStates]

Parameters
----------
  ha_ws_url           (string)  ws://localhost:8123/api/websocket
  ha_token            (string)  HA long-lived access token (required)
  reconnect_delay_s   (float)   seconds between reconnection attempts  [5.0]
  all_states_period_s (float)   periodic full-snapshot interval in s   [0.0]
                                  ≤ 0 disables the timer
"""

import asyncio
import json
import logging
import threading
from typing import Any, Dict, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from builtin_interfaces.msg import Time as RosTime

from qbo_ha_interfaces.msg import EntityState, HaConnectionStatus
from qbo_ha_interfaces.srv import CallHaService, GetEntityState, GetAllStates

from .ha_client import HaClient


# ── Helpers ───────────────────────────────────────────────────────────────────

def _ros_now(node: Node) -> RosTime:
    sec, nsec = node.get_clock().now().seconds_nanoseconds()
    t = RosTime()
    t.sec = sec
    t.nanosec = nsec
    return t


def _raw_to_entity_state(raw: Dict[str, Any], node: Node) -> EntityState:
    """Convert a HA state dict to an EntityState ROS2 message."""
    msg = EntityState()
    msg.stamp = _ros_now(node)
    msg.entity_id = raw.get("entity_id", "")
    msg.domain = msg.entity_id.split(".")[0] if "." in msg.entity_id else ""
    attrs = raw.get("attributes") or {}
    msg.friendly_name = str(attrs.get("friendly_name", msg.entity_id))
    msg.state = str(raw.get("state", ""))
    msg.attributes = json.dumps(attrs)
    msg.area_id = ""  # populated by optional entity-registry cache (future work)
    return msg


# ── Node ─────────────────────────────────────────────────────────────────────

class HaBridgeNode(Node):

    def __init__(self) -> None:
        super().__init__("ha_bridge")

        # ── Parameters ───────────────────────────────────────────────────────
        self.declare_parameter("ha_ws_url", "ws://localhost:8123/api/websocket")
        self.declare_parameter("ha_token", "")
        self.declare_parameter("reconnect_delay_s", 5.0)
        self.declare_parameter("all_states_period_s", 0.0)

        self._ws_url: str = self.get_parameter("ha_ws_url").value
        self._token: str = self.get_parameter("ha_token").value
        reconnect_delay: float = self.get_parameter("reconnect_delay_s").value
        all_states_period: float = self.get_parameter("all_states_period_s").value

        if not self._token:
            self.get_logger().warning(
                "Parameter 'ha_token' is empty — set it via launch file or CLI"
            )

        # ── QoS ──────────────────────────────────────────────────────────────
        # connection topic is latched so late subscribers receive the last status
        _latched = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        # ── Publishers ───────────────────────────────────────────────────────
        self._pub_state = self.create_publisher(EntityState, "~/entity_state", 50)
        self._pub_conn = self.create_publisher(HaConnectionStatus, "~/connection", _latched)

        # ── Services ─────────────────────────────────────────────────────────
        self.create_service(CallHaService, "~/call_service", self._srv_call_service)
        self.create_service(GetEntityState, "~/get_entity_state", self._srv_get_entity_state)
        self.create_service(GetAllStates, "~/get_all_states", self._srv_get_all_states)

        # ── Asyncio loop in a dedicated background thread ─────────────────────
        self._loop = asyncio.new_event_loop()
        # _ha_client is written from asyncio thread, read from ROS2 threads;
        # Python GIL makes None-check safe without explicit locking.
        self._ha_client: Optional[HaClient] = None
        self._bg_thread = threading.Thread(
            target=self._loop.run_forever,
            name="ha_asyncio",
            daemon=True,
        )
        self._bg_thread.start()

        # ── Optional periodic full-snapshot timer ─────────────────────────────
        if all_states_period > 0:
            self._timer_snapshot = self.create_timer(
                all_states_period, self._cb_snapshot_timer
            )

        # ── Start connection loop ─────────────────────────────────────────────
        asyncio.run_coroutine_threadsafe(
            self._connection_loop(reconnect_delay), self._loop
        )
        self.get_logger().info(f"HaBridgeNode initialised (HA: {self._ws_url})")

    # ── Async connection / reconnection loop ──────────────────────────────────

    async def _connection_loop(self, reconnect_delay: float) -> None:
        """Connect, subscribe, keep alive, reconnect on drop — runs forever."""
        while rclpy.ok():
            self._publish_conn_status(connected=False, message="Connecting…")
            client = HaClient(self._ws_url, self._token)
            try:
                if not await client.connect():
                    raise ConnectionError("WebSocket connection or authentication failed (see logs above)")

                self._ha_client = client
                self._publish_conn_status(connected=True, message="Connected")
                self.get_logger().info("Connected to Home Assistant")

                # Publish current snapshot so subscribers start with fresh data
                await self._snapshot_and_publish(client)

                # Subscribe to real-time state changes
                await client.subscribe_state_changes(self._on_state_changed)

                # Block until the connection drops (recv_loop task ends)
                recv_task = client.ws._recv_task
                if recv_task:
                    await recv_task

            except asyncio.CancelledError:
                break
            except Exception as exc:
                self.get_logger().warning(
                    f"HA connection error: {exc} — retrying in {reconnect_delay:.1f} s"
                )
            finally:
                self._ha_client = None
                await client.disconnect()
                self._publish_conn_status(connected=False, message="Disconnected")

            await asyncio.sleep(reconnect_delay)

    # ── HA event callback (asyncio thread) ────────────────────────────────────

    async def _on_state_changed(self, raw_event: Dict[str, Any]) -> None:
        """Called for every state_changed event pushed by HA."""
        data = raw_event.get("event", {}).get("data", {})
        new_state = data.get("new_state")
        if not new_state:
            return
        # rclpy publishers are thread-safe
        self._pub_state.publish(_raw_to_entity_state(new_state, self))

    # ── Full-snapshot helpers ─────────────────────────────────────────────────

    def _cb_snapshot_timer(self) -> None:
        """ROS2 timer callback — fire-and-forget asyncio snapshot."""
        if self._ha_client is None:
            return
        asyncio.run_coroutine_threadsafe(
            self._snapshot_and_publish(self._ha_client), self._loop
        )

    async def _snapshot_and_publish(self, client: HaClient) -> None:
        try:
            states = await client.get_states()
            for raw in states:
                self._pub_state.publish(_raw_to_entity_state(raw, self))
            self.get_logger().debug(f"Published snapshot of {len(states)} entities")
        except Exception as exc:
            self.get_logger().warning(f"Snapshot failed: {exc}")

    # ── Connection status helper ──────────────────────────────────────────────

    def _publish_conn_status(self, *, connected: bool, message: str) -> None:
        msg = HaConnectionStatus()
        msg.stamp = _ros_now(self)
        msg.connected = connected
        msg.ha_url = self._ws_url
        msg.message = message
        self._pub_conn.publish(msg)

    # ── Service: call a HA service ────────────────────────────────────────────

    def _srv_call_service(
        self,
        request: CallHaService.Request,
        response: CallHaService.Response,
    ) -> CallHaService.Response:
        if self._ha_client is None:
            response.success = False
            response.error_message = "Not connected to Home Assistant"
            return response
        try:
            service_data = json.loads(request.service_data) if request.service_data.strip() else None
            fut = asyncio.run_coroutine_threadsafe(
                self._ha_client.call_service(
                    request.domain,
                    request.service,
                    entity_id=request.entity_id or None,
                    service_data=service_data,
                ),
                self._loop,
            )
            result = fut.result(timeout=15.0)
            response.success = result.get("success", True)
            response.result = json.dumps(result)
        except Exception as exc:
            response.success = False
            response.error_message = str(exc)
            self.get_logger().error(f"call_service error: {exc}")
        return response

    # ── Service: get one entity state ────────────────────────────────────────

    def _srv_get_entity_state(
        self,
        request: GetEntityState.Request,
        response: GetEntityState.Response,
    ) -> GetEntityState.Response:
        if self._ha_client is None:
            response.success = False
            response.error_message = "Not connected to Home Assistant"
            return response
        try:
            fut = asyncio.run_coroutine_threadsafe(
                self._ha_client.get_state(request.entity_id), self._loop
            )
            raw = fut.result(timeout=15.0)
            if raw is None:
                response.success = False
                response.error_message = f"Entity '{request.entity_id}' not found"
            else:
                response.success = True
                response.entity_state = _raw_to_entity_state(raw, self)
        except Exception as exc:
            response.success = False
            response.error_message = str(exc)
            self.get_logger().error(f"get_entity_state error: {exc}")
        return response

    # ── Service: get all states (optionally filtered by domain) ───────────────

    def _srv_get_all_states(
        self,
        request: GetAllStates.Request,
        response: GetAllStates.Response,
    ) -> GetAllStates.Response:
        if self._ha_client is None:
            response.success = False
            response.error_message = "Not connected to Home Assistant"
            return response
        try:
            fut = asyncio.run_coroutine_threadsafe(
                self._ha_client.get_states(), self._loop
            )
            states = fut.result(timeout=15.0)
            domain_filter = request.domain_filter.strip()
            msgs = []
            for raw in states:
                if domain_filter:
                    entity_id = raw.get("entity_id", "")
                    if not entity_id.startswith(domain_filter + "."):
                        continue
                msgs.append(_raw_to_entity_state(raw, self))
            response.success = True
            response.states = msgs
        except Exception as exc:
            response.success = False
            response.error_message = str(exc)
            self.get_logger().error(f"get_all_states error: {exc}")
        return response

    # ── Cleanup ───────────────────────────────────────────────────────────────

    def destroy_node(self) -> None:
        self._loop.call_soon_threadsafe(self._loop.stop)
        self._bg_thread.join(timeout=5.0)
        super().destroy_node()
