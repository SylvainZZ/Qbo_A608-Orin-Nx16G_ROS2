"""One-shot script: fetch all HA entities via ha_bridge, display and write ha_entities.json."""

from __future__ import annotations

import json
import sys
from collections import defaultdict
from datetime import datetime, timezone
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node

from qbo_ha_interfaces.srv import GetAllStates

# Resolves to share/qbo_home_intent/knowledge/ in the install space.
# With --symlink-install this is a symlink to the source tree, so writes persist across rebuilds.
_DEFAULT_OUTPUT = Path(get_package_share_directory("qbo_home_intent")) / "knowledge" / "ha_entities.json"


def _fetch(node: Node) -> list | None:
    cli = node.create_client(GetAllStates, "/ha_bridge/get_all_states")

    if not cli.wait_for_service(timeout_sec=5.0):
        node.get_logger().error("Service /ha_bridge/get_all_states not available — ha_bridge running?")
        return None

    req = GetAllStates.Request()
    req.domain_filter = ""  # empty string = all domains

    future = cli.call_async(req)
    rclpy.spin_until_future_complete(node, future, timeout_sec=30.0)

    if future.result() is None:
        node.get_logger().error("Timeout (30 s) waiting for get_all_states")
        return None

    resp = future.result()
    if not resp.success:
        node.get_logger().error(f"get_all_states error: {resp.error_message}")
        return None

    return list(resp.states)


def _attrs_preview(domain: str, attributes_json: str) -> str:
    try:
        attrs = json.loads(attributes_json) if attributes_json else {}
    except (json.JSONDecodeError, ValueError):
        return ""

    if domain == "light" and "brightness" in attrs:
        return f"  brightness={attrs['brightness']}"
    if domain == "cover" and "current_position" in attrs:
        return f"  pos={attrs['current_position']} %"
    if domain == "climate":
        current = attrs.get("current_temperature", "?")
        setpoint = attrs.get("temperature", "?")
        return f"  mesure={current} °C  consigne={setpoint} °C"
    if domain == "sensor":
        unit = attrs.get("unit_of_measurement", "")
        return f"  {unit}" if unit else ""
    return ""


def _group_by_domain(states: list) -> dict[str, list]:
    by_domain: dict[str, list] = defaultdict(list)
    for s in states:
        domain = s.entity_id.split(".")[0] if "." in s.entity_id else (s.domain or "unknown")
        by_domain[domain].append(s)
    return by_domain


def _display(by_domain: dict[str, list]) -> None:
    total = sum(len(v) for v in by_domain.values())
    for domain in sorted(by_domain):
        entities = sorted(by_domain[domain], key=lambda x: x.entity_id)
        print(f"\n{'─' * 70}")
        print(f"  {domain.upper()}  —  {len(entities)} entité(s)")
        print(f"{'─' * 70}")
        for e in entities:
            area = f"  [{e.area_id}]" if e.area_id else ""
            preview = _attrs_preview(domain, e.attributes)
            print(f"  {e.entity_id:<48}  {e.state:<10}{area}{preview}")

    print(f"\n{'═' * 70}")
    print(f"  TOTAL : {total} entité(s)  —  {len(by_domain)} domaine(s)")
    print(f"{'═' * 70}\n")


def _write_json(by_domain: dict[str, list], output: Path) -> None:
    total = sum(len(v) for v in by_domain.values())

    # Build grouped structure mirroring the terminal display
    domains_data: dict[str, dict] = {}
    for domain in sorted(by_domain):
        entities = sorted(by_domain[domain], key=lambda x: x.entity_id)
        domain_entities = []
        for e in entities:
            try:
                attrs = json.loads(e.attributes) if e.attributes else {}
            except (json.JSONDecodeError, ValueError):
                attrs = {}
            domain_entities.append({
                "entity_id": e.entity_id,
                "friendly_name": e.friendly_name,
                "state": e.state,
                "area_id": e.area_id,
                "attributes": attrs,
            })
        domains_data[domain] = {
            "count": len(entities),
            "entities": domain_entities,
        }

    payload = {
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "total_entities": total,
        "total_domains": len(by_domain),
        "domains": domains_data,
    }

    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")


def _find_source_path(share_path: Path, package_name: str) -> Path | None:
    # share_path = <ws>/install/<pkg>/share/<pkg>  →  workspace root is 4 levels up
    workspace_root = share_path.parents[3]
    candidates = sorted(workspace_root.glob(f"src/**/{package_name}/knowledge/ha_entities.json"))
    return candidates[0] if candidates else None


def main(args=None) -> None:
    import argparse
    parser = argparse.ArgumentParser(description="Sync HA entities to ha_entities.json")
    parser.add_argument("-o", "--output", type=Path, default=_DEFAULT_OUTPUT,
                        help=f"Output path (default: {_DEFAULT_OUTPUT})")
    parsed, ros_args = parser.parse_known_args()
    output: Path = parsed.output

    rclpy.init(args=ros_args)
    node = Node("ha_entities_sync")
    node.get_logger().info("Fetching all HA entities via /ha_bridge/get_all_states …")

    try:
        states = _fetch(node)
        if states is None:
            sys.exit(1)
        node.get_logger().info(f"{len(states)} entities received")
        by_domain = _group_by_domain(states)
        _display(by_domain)
        _write_json(by_domain, output)
        node.get_logger().info(f"Written → {output.resolve()}")
        # Also write to the source tree so the file survives the next colcon build
        src_path = _find_source_path(_DEFAULT_OUTPUT.parent.parent, "qbo_home_intent")
        if src_path and src_path.resolve() != output.resolve():
            _write_json(by_domain, src_path)
            node.get_logger().info(f"Written → {src_path.resolve()}")
        elif not src_path:
            node.get_logger().warn("Source knowledge/ not found — rebuild with --symlink-install to avoid this")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

