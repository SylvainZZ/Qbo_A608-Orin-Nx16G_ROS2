"""Test runner: calls /home_intent/parse for each case in tests/*.yaml."""

from __future__ import annotations

import sys
from pathlib import Path
from typing import Any

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node

from qbo_home_interfaces.srv import ParseHomeCommand

_INTENT_TYPE = {"unknown": 0, "control": 1, "query": 2, "status": 3, "cancel": 4}
_STATUS = {"parsed": 0, "resolved": 1, "needs_clarification": 2, "rejected": 3}

_TESTS_DIR = Path(get_package_share_directory("qbo_home_intent")) / "tests"

_G = "\033[92m"   # green
_R = "\033[91m"   # red
_Y = "\033[93m"   # yellow
_B = "\033[1m"    # bold
_D = "\033[2m"    # dim
_X = "\033[0m"    # reset

_SEP_D = "\u2550" * 72   # ════... (double, file header)
_SEP_S = "\u2500" * 72   # ────... (single, final summary)


def _call_parse(node: Node, cli, text: str) -> ParseHomeCommand.Response | None:
    req = ParseHomeCommand.Request()
    req.text = text
    req.session_id = "test_runner"
    req.resolve = True
    req.execute = False  # never trigger real HA actions during tests
    future = cli.call_async(req)
    rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)
    return future.result()


def _extract_actual(resp: ParseHomeCommand.Response) -> dict[str, Any]:
    intent = resp.intent
    return {
        "intent_type": intent.intent_type,
        "action": intent.action,
        "area": intent.target.area,
        "device_class": intent.target.device_class,
        "entity_id": intent.target.entity_id,
        "status": intent.status,
        "missing_slot": intent.missing_slot,
        "parameters": {p.name: p.value for p in intent.parameters},
        "risk_level": intent.risk_level,
        "confirmation_required": intent.confirmation_required,
    }


def _normalize_expected(raw: dict) -> dict[str, Any]:
    result = dict(raw)
    if isinstance(result.get("intent_type"), str):
        result["intent_type"] = _INTENT_TYPE.get(result["intent_type"], -1)
    if isinstance(result.get("status"), str):
        result["status"] = _STATUS.get(result["status"], -1)
    return result


def _compare(expected: dict, actual: dict) -> list[tuple[str, Any, Any]]:
    """Return (field, expected, actual) for every mismatch."""
    mismatches = []
    for field, exp_val in expected.items():
        if field == "note":
            continue
        if field == "parameters":
            for k, v in exp_val.items():
                got = actual.get("parameters", {}).get(k)
                if str(got) != str(v):
                    mismatches.append((f"parameters.{k}", v, got))
        else:
            got = actual.get(field)
            if got != exp_val:
                mismatches.append((field, exp_val, got))
    return mismatches


def _run_file(node: Node, cli, yaml_path: Path) -> tuple[int, int]:
    with open(yaml_path, encoding="utf-8") as fh:
        data = yaml.safe_load(fh)
    tests = data.get("tests", [])
    passed = 0

    print(f"\n{_B}{_SEP_D}{_X}")
    print(f"{_B}  {yaml_path.name}  \u2014  {len(tests)} test(s){_X}")
    print(f"{_B}{_SEP_D}{_X}")

    for i, case in enumerate(tests, 1):
        text = case["text"]
        expected = _normalize_expected(case.get("expected", {}))

        resp = _call_parse(node, cli, text)
        if resp is None:
            print(f"  {_R}\u2717{_X} [{i:02d}] TIMEOUT  {_D}{text!r}{_X}")
            continue

        actual = _extract_actual(resp)
        mismatches = _compare(expected, actual)

        if not mismatches:
            passed += 1
            print(f"  {_G}\u2713{_X} [{i:02d}]  {_D}{text!r}{_X}")
        else:
            print(f"  {_R}\u2717{_X} [{i:02d}]  {_B}{text!r}{_X}")
            for field, exp_val, act_val in mismatches:
                print(f"         {_Y}{field:<24}{_X}"
                      f"  attendu {_G}{exp_val!r}{_X}"
                      f"  obtenu  {_R}{act_val!r}{_X}")

    status_line = f"{_G}{passed}/{len(tests)}{_X}" if passed == len(tests) else f"{_R}{passed}/{len(tests)}{_X}"
    print(f"  {_D}  \u2514 {status_line} passé(s){_X}")
    return passed, len(tests)


def main(args=None) -> None:
    import argparse
    parser = argparse.ArgumentParser(description="Run qbo_home_intent test suites")
    parser.add_argument("files", nargs="*",
                        help="Fichiers YAML spécifiques (défaut: tous les tests)")
    parsed, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = Node("home_intent_test_runner")

    cli = node.create_client(ParseHomeCommand, "/home_intent/parse")
    node.get_logger().info("Attente du service /home_intent/parse …")
    if not cli.wait_for_service(timeout_sec=5.0):
        node.get_logger().error(
            "/home_intent/parse indisponible.\n"
            "  Vérifiez que le nœud est lancé : ros2 launch qbo_home_intent home_intent.launch.py\n"
            "  Ou : ros2 node list | grep home_intent"
        )
        rclpy.shutdown()
        sys.exit(1)

    yaml_files = [Path(f) for f in parsed.files] if parsed.files else sorted(_TESTS_DIR.glob("*.yaml"))

    total_passed = total_tests = 0
    try:
        for yaml_path in yaml_files:
            p, t = _run_file(node, cli, yaml_path)
            total_passed += p
            total_tests += t
    finally:
        node.destroy_node()
        rclpy.shutdown()

    failed = total_tests - total_passed
    col = _G if failed == 0 else _R
    print(f"\n{_B}{_SEP_S}{_X}")
    print(f"  {_B}TOTAL : {col}{total_passed}/{total_tests} passés{_X}  "
          f"{_R if failed else _D}({failed} échec(s)){_X}")
    print(f"{_B}{_SEP_S}{_X}\n")
    sys.exit(0 if failed == 0 else 1)


if __name__ == "__main__":
    main()
