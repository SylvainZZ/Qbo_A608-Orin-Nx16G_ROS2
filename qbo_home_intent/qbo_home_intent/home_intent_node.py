"""ROS2 node: home intent pipeline (parse → resolve → execute)."""

from __future__ import annotations

import uuid

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

from qbo_home_interfaces.msg import HomeIntent, HomeParameter, HomeTarget, IntentResult
from qbo_home_interfaces.srv import ExecuteHomeIntent, ParseHomeCommand, ResolveHomeIntent

from .classifier import IntentClassifier
from .dialogue_context import DialogueContextManager
from .executor import HomeExecutor
from .normalizer import normalize
from .resolver import HomeResolver
from .response_builder import ResponseBuilder
from .slot_extractor import SlotExtractor
from .validator import IntentValidator

# Default device_class inferred from action when the user doesn't name an object type
_ACTION_DEFAULT_CLASS: dict[str, str] = {
    "turn_on": "light",
    "turn_off": "light",
    "set_brightness": "light",
    "set_color": "light",
    "open": "cover",
    "close": "cover",
    "stop": "cover",
    "set_position": "cover",
    "set_temperature": "climate",
    "select_option": "select",
    "read_temperature": "temperature",
    "read_power": "power",
    "read_energy": "energy",
}


class HomeIntentNode(Node):
    def __init__(self) -> None:
        super().__init__("home_intent")

        self._classifier = IntentClassifier()
        self._extractor = SlotExtractor()
        self._resolver = HomeResolver()
        self._validator = IntentValidator()
        self._executor = HomeExecutor(self)
        self._response = ResponseBuilder()
        self._context = DialogueContextManager()

        # ReentrantCallbackGroup lets service callbacks run concurrently so the
        # executor thread is free to process ha_bridge client responses.
        _cbg = ReentrantCallbackGroup()
        self.create_service(ParseHomeCommand, "~/parse", self._handle_parse, callback_group=_cbg)
        self.create_service(ResolveHomeIntent, "~/resolve", self._handle_resolve, callback_group=_cbg)
        self.create_service(ExecuteHomeIntent, "~/execute", self._handle_execute, callback_group=_cbg)

        self.get_logger().info("home_intent node ready")

    # ------------------------------------------------------------------
    # Service handlers
    # ------------------------------------------------------------------

    def _handle_parse(
        self,
        request: ParseHomeCommand.Request,
        response: ParseHomeCommand.Response,
    ) -> ParseHomeCommand.Response:
        try:
            intent = self._parse(request.text, request.session_id)
            response.intent = intent

            if request.resolve or request.execute:
                intent = self._resolve(intent, request.session_id)
                response.intent = intent

            if request.execute and intent.status == HomeIntent.STATUS_RESOLVED:
                result = self._execute(intent, dry_run=False)
                response.result = result
                response.success = result.success
            else:
                response.result = self._build_status_result(intent, request.session_id)
                response.success = True

        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"parse handler error: {exc}")
            response.success = False
            response.error_message = str(exc)

        return response

    def _handle_resolve(
        self,
        request: ResolveHomeIntent.Request,
        response: ResolveHomeIntent.Response,
    ) -> ResolveHomeIntent.Response:
        try:
            response.resolved_intent = self._resolve(request.intent, request.session_id)
            response.success = True
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"resolve handler error: {exc}")
            response.success = False
            response.error_message = str(exc)
        return response

    def _handle_execute(
        self,
        request: ExecuteHomeIntent.Request,
        response: ExecuteHomeIntent.Response,
    ) -> ExecuteHomeIntent.Response:
        try:
            result = self._execute(request.intent, dry_run=request.dry_run)
            response.result = result
            response.success = result.success
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"execute handler error: {exc}")
            response.success = False
            response.error_message = str(exc)
        return response

    # ------------------------------------------------------------------
    # Pipeline steps
    # ------------------------------------------------------------------

    def _parse(self, text: str, session_id: str) -> HomeIntent:
        normalized = normalize(text)
        intent_type = self._classifier.classify(normalized)
        slots = self._extractor.extract(normalized)

        intent = HomeIntent()
        intent.request_id = str(uuid.uuid4())
        intent.stamp = self.get_clock().now().to_msg()
        intent.raw_text = text
        intent.intent_type = intent_type
        intent.action = slots.action
        intent.target = HomeTarget(
            area=slots.area,
            device_class=slots.device_class,
            name=slots.name,
            entity_id=slots.entity_id,
        )
        intent.parameters = [
            HomeParameter(name=k, value=v) for k, v in slots.parameters.items()
        ]
        intent.confidence = 1.0 if (slots.action and (slots.area or slots.entity_id)) else 0.6
        intent.status = HomeIntent.STATUS_PARSED
        return intent

    def _build_status_result(self, intent: HomeIntent, session_id: str) -> IntentResult:
        result = IntentResult()
        result.request_id = intent.request_id
        result.stamp = self.get_clock().now().to_msg()
        # intent is already in ParseHomeCommand_Response.intent — not duplicated here
        if intent.status == HomeIntent.STATUS_NEEDS_CLARIFICATION:
            result.status = IntentResult.STATUS_NEEDS_CLARIFICATION
            result.spoken_response = self._response.build_clarification_request(
                intent.missing_slot, list(intent.candidates)
            )
        elif intent.status == HomeIntent.STATUS_REJECTED:
            result.status = IntentResult.STATUS_FAILED
            result.spoken_response = self._response.build_error_response("REJECTED")
        elif intent.status == HomeIntent.STATUS_RESOLVED and intent.confirmation_required:
            result.status = IntentResult.STATUS_NEEDS_CONFIRMATION
            warning = ""
            if session_id:
                session = self._context.get_or_create(session_id)
                warning = (session.pending_action or {}).get("warning", "")
            result.spoken_response = self._response.build_confirmation_request(warning)
        else:
            result.spoken_response = self._response.build_unknown_response()
        return result

    def _resolve(self, intent: HomeIntent, session_id: str) -> HomeIntent:
        # Infer device_class from action when the user doesn't specify an object type
        if not intent.target.device_class and intent.action:
            intent.target.device_class = _ACTION_DEFAULT_CLASS.get(intent.action, "")

        candidates = self._resolver.resolve(
            area=intent.target.area,
            device_class=intent.target.device_class,
            name=intent.target.name,
            entity_id=intent.target.entity_id,
        )

        if not candidates:
            intent.status = HomeIntent.STATUS_NEEDS_CLARIFICATION
            intent.missing_slot = "device_class" if not intent.target.device_class else "area"
            return intent

        if len(candidates) > 1 and not intent.target.entity_id:
            intent.status = HomeIntent.STATUS_NEEDS_CLARIFICATION
            intent.missing_slot = "name" if intent.target.area else "area"
            intent.candidates = [c.get("entity_id", c.get("id", "")) for c in candidates[:5]]
            return intent

        obj = candidates[0]
        intent.target.entity_id = obj.get("entity_id", "")
        intent.target.name = obj.get("label", intent.target.name)
        intent.target.device_class = obj.get("device_class", intent.target.device_class)
        intent.target.aliases = obj.get("aliases", [])

        params_dict = {p.name: p.value for p in intent.parameters}
        valid, err = self._validator.validate_action(intent.target.device_class, intent.action)
        if not valid:
            intent.status = HomeIntent.STATUS_REJECTED
            self.get_logger().warn(f"Action rejected: {err}")
            return intent

        risk, warning = self._validator.get_risk(
            intent.target.entity_id,
            intent.target.device_class,
            intent.action,
            params_dict,
        )
        intent.risk_level = risk
        intent.confirmation_required = self._validator.confirmation_required(risk)
        intent.status = HomeIntent.STATUS_RESOLVED

        # Store pending action in dialogue context when confirmation needed
        if intent.confirmation_required and session_id:
            session = self._context.get_or_create(session_id)
            session.waiting_confirmation = True
            session.pending_action = {
                "action": intent.action,
                "entity_id": intent.target.entity_id,
                "device_class": intent.target.device_class,
                "parameters": params_dict,
                "warning": warning,
            }
            self._context.update(session)

        return intent

    def _execute(self, intent: HomeIntent, dry_run: bool) -> IntentResult:
        result = IntentResult()
        result.request_id = intent.request_id
        result.stamp = self.get_clock().now().to_msg()
        result.intent = intent

        params_dict = {p.name: p.value for p in intent.parameters}

        success, spoken, technical = self._executor.execute(
            action=intent.action,
            entity_id=intent.target.entity_id,
            device_class=intent.target.device_class,
            parameters=params_dict,
            dry_run=dry_run,
            label=intent.target.name,
        )

        result.success = success
        result.status = IntentResult.STATUS_SUCCESS if success else IntentResult.STATUS_FAILED
        result.spoken_response = spoken
        result.technical_message = technical
        if intent.target.entity_id:
            result.affected_entities = [intent.target.entity_id]

        return result
