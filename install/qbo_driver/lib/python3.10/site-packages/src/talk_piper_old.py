#!/usr/bin/env python3
"""
ROS2 version of the original ROS1 `piper2wave_node`.

Features:
- Keeps the same service interfaces (`qbo_msgs/srv/Text2Speach`).
- Talks to the local Piper HTTP server (POST /tts) exactly as before.
- Supports dynamic language change through a parameter and a topic (`/system_lang`).
- Clean shutdown and async-friendly (spins in a separate executor).

Assumptions:
- The custom service `Text2Speach.srv` has the same signature in ROS2
  (string sentence -> bool success) and is built in an `interface` package
  that is part of your workspace.
- `qbo_msgs` is the ROS2 package that exports this service.

To build:
  colcon build --packages-select qbo_driver

Run:
  ros2 run qbo_driver qbo_talk --ros-args -p audio_device_out_name:="Jabra"
"""

from __future__ import annotations

import os
import requests
import sounddevice as sd
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import String
from qbo_msgs.srv import Text2Speach


PIPER_ENDPOINT = os.getenv("PIPER_ENDPOINT", "http://localhost:8080/tts")


def _post_to_piper(text: str) -> bool:
    """Send text to Piper HTTP endpoint, return True on 200."""
    try:
        resp = requests.post(PIPER_ENDPOINT, json={"text": text})
        resp.raise_for_status()
        return True
    except requests.RequestException as exc:
        print(f"[ERROR] Piper request failed: {exc}")
        return False


class Piper2WaveNode(Node):
    """ROS2 Node wrapping Piper TTS and exposing legacy services."""

    LANGUAGES_VOICES = {
        "en": "en-GB",
        "es": "es-ES",
        "fr": "fr-FR",
    }

    def __init__(self) -> None:
        super().__init__("talk_pico2wave")

        # Declare and get the current system language parameter
        self.declare_parameter("audio_out_device_name", "usb")
        self.declare_parameter("system_lang", "fr")
        self.language: str = self.get_parameter("system_lang").get_parameter_value().string_value
        self.set_language(self.language)

        # Services
        self.create_service(Text2Speach, "/qbo_driver/piper2wave_say", self.say_callback)
        self.create_service(Text2Speach, "/qbo_driver/set_language", self.set_language_callback)

        # Topic subscriber for dynamic language switch
        self.create_subscription(String, "/system_lang", self.system_lang_callback, 10)

        self.get_logger().info(f"Piper2WaveNode ready - language set to '{self.language}'.")

    # ---------------------------------------------------------------------
    # Callbacks
    # ---------------------------------------------------------------------

    def say_callback(self, request: Text2Speach.Request, response: Text2Speach.Response):
        text = request.sentence.replace("\"", "\\\"")
        self.get_logger().info(f"[TTS] {text}")
        response.success = _post_to_piper(text)
        return response

    def set_language_callback(self, request: Text2Speach.Request, response: Text2Speach.Response):
        response.success = self.set_language(request.sentence)
        return response

    def system_lang_callback(self, msg: String):
        self.get_logger().info(f"/system_lang changed → {msg.data}")
        self.set_language(msg.data)

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def set_language(self, lang: str) -> bool:
        if lang in self.LANGUAGES_VOICES:
            self.language = lang
            voice = self.LANGUAGES_VOICES[lang]
            self.get_logger().info(f"Voice set to {voice} ({lang})")
            # You could add logic here to tell Piper which model to use per‑lang.
            return True
        else:
            self.get_logger().warning(f"Unknown language '{lang}'. Voice unchanged.")
            return False


# ---------------------------------------------------------------------------
# Main entry point
# ---------------------------------------------------------------------------

def main():
    rclpy.init()
    node = Piper2WaveNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
