import subprocess
from typing import Optional

def launch_ros_launch(package: str, launch_file: str) -> subprocess.Popen:
    return subprocess.Popen(['ros2', 'launch', package, launch_file])

def stop_process(proc: Optional[subprocess.Popen]):
    if proc and proc.poll() is None:
        proc.terminate()
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            proc.kill()
