import subprocess
import os, signal
import time
import psutil
from typing import List, Optional

def list_children(pid):
    parent = psutil.Process(pid)
    return [p.pid for p in parent.children(recursive=True)]

def kill_process(pid) -> bool:
    try:
        os.killpg(pid, signal.SIGINT)
        time.sleep(3)
        os.killpg(pid, signal.SIGTERM)
        time.sleep(3)
        os.killpg(pid, signal.SIGKILL)
        print(f"Process group {pid} terminated.")
        return True
    except ProcessLookupError:
        raise Exception(f"Process {pid} not found.")
    except PermissionError:
        raise Exception(f"Permission denied to terminate {pid}.")

def run_command(command, cwd: Optional[str] = None) -> int:
    try:
        proc = subprocess.Popen(
            command,
            cwd=cwd,                     # None = inherit current directory
            start_new_session=True,     # create process group
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL
        )
        return proc.pid
    except Exception as e:
        raise Exception(f"Error running command: {e}")

def record_topics(rec_path: str, topics: List[str]) -> int:
    command = ["ros2", "bag", "record"]
    if len(topics) > 0:
        command += topics
    else:
        command += ["--all-topics"]
    return run_command(command, cwd=rec_path)

def launch_ros2(launch_file) -> int:
    command = ["ros2", "launch", launch_file]
    return run_command(command)

if __name__ == "__main__":
    launch_file = "/home/malekpdas/ros2-ws/src/launch/all_nodes_launch.yaml"
    pid = launch_ros2(launch_file)
    print(f"Launched ROS2 nodes with PID: {pid}")
    children = list_children(pid)
    print(f"Children: {children}")
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("Shutting down...")
        kill_process(pid)
    
