import subprocess
import os, signal
import time
import psutil

def list_children(pid):
    parent = psutil.Process(pid)
    return [p.pid for p in parent.children(recursive=True)]

def kill_process(pid) -> bool:
    try:
        os.killpg(pid, signal.SIGINT)
        time.sleep(2)
        os.killpg(pid, signal.SIGTERM)
        time.sleep(2)
        os.killpg(pid, signal.SIGKILL)
        print(f"Process group {pid} terminated.")
        return True
    except ProcessLookupError:
        print(f"Process {pid} not found.")
        return False
    except PermissionError:
        print(f"Permission denied to terminate {pid}.")
        return False

def run_command(command) -> int:
    try:
        proc = subprocess.Popen(
            command,
            start_new_session=True,      # CREATE process group
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL
        )
        return proc.pid   # group leader PID
    except Exception as e:
        print(f"Error running command: {e}")
        return None


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
    
