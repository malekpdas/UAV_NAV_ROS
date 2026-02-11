from fastapi import APIRouter, HTTPException, Request, status
from fastapi_bridge.helpers.terminal_cmd import launch_ros2, record_topics, kill_process
from fastapi_bridge.helpers.format_converter import write_yaml
from typing import List, Dict, Any
from pydantic import BaseModel
from pathlib import Path
import shutil

router = APIRouter(prefix="/launch", tags=["launch"])

@router.get("/start", status_code=status.HTTP_200_OK)
def launch_nodes(request: Request):
    try:
        launch_file_path = f"{request.app.state.launch_dir}/launch.yaml"
            
        pid = launch_ros2(launch_file_path)

        if pid is None:
            raise HTTPException(status_code=status.HTTP_500_INTERNAL_SERVER_ERROR, detail="Error launching nodes.")

        request.app.state.launch_pid = pid
        return {"msg": "OK", "pid": pid}
        
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=status.HTTP_500_INTERNAL_SERVER_ERROR, detail=str(e))

@router.get("/stop", status_code=status.HTTP_200_OK)
def stop_nodes(request: Request):
    try:
        try:
            launch_pid = request.app.state.launch_pid
            launch_killed = kill_process(launch_pid)
        except:
            launch_killed = False

        try:
            recording_pid = request.app.state.recording_pid
            recording_killed = kill_process(recording_pid)
        except:
            recording_killed = False

        if launch_killed and recording_killed:
            request.app.state.launch_pid = None
            request.app.state.recording_pid = None
            
        return {"msg": "OK", "launch_killed": launch_killed, "recording_killed": recording_killed}
        
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=status.HTTP_500_INTERNAL_SERVER_ERROR, detail=str(e))

class LaunchEntry(BaseModel):
    pkg: str
    exec: str
    id: str

class LaunchFileBody(BaseModel):
    launch: List[LaunchEntry]

def to_ros2_yaml(body: LaunchFileBody, launch_dir: str, ws_dir: str) -> dict:
    launch_dict = {
        "launch": []
    }
    for e in body.launch:
        config_path = f"{launch_dir}/config/{e.id.replace(" ", "_")}.yaml"
        if not Path(config_path).exists():
            config_path = f"{ws_dir}/{e.pkg}/config/{e.exec}.yaml"
        launch_dict["launch"].append({
            "node": {
                "pkg": e.pkg,
                "exec": e.exec,
                "name": e.id.replace(" ", "_"),
                "output": "log",
                "param": [
                    {"from": config_path}
                ]
            }
        })
    return launch_dict

@router.post("/create", status_code=status.HTTP_200_OK)
def create_launch_file(request: Request, data: LaunchFileBody):
    try:
        ws_dir = request.app.state.ws_dir
        launch_dir = request.app.state.launch_dir
        Path(launch_dir).mkdir(parents=True, exist_ok=True)

        launch_yaml = to_ros2_yaml(data, launch_dir, ws_dir)

        launch_file_path = f"{launch_dir}/launch.yaml"
        write_yaml(launch_yaml, launch_file_path)
        return {"created": launch_file_path, "msg": "OK"}
    except Exception as e:
        raise HTTPException(status_code=status.HTTP_500_INTERNAL_SERVER_ERROR, detail=str(e))
        
class RecordingData(BaseModel):
    topics: List[str]

@router.post("/start_recording", status_code=status.HTTP_200_OK)
def start_recording(request: Request, data: RecordingData):
    try:

        rec_dir = request.app.state.rec_dir
        Path(rec_dir).mkdir(parents=True, exist_ok=True)

        pid = record_topics(rec_dir, data.topics)

        if pid is None:
            raise HTTPException(status_code=status.HTTP_500_INTERNAL_SERVER_ERROR, detail="Error recording topics.")

        request.app.state.recording_pid = pid
        return {"msg": "OK", "pid": pid}
        
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=status.HTTP_500_INTERNAL_SERVER_ERROR, detail=str(e))