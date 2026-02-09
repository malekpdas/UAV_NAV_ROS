from fastapi import APIRouter, HTTPException, Request, status
from fastapi_bridge.helpers.terminal_cmd import launch_ros2, kill_process
from fastapi_bridge.helpers.format_converter import write_yaml
from typing import List, Dict, Any
from pydantic import BaseModel
from pathlib import Path

router = APIRouter(prefix="/launch", tags=["launch"])

@router.post("/start", status_code=status.HTTP_200_OK)
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

@router.post("/stop", status_code=status.HTTP_200_OK)
def stop_nodes(request: Request):
    try:
        pid = request.app.state.launch_pid

        if not pid:
            raise HTTPException(status_code=status.HTTP_404_NOT_FOUND, detail="No launch running")

        is_ok = kill_process(pid)

        if is_ok:
            request.app.state.launch_pid = None
            return {"msg": "OK"}
        
        raise HTTPException(status_code=status.HTTP_400_BAD_REQUEST, detail="Unknown command")

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

def to_ros2_yaml(body: LaunchFileBody, launch_dir: str) -> dict:
    return {
        "launch": [
            {
                "node": {
                    "pkg": e.pkg,
                    "exec": e.exec,
                    "name": e.id.replace(" ", "_"),
                    "output": "log",
                    "param": [
                        {"from": f"{launch_dir}/config/{e.id.replace(" ", "_")}.yaml"}
                    ]
                }
            }
            for e in body.launch
        ]
    }

@router.post("/create", status_code=status.HTTP_200_OK)
def create_launch_file(request: Request, data: LaunchFileBody):
    try:
        launch_dir = request.app.state.launch_dir
        Path(launch_dir).mkdir(parents=True, exist_ok=True)

        launch_yaml = to_ros2_yaml(data, launch_dir)

        launch_file_path = f"{launch_dir}/launch.yaml"
        write_yaml(launch_yaml, launch_file_path)
        return {"created": launch_file_path, "msg": "OK"}
    except Exception as e:
        raise HTTPException(status_code=status.HTTP_500_INTERNAL_SERVER_ERROR, detail=str(e))
        
