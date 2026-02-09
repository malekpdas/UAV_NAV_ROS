from fastapi import APIRouter, HTTPException, Request, status
from fastapi_bridge.helpers.terminal_cmd import launch_ros2, kill_process
from fastapi_bridge.helpers.format_converter import write_yaml, open_yaml_config
from typing import List, Dict, Any
from pydantic import BaseModel
from pathlib import Path

router = APIRouter(prefix="/config", tags=["config"])

class ConfigReadBody(BaseModel):
    id: str                  # node name
    exec: str                  # node name
    pkg: str                   # package name
    default: bool              # default config

@router.post("/read", status_code=status.HTTP_200_OK)
def read_config_file(request: Request, data: ConfigReadBody):

    try:
        exec_name = data.exec
        pkg_name = data.pkg
        node_id = data.id
        default = data.default

        config_path = f"{request.app.state.launch_dir}/config/{node_id}.yaml"
        default_config_path = f"{request.app.state.ws_root}/src/{pkg_name}/config/{exec_name}.yaml"

        default = not Path(config_path).exists() or default

        if not default:
            config = open_yaml_config(config_path, node_id)
            return {"config": config, "msg": "OK", "default": False}
        else:
            config = open_yaml_config(default_config_path, exec_name)
            return {"config": config, "msg": "OK", "default": True}

    except Exception as e:
        raise HTTPException(status_code=status.HTTP_500_INTERNAL_SERVER_ERROR, detail=str(e))


class ConfigFileBody(BaseModel):
    id: str                  # node name
    config: Dict[str, Any]

def to_ros2_config(data: ConfigFileBody) -> dict:
    return {
        data.id: {
            "ros__parameters": data.config
        }
    }

@router.post("/create", status_code=status.HTTP_200_OK)
def create_config_file(request: Request, data: ConfigFileBody):
    try:
        config_dir = f"{request.app.state.launch_dir}/config"
        Path(config_dir).mkdir(parents=True, exist_ok=True)

        name = data.id.replace(" ", "_")

        config_yaml = to_ros2_config(data)
        
        config_file_path = f"{config_dir}/{name}.yaml"
        write_yaml(config_yaml, config_file_path)

        return {"created": config_file_path, "msg": "OK"}

    except Exception as e:
        raise HTTPException(status_code=status.HTTP_500_INTERNAL_SERVER_ERROR, detail=str(e))
        
