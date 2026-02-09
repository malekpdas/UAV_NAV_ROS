import yaml

def open_yaml_config(path: str, exec_name: str) -> dict:
    with open(path, 'r') as f:
        config = yaml.safe_load(f)
        return config[exec_name]['ros__parameters']

def write_yaml(yaml_data: dict, path: str):
    with open(path, "w") as f:
        yaml.dump(yaml_data, f, default_flow_style=False, sort_keys=False)
