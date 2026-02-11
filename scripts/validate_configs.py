import os
import yaml
import sys

def get_yaml_content(file_path):
    try:
        with open(file_path, 'r') as f:
            return yaml.safe_load(f)
    except Exception as e:
        return None

def find_parameters(data, path=""):
    """
    Recursively find parameters in the YAML structure.
    A parameter is identified by having a 'value' key.
    """
    parameters = []
    if isinstance(data, dict):
        if 'value' in data:
            parameters.append((path, data))
        else:
            for key, value in data.items():
                new_path = f"{path}.{key}" if path else key
                parameters.extend(find_parameters(value, new_path))
    return parameters

def validate_config(file_path):
    content = get_yaml_content(file_path)
    if content is None:
        return ["Could not parse YAML file"]
    
    problems = []
    
    # Check each top-level node (usually the node name)
    for node_name, node_data in content.items():
        if not isinstance(node_data, dict) or 'ros__parameters' not in node_data:
            # Skip nodes that don't have ros__parameters (might be global configs)
            continue
            
        params_root = node_data['ros__parameters']
        if not params_root:
            problems.append(f"Node '{node_name}' has empty 'ros__parameters'")
            continue
            
        params = find_parameters(params_root, node_name)
        
        for p_path, p_data in params:
            # Check for description or desc
            if 'description' not in p_data and 'desc' not in p_data:
                problems.append(f"Parameter '{p_path}' is missing 'description' or 'desc'")
            
            # Check for dtype
            if 'dtype' not in p_data:
                problems.append(f"Parameter '{p_path}' is missing 'dtype'")
                
    return problems

def main():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    src_dir = os.path.abspath(os.path.join(script_dir, '..', 'src'))
    
    if not os.path.exists(src_dir):
        print(f"Error: src directory not found at {src_dir}")
        sys.exit(1)
        
    config_files_with_problems = []
    
    for root, dirs, files in os.walk(src_dir):
        if 'config' in dirs:
            config_dir = os.path.join(root, 'config')
            for f in os.listdir(config_dir):
                if f.endswith('.yaml'):
                    file_path = os.path.join(config_dir, f)
                    problems = validate_config(file_path)
                    if problems:
                        rel_path = os.path.relpath(file_path, src_dir)
                        config_files_with_problems.append((rel_path, problems))
                        
    if not config_files_with_problems:
        print("\n" + "="*50)
        print("CONFIG VALIDATION: ALL FILES PASSED")
        print("="*50 + "\n")
    else:
        print("\n" + "="*50)
        print("CONFIG VALIDATION REPORT: PROBLEMS FOUND")
        print("="*50)
        for rel_path, problems in sorted(config_files_with_problems):
            print(f"\n[File: {rel_path}]")
            for prob in problems:
                print(f"  - ERROR: {prob}")
        print("\n" + "="*50 + "\n")

if __name__ == "__main__":
    main()
