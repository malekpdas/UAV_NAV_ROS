import os
import yaml
import sys

def get_yaml_content(file_path):
    try:
        with open(file_path, 'r') as f:
            return yaml.safe_load(f)
    except Exception as e:
        return None

def verify_nodes(src_dir):
    report = []
    
    # Traverse src directory to find all 'descriptor' folders
    for root, dirs, files in os.walk(src_dir):
        if 'descriptor' in dirs:
            pkg_path = root
            pkg_name = os.path.basename(pkg_path)
            descriptor_dir = os.path.join(pkg_path, 'descriptor')
            
            # For each yaml file in the descriptor directory
            for descriptor_file in os.listdir(descriptor_dir):
                if not descriptor_file.endswith('.yaml'):
                    continue
                
                node_name = descriptor_file.replace('.yaml', '')
                problems = []
                
                # paths
                descriptor_path = os.path.join(descriptor_dir, descriptor_file)
                config_path = os.path.join(pkg_path, 'config', f'{node_name}.yaml')
                launch_yaml_path = os.path.join(pkg_path, 'launch', f'{node_name}.launch.yaml')
                launch_py_path = os.path.join(pkg_path, 'launch', f'{node_name}.launch.py')
                
                # 1. Verify Descriptor Content
                desc_content = get_yaml_content(descriptor_path)
                if desc_content:
                    node_info = desc_content.get('node_info', {})
                    if node_info.get('pkg_name') != pkg_name:
                        problems.append(f"Descriptor pkg_name '{node_info.get('pkg_name')}' does not match directory pkg_name '{pkg_name}'")
                    if node_info.get('executable_name') != node_name:
                        problems.append(f"Descriptor executable_name '{node_info.get('executable_name')}' does not match file name '{node_name}'")
                    
                    # Check default_config_file_path if it exists
                    default_config = node_info.get('default_config_file_path', '')
                    if default_config and default_config != f"config/{node_name}.yaml":
                        problems.append(f"Descriptor default_config_file_path '{default_config}' should be 'config/{node_name}.yaml'")
                else:
                    problems.append(f"Could not parse descriptor file: {descriptor_file}")

                # 2. Verify Config File
                if not os.path.exists(config_path):
                    problems.append(f"Config file missing: config/{node_name}.yaml")
                else:
                    config_content = get_yaml_content(config_path)
                    if config_content:
                        if node_name not in config_content:
                            problems.append(f"Config file top-level key must be '{node_name}'")
                    else:
                        problems.append(f"Could not parse config file: config/{node_name}.yaml")

                # 3. Verify Launch File
                launch_exists = os.path.exists(launch_yaml_path) or os.path.exists(launch_py_path)
                if not launch_exists:
                    problems.append(f"Launch file missing: launch/{node_name}.launch.yaml or .py")
                
                if os.path.exists(launch_yaml_path):
                    launch_content = get_yaml_content(launch_yaml_path)
                    if launch_content:
                        launch_items = launch_content.get('launch', [])
                        node_found = False
                        for item in launch_items:
                            if 'node' in item:
                                node_info = item['node']
                                l_pkg = node_info.get('pkg')
                                l_exec = node_info.get('exec')
                                l_name = node_info.get('name')
                                
                                if l_pkg == pkg_name and l_exec == node_name:
                                    node_found = True
                                    if l_name != node_name:
                                        problems.append(f"Launch file node name '{l_name}' should be '{node_name}'")
                                    
                                    # Check params (very basic check)
                                    params = node_info.get('param', [])
                                    found_param = False
                                    expected_param = f"$(find-pkg-share {pkg_name})/config/{node_name}.yaml"
                                    for p in params:
                                        if isinstance(p, dict) and p.get('from') == expected_param:
                                            found_param = True
                                            break
                                    if not found_param:
                                        problems.append(f"Launch file missing parameter from: {expected_param}")
                        
                        if not node_found:
                             problems.append(f"Launch file does not define a node for {node_name}")
                    else:
                        problems.append(f"Could not parse launch file: launch/{node_name}.launch.yaml")

                report.append({
                    'package': pkg_name,
                    'node': node_name,
                    'problems': problems
                })
    
    return report

def print_report(report):
    print("\n" + "="*50)
    print("ROS2 NODE STRUCTURE VERIFICATION REPORT")
    print("="*50)
    
    all_ok = True
    sorted_report = sorted(report, key=lambda x: (x['package'], x['node']))
    
    current_pkg = ""
    for item in sorted_report:
        if item['package'] != current_pkg:
            current_pkg = item['package']
            print(f"\n[Package: {current_pkg}]")
        
        status = "PASSED" if not item['problems'] else "FAILED"
        print(f"  - Node: {item['node']:<30} [{status}]")
        
        if item['problems']:
            all_ok = False
            for prob in item['problems']:
                print(f"      ERROR: {prob}")
                
    print("\n" + "="*50)
    if all_ok:
        print("OVERALL STATUS: ALL NODES CONSISTENT")
    else:
        print("OVERALL STATUS: INCONSISTENCIES FOUND")
    print("="*50 + "\n")

if __name__ == "__main__":
    src_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'src'))
    if not os.path.exists(src_path):
        print(f"Error: src directory not found at {src_path}")
        sys.exit(1)
        
    node_report = verify_nodes(src_path)
    print_report(node_report)
