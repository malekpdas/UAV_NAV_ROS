import os
import yaml
import json

def bundle_node_descriptions(base_dir):
    """
    Scans base_dir for 'descriptor' folders, parses YAML files, and returns the bundle.
    """
    bundle = []
    
    # Path to the 'src' directory relative to the workspace root
    src_dir = os.path.join(base_dir, 'src')
    
    if not os.path.exists(src_dir):
        print(f"Error: Could not find 'src' directory at {src_dir}")
        return bundle

    for root, dirs, files in os.walk(src_dir):
        if 'descriptor' in dirs:
            descriptor_path = os.path.join(root, 'descriptor')
            
            for file in os.listdir(descriptor_path):
                if file.endswith('.yaml') or file.endswith('.yml'):
                    file_path = os.path.join(descriptor_path, file)
                    with open(file_path, 'r') as f:
                        data = yaml.safe_load(f)
                        if data and 'node_info' in data:
                            # Add the default config path based on user's rule
                            pkg_name_on_disk = os.path.basename(root)
                            node_name = data['node_info'].get('executable_name') or data['node_info'].get('node_default_id', 'unknown')
                            config_path = os.path.join(base_dir, "src", pkg_name_on_disk, "config", f"{node_name}.yaml")
                            
                            data['node_info']['default_config'] = config_path
                            bundle.append(data['node_info'])

    return bundle

if __name__ == "__main__":
    # Test call to print results
    ws_root = "/home/malekpdas/ros2-ws"
    bundle = bundle_node_descriptions(ws_root)
    print(f"\nBundled {len(bundle)} node(s):")
    print(json.dumps(bundle, indent=4))
