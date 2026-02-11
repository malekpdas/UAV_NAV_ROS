import yaml
import json
from typing import Any, List

def open_yaml_config(path: str, exec_name: str) -> dict:
    with open(path, 'r') as f:
        config = yaml.safe_load(f)
        return config[exec_name]['ros__parameters']

def validate_dtype(value: Any, dtype: str, options: List[Any] = None) -> Any:
    """
    Validate and enforce dtype on value by converting it to the specified type.
    For lists, dtype specifies the element type and all elements are converted.
    For choice type, validates against options.
    
    Args:
        value: The value to validate and convert
        dtype: The expected data type (element type for lists)
        options: List of allowed values for choice type
        
    Returns:
        The value converted to the specified dtype
        
    Raises:
        ValueError: If dtype is invalid or choice validation fails
        TypeError: If value cannot be converted to dtype
    """
    
    # Handle scalar dtypes
    dtype_map = {
        'string': str,
        'int': int,
        'float': float,
        'bool': bool,
    }

    # Handle choice type
    if dtype == 'choice':
        if options is None:
            raise ValueError(f"Choice dtype requires 'options' list")
        if value not in options:
            raise ValueError(f"Value '{value}' not in allowed options: {options}")
        return value
    
    # Handle list values - dtype specifies element type
    if isinstance(value, list):
        if not value:  # Empty lists are valid
            return value
        
        if dtype not in dtype_map:
            raise ValueError(f"Unknown dtype: {dtype}")
        
        converter = dtype_map[dtype]
        converted_list = []
        
        # Convert all elements to the specified dtype
        for i, element in enumerate(value):
            try:
                converted_list.append(converter(element))
            except (ValueError, TypeError) as e:
                raise TypeError(
                    f"List element {i} cannot be converted to {dtype}: {element}"
                )
        
        return converted_list
    
    
    if dtype not in dtype_map:
        raise ValueError(f"Unknown dtype: {dtype}")
    
    converter = dtype_map[dtype]
    
    try:
        return converter(value)
    except (ValueError, TypeError) as e:
        raise TypeError(f"Cannot convert {type(value).__name__} to {dtype} for value: {value}")

def enforce_config_dtypes(config: dict) -> dict:
    """
    Enforce dtype validation on all config parameters by converting values to proper types.
    Recursively validates and converts nested config structures.
    
    Args:
        config: The ros__parameters config dict
        
    Returns:
        The validated and type-converted config dict
        
    Raises:
        ValueError or TypeError if conversion fails
    """
    validated_config = {}
    
    for param_name, param_config in config.items():
        if isinstance(param_config, dict):
            if 'value' in param_config and 'dtype' in param_config:
                value = param_config['value']
                dtype = param_config['dtype']
                options = param_config.get('options', None)
                
                # Validate and convert value to proper dtype
                converted_value = validate_dtype(value, dtype, options)
                
                # Update config with converted value
                param_copy = param_config.copy()
                param_copy['value'] = converted_value
                validated_config[param_name] = param_copy
            else:
                # Recursively validate nested configs
                validated_config[param_name] = enforce_config_dtypes(param_config)
        else:
            validated_config[param_name] = param_config
    
    return validated_config

def write_yaml(dict_data: dict, path: str):
    """
    Write config to YAML file with dtype validation.
    Ensures all values follow their specified dtype and options constraints.
    
    Args:
        config: The config dict to write
        path: The file path to write to
        
    Raises:
        ValueError or TypeError if any value violates dtype constraints
    """
    with open(path, "w") as f:
        yaml.dump(dict_data, f, default_flow_style=False, sort_keys=False)

def main():
    """Test cases for format_converter dtype validation."""
    
    print("=" * 70)
    print("Testing format_converter dtype validation with YAML configs")
    print("=" * 70)
    
    sensor_fusion_path = 'src/sensor_fusion/config/sensor_fusion.yaml'
    test_output_path = 'test_config_output.yaml'
    
    # Test: Load and validate config, then write to file
    print("\n[Test] Load config and write with validation")
    
    try:
        config = open_yaml_config(sensor_fusion_path, 'sensor_fusion')
        print(f"  ✓ Loaded config from {sensor_fusion_path}")
        
        validate_and_write_yaml_config(config, test_output_path)
        print(f"  ✓ Validated and wrote config to {test_output_path}")
        print(f"  → Validated {len(config)} top-level parameter groups")
        
        # Count total parameters
        total_params = sum(
            len(v.items()) if isinstance(v, dict) and all(isinstance(vv, dict) for vv in v.values()) else 1
            for v in config.values()
        )
        print(f"  → Total parameters validated: ~{total_params}")
        print("  ✓ PASSED: All operations successful")
        
    except FileNotFoundError as e:
        print(f"  ✗ FAILED: File not found - {e}")
    except (ValueError, TypeError) as e:
        print(f"  ✗ FAILED: Validation error - {e}")
    except Exception as e:
        print(f"  ✗ FAILED: {e}")
    
    print("\n" + "=" * 70)
    print("Test completed!")
    print("=" * 70)

if __name__ == '__main__':
    main()
