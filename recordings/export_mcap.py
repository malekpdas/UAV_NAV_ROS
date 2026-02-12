from rosbags.rosbag2 import Reader
from rosbags.typesys import get_typestore, Stores, get_types_from_msg
import csv
import os
import argparse
import numpy as np

# Fields to skip
SKIP_FIELDS = {'__msgtype__', 'layout'}

# Custom message definitions
CUSTOM_MSGS = {
    'custom_interfaces/msg/FlightMode': """
std_msgs/Header header
string mode
""",
    'custom_interfaces/msg/RcChannels': """
std_msgs/Header header
int32[] channels
"""
}

def register_custom_messages(typestore):
    """Register custom message definitions with the typestore."""
    for msg_type, msg_def in CUSTOM_MSGS.items():
        try:
            # Parse and register the custom message
            types = get_types_from_msg(msg_def, msg_type)
            for name, cls in types.items():
                typestore.register(types)
            print(f"✓ Registered custom message: {msg_type}")
        except Exception as e:
            print(f"⚠ Failed to register {msg_type}: {e}")

def flatten_msg(msg, parent_key='', sep='.'):
    """Recursively flatten a ROS message into a flat dictionary."""
    items = []
    
    if hasattr(msg, '__annotations__'):
        # It's a ROS message object
        for field in msg.__annotations__:
            # Skip metadata and layout fields
            if field in SKIP_FIELDS:
                continue
                
            value = getattr(msg, field)
            new_key = f"{parent_key}{sep}{field}" if parent_key else field
            
            if hasattr(value, '__annotations__'):
                # Nested message
                items.extend(flatten_msg(value, new_key, sep=sep).items())
            elif isinstance(value, (list, tuple, np.ndarray)):
                # Array - keep as comma-separated string
                arr = list(value)
                # Check if it's an array of messages or primitives
                if arr and hasattr(arr[0], '__annotations__'):
                    # Array of messages - stringify each
                    items.append((new_key, str(arr)))
                else:
                    # Array of primitives - convert to space/comma separated
                    items.append((new_key, ' '.join(map(str, arr))))
            else:
                # Primitive value
                items.append((new_key, value))
    else:
        # It's already a primitive
        items.append((parent_key, msg))
    
    return dict(items)

def process_rosbag(bag_path, output_dir=None):
    """Process a ROS2 bag file and export topics to CSV files."""
    
    # Default output directory to bag_path/csv if not specified
    if output_dir is None:
        output_dir = os.path.join(bag_path, 'csv')
    
    # Create output directory
    os.makedirs(output_dir, exist_ok=True)
    
    # Get typestore for Jazzy
    typestore = get_typestore(Stores.ROS2_JAZZY)
    
    # Register custom messages
    register_custom_messages(typestore)
    
    # Track errors per topic to avoid spam
    topic_errors = {}
    
    # Open the bag
    with Reader(bag_path) as reader:
        
        # Process each topic
        for connection in reader.connections:
            topic_name = connection.topic.replace("/", "_").lstrip("_")
            csv_filename = os.path.join(output_dir, f'{topic_name}.csv')
            
            print(f"Processing: {connection.topic} -> {csv_filename}")
            
            # Track errors for this specific topic
            error_count = 0
            error_printed = False
            
            with open(csv_filename, 'w', newline='') as csvfile:
                writer = None
                message_count = 0
                
                # Read all messages for this topic
                for conn, timestamp, rawdata in reader.messages(connections=[connection]):
                    try:
                        # Deserialize message
                        msg = typestore.deserialize_cdr(rawdata, connection.msgtype)
                        
                        # Flatten the message
                        msg_dict = {'timestamp_ns': timestamp}
                        msg_dict.update(flatten_msg(msg))
                        
                        # Initialize writer with headers on first message
                        if writer is None:
                            writer = csv.DictWriter(csvfile, fieldnames=msg_dict.keys())
                            writer.writeheader()
                        
                        writer.writerow(msg_dict)
                        message_count += 1
                        
                    except Exception as e:
                        error_count += 1
                        # Only print the error once per topic
                        if not error_printed:
                            error_msg = str(e)
                            print(f"⚠ Error in {connection.topic}: {error_msg}")
                            
                            # Suggest loading custom messages
                            if "custom_interfaces" in connection.msgtype or "/" in connection.msgtype:
                                print(f"  → Custom message type detected: {connection.msgtype}")
                                print(f"  → You may need to add it to CUSTOM_MSGS dictionary")
                            
                            error_printed = True
                        continue
            
            # Summary for this topic
            if error_count > 0:
                print(f"✓ Completed {connection.topic} ({message_count} messages, {error_count} errors)")
            else:
                print(f"✓ Completed {connection.topic} ({message_count} messages)")
    
    print(f"\nAll topics exported to {output_dir}/")

def main():
    """Main entry point for the script."""
    parser = argparse.ArgumentParser(
        description='Convert ROS2 bag topics to CSV files.',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    parser.add_argument(
        'bag_path',
        help='Path to the ROS2 bag directory'
    )
    parser.add_argument(
        '-o', '--output',
        default=None,
        help='Output directory for CSV files (default: <bag_path>/csv)'
    )
    
    args = parser.parse_args()
    
    # Validate bag path exists
    if not os.path.exists(args.bag_path):
        print(f"Error: Bag path '{args.bag_path}' does not exist")
        return 1
    
    try:
        process_rosbag(args.bag_path, args.output)
        return 0
    except Exception as e:
        print(f"Error processing bag: {e}")
        return 1

if __name__ == '__main__':
    exit(main())