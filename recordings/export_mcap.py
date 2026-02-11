from rosbags.rosbag2 import Reader
from rosbags.typesys import get_typestore, Stores
import csv
import os
import numpy as np

# Path to your bag folder
bag_path = '.'  # Current directory, change if needed

# Create output directory
output_dir = 'csv_output'
os.makedirs(output_dir, exist_ok=True)

# Get typestore for Jazzy
typestore = get_typestore(Stores.ROS2_JAZZY)

# Fields to skip
SKIP_FIELDS = {'__msgtype__', 'layout'}

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

# Open the bag
with Reader(bag_path) as reader:
    
    # Process each topic
    for connection in reader.connections:
        topic_name = connection.topic.replace("/", "_").lstrip("_")
        csv_filename = os.path.join(output_dir, f'{topic_name}.csv')
        
        print(f"Processing: {connection.topic} -> {csv_filename}")
        
        with open(csv_filename, 'w', newline='') as csvfile:
            writer = None
            
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
                    
                except Exception as e:
                    print(f"Error processing message in {connection.topic}: {e}")
                    continue
        
        print(f"✓ Completed {connection.topic}")

print(f"\nAll topics exported to {output_dir}/")
