
from rosbags.rosbag1 import Reader
from rosbags.serde import deserialize_cdr, ros1_to_cdr
import csv
import os
from datetime import datetime
from collections import defaultdict
import numpy as np

def bag_to_csv(bag_file, output_dir, topics_and_fields):
    os.makedirs(output_dir, exist_ok=True)
    
    combined_csv_path = os.path.join(output_dir, 'combined_data.csv')
    with open(combined_csv_path, 'w') as combined_file:
        combined_writer = csv.writer(combined_file)
        
        headers = ['timestamp']
        for topic in topics_and_fields:
            for field in topics_and_fields[topic]:
                headers.append(f"{topic.replace('/', '_')}_{field.replace('.', '_')}")
        combined_writer.writerow(headers)
        
        with Reader(bag_file) as reader:
            print("\nAvailable topics in the bag:")
            for connection in reader.connections:
                print(f"- {connection.topic} (type: {connection.msgtype})")
            print()

            data_dict = defaultdict(dict)
            
            for connection, timestamp, rawdata in reader.messages():
                if connection.topic in topics_and_fields:
                    try:
                        cdr_data = ros1_to_cdr(rawdata, connection.msgtype)
                        msg = deserialize_cdr(cdr_data, connection.msgtype)
                        
                        for field in topics_and_fields[connection.topic]:
                            value = msg
                            try:
                                for part in field.split('.'):
                                    if hasattr(value, part):
                                        value = getattr(value, part)
                                    else:
                                        value = value[part]
                                field_name = f"{connection.topic.replace('/', '_')}_{field.replace('.', '_')}"
                                data_dict[timestamp][field_name] = str(value)
                            except Exception as e:
                                print(f"Field access error on {connection.topic}.{field}: {e}")
                                field_name = f"{connection.topic.replace('/', '_')}_{field.replace('.', '_')}"
                                data_dict[timestamp][field_name] = np.nan
                                
                    except Exception as e:
                        print(f"\nError processing message on {connection.topic}: {str(e)}")
                        print(f"Message type: {connection.msgtype}")
                        print("Skipping this message...\n")

            # Write all data sorted by timestamp
            for timestamp in sorted(data_dict.keys()):
                row = [timestamp]
                for field_name in headers[1:]:  # Skip timestamp column
                    row.append(data_dict[timestamp].get(field_name, np.nan))
                combined_writer.writerow(row)

    print(f"\nCombined data written to {combined_csv_path}")

if __name__ == '__main__':
    topics_and_fields = {
        '/ekf/odometry/filtered': [
            'twist.twist.linear.x'
        ],
        '/OF/odom': [
            'twist.twist.linear.x',
            'twist.twist.linear.y'
        ]
    }
    
    bag_file = ... # TODO: add bag file path
    output_dir = ... # TODO: add output csv file path
    
    print(f"\nStarting conversion of {bag_file}")
    bag_to_csv(bag_file, output_dir, topics_and_fields)
    print(f"\nConversion complete. Files saved to {output_dir}")