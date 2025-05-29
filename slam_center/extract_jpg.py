import sqlite3
import os
import yaml
import base64
import struct
from pathlib import Path

# Set these to your bag directory and the output directory for images
BAG_DIR = f"{Path.home()}/ROS2/ros2_rtp_ws/src/test_zmq/rosbag/rosbag2_2025_05_26-23_56_14"  # Directory containing metadata.yaml and .db3
OUT_DIR = os.path.join(BAG_DIR, "extracted_images")
os.makedirs(OUT_DIR, exist_ok=True)

def align4(offset):
    return (offset + 3) & ~0x03

def parse_cdr_string(buf, offset):
    strlen = struct.unpack_from('<I', buf, offset)[0]
    offset += 4
    string = buf[offset:offset+strlen].decode('utf-8', errors='replace')
    offset += strlen
    offset = align4(offset)
    return string, offset

def parse_cdr_compressed_image(cdr):
    # --- Skip the CDR encapsulation header! ---
    off = 4  # Skip the first 4 bytes (CDR encapsulation)
    # Header: sec, nanosec, frame_id
    sec = struct.unpack_from('<I', cdr, off)[0]; off += 4
    nanosec = struct.unpack_from('<I', cdr, off)[0]; off += 4
    frame_id, off = parse_cdr_string(cdr, off)
    # format string
    fmt, off = parse_cdr_string(cdr, off)
    # data field (uint8[])
    data_len = struct.unpack_from('<I', cdr, off)[0]
    off += 4
    img_bytes = cdr[off:off+data_len]
    return {
        "sec": sec,
        "nanosec": nanosec,
        "frame_id": frame_id,
        "format": fmt,
        "data": img_bytes
    }

with open(os.path.join(BAG_DIR, "metadata.yaml"), "r") as f:
    metadata = yaml.safe_load(f)
db3_file = os.path.join(BAG_DIR, metadata['rosbag2_bagfile_information']['relative_file_paths'][0])
topic_name = None
for topic in metadata['rosbag2_bagfile_information']['topics_with_message_count']:
    if topic['topic_metadata']['type'] == "sensor_msgs/msg/CompressedImage":
        topic_name = topic['topic_metadata']['name']
        break
if topic_name is None:
    raise Exception("No CompressedImage topic found in metadata.")
conn = sqlite3.connect(db3_file)
cursor = conn.cursor()
cursor.execute("SELECT id FROM topics WHERE name=?", (topic_name,))
row = cursor.fetchone()
if row is None:
    raise Exception(f"Topic {topic_name} not found in database.")
topic_id = row[0]
cursor.execute("SELECT timestamp, data FROM messages WHERE topic_id=? ORDER BY timestamp", (topic_id,))
rows = cursor.fetchall()

for i, (timestamp, cdr_bytes) in enumerate(rows):
    try:
        msg = parse_cdr_compressed_image(cdr_bytes)
        fmt = msg['format'].lower()
        if fmt not in ["jpg", "jpeg", "png"]:
            fmt = "jpg"
        fname = os.path.join(OUT_DIR, f"frame_{i:04d}.{fmt}")
        with open(fname, "wb") as f:
            f.write(msg['data'])
        print(f"Wrote {fname}")
    except Exception as e:
        print(f"Error parsing message {i} (timestamp {timestamp}): {e}")

print("Done. Images written to", OUT_DIR)