import json
import glob
import os
import h5py
import numpy as np
from pathlib import Path
from typing import List, Optional, Callable, Union

class H5Wrapper(object):
    def __init__(self, data_fpath, mode="w") -> None:
        """
        Initialize H5Wrapper
        
        Args:
            data_fpath: Path to HDF5 file
            mode: File mode - "w" (write/overwrite), "a" (append), "r+" (read-write)
        """
        self.data_fpath = data_fpath
        self.mode = mode
        # Create parent directory if it doesn't exist
        os.makedirs(os.path.dirname(data_fpath) if os.path.dirname(data_fpath) else ".", exist_ok=True)
        self.h5f = h5py.File(data_fpath, mode)
        self.image_cache = {}

    # linear apply for forced stream
    def add_value(self, key, value, type):
        group = "/"
        if key[0] == "/":
            key = key[1:]  # remove first / from key
        if key in self.h5f[group]:
            self.h5f[group][key].resize(
                    self.h5f[group][key].shape[0] + 1, axis=0)
            self.h5f[group][key][-1] = value
        else:
            # print("create_dataset(value):", key)
            self.h5f.create_dataset(
                f"{group}/{key}",
                data=value,
                shape=(1,),
                maxshape=(None,), dtype = type)

    def add_stream(self, key, value, type, group="/"):
        if key[0] == "/":
            key = key[1:]  # remove first / from key
        if key in self.h5f[group]:
            current_size= self.h5f[group][key].shape[0]
            self.h5f[group][key].resize(
                    self.h5f[group][key].shape[0] + len(value), axis=0)
            self.h5f[group][key][current_size:] = value
        else:
            self.h5f.create_dataset(
                f"{group}/{key}",
                data=value,
                shape=(len(value),),
                maxshape=(None,), dtype = type
            )

    def add_stream_fixedlen(self, key, value, group = '/'):
        assert key in self.h5f[group]
        self.h5f[group][key].resize(
            (self.h5f[group][key].shape[0] + 1), axis=0)
        data = np.zeros(self.h5f[group][key].shape[1])
        if len(value) > 0:
            data[:len(value)] = value
        self.h5f[group][key][-1] = data

    def add_object(self, key, value, type, group = '/'):
        if key in self.h5f[group]:
            self.h5f[group][key].resize(
                (self.h5f[group][key].shape[0] + 1), axis=0)
            self.h5f[group][key][-1] = value
        else:
            # print("create_dataset(object):", key, np.array(value).shape,)
            self.h5f.create_dataset(
                f"{group}/{key}",
                data=value,
                shape=(1,) + np.array(value).shape,
                maxshape=(None,) + np.array(value).shape,
                dtype=type
            )

    def add_dict(self, data_dict, group="/"):
        for key, value in data_dict.items():
            self.add_object(key, value, group)
    
    def close(self):
        """Close the HDF5 file"""
        if self.h5f:
            self.h5f.close()
    
    def __enter__(self):
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()

def get_gesture_number(filename):
    """Extract gesture number from filename, handling both 'Gestrue' and 'Gesture' prefixes"""
    basename = os.path.basename(filename)
    # Handle both 'Gestrue' and 'Gesture' prefixes
    if 'Gestrue_' in basename:
        num = basename.split('Gestrue_')[1].split('_')[0]
    elif 'Gesture_' in basename:
        num = basename.split('Gesture_')[1].split('_')[0]
    else:
        return float('inf')  # Put unknown formats at the end
    try:
        return int(num)
    except ValueError:
        return float('inf')  # Put non-numeric at the end

def get_file_mtime(filename):
    """Get file modification time for sorting"""
    return os.path.getmtime(filename)

def get_last_stamp(h5_file):
    """Get the last timestamp from existing HDF5 file"""
    if not os.path.exists(h5_file):
        return 0.0
    try:
        with h5py.File(h5_file, 'r') as f:
            if 'stamp' in f:
                stamps = f['stamp'][:]
                return float(stamps[-1]) if len(stamps) > 0 else 0.0
    except Exception as e:
        print(f"Warning: Could not read last stamp from {h5_file}: {e}")
    return 0.0

def convert_time_unit(value: float, unit: str) -> float:
    """
    Convert time value to seconds based on unit
    
    Args:
        value: Time value
        unit: Time unit ('s', 'ms', 'us', 'ns')
    
    Returns:
        Time value in seconds
    """
    unit_map = {
        's': 1.0,
        'ms': 1e-3,
        'us': 1e-6,
        'ns': 1e-9,
        'sec': 1.0,
        'second': 1.0,
        'seconds': 1.0,
        'millisecond': 1e-3,
        'milliseconds': 1e-3,
        'microsecond': 1e-6,
        'microseconds': 1e-6,
        'nanosecond': 1e-9,
        'nanoseconds': 1e-9,
    }
    unit_lower = unit.lower()
    if unit_lower not in unit_map:
        raise ValueError(f"Unknown time unit: {unit}. Supported units: {list(unit_map.keys())}")
    return value * unit_map[unit_lower]

def sort_json_files(files: List[str], sort_by: str = 'number', sort_func: Optional[Callable] = None) -> List[str]:
    """
    Sort JSON files based on specified method
    
    Args:
        files: List of file paths
        sort_by: Sorting method - 'number', 'time', 'custom', or 'none'
        sort_func: Custom sorting function (used when sort_by='custom')
    
    Returns:
        Sorted list of file paths
    """
    if sort_by == 'number':
        files.sort(key=get_gesture_number)
    elif sort_by == 'time':
        files.sort(key=get_file_mtime)
    elif sort_by == 'custom':
        if sort_func is None:
            raise ValueError("sort_func must be provided when sort_by='custom'")
        files.sort(key=sort_func)
    elif sort_by == 'none':
        # Keep original order
        pass
    else:
        raise ValueError(f"Unknown sort_by value: {sort_by}. Supported: 'number', 'time', 'custom', 'none'")
    return files

def gen_h5(
    json_folder: Optional[str] = None,
    json_files: Optional[List[str]] = None,
    h5_file: str = "output.hdf5",
    interval: float = 1.0,
    interval_unit: str = 's',
    samples_per_gesture: int = 5,
    repeat_count: int = 1,
    sort_by: str = 'number',
    sort_func: Optional[Callable] = None,
    append: bool = False,
    start_stamp: Optional[float] = None
):
    """
    Generate HDF5 file from JSON gesture files
    
    Args:
        json_folder: Folder containing JSON files (used if json_files is None)
        json_files: List of specific JSON file paths to process (overrides json_folder)
        h5_file: Output HDF5 file path
        interval: Time interval between gestures (in specified unit)
        interval_unit: Time unit for interval ('s', 'ms', 'us', 'ns', etc.)
        samples_per_gesture: Number of time samples to generate per gesture
        repeat_count: Number of times to repeat each gesture
        sort_by: Sorting method - 'number', 'time', 'custom', or 'none'
        sort_func: Custom sorting function (used when sort_by='custom')
        append: If True, append to existing HDF5 file; if False, overwrite
        start_stamp: Starting timestamp (if None, uses 0.0 or last stamp from file if append=True)
    """
    # Get JSON files
    if json_files is not None:
        # Use provided file list
        json_files = [str(f) for f in json_files]
    elif json_folder is not None:
        # Get all JSON files from folder
        json_files = glob.glob(os.path.join(json_folder, "*.json"))
    else:
        raise ValueError("Either json_folder or json_files must be provided")
    
    if not json_files:
        print("Warning: No JSON files found")
        return
    
    # Sort files
    json_files = sort_json_files(json_files, sort_by=sort_by, sort_func=sort_func)
    
    # Convert interval to seconds
    interval_seconds = convert_time_unit(interval, interval_unit)
    sub_interval = interval_seconds / samples_per_gesture
    
    # Determine starting timestamp
    if start_stamp is not None:
        stamp = start_stamp
    elif append and os.path.exists(h5_file):
        stamp = get_last_stamp(h5_file) + sub_interval
    else:
        stamp = 0.0
    
    # Open HDF5 file
    mode = "a" if append and os.path.exists(h5_file) else "w"
    with H5Wrapper(h5_file, mode=mode) as wr:
        # Repeat the entire sequence of gestures
        for repeat_idx in range(repeat_count):
            # Process each gesture file in the sequence
            for json_file in json_files:
                with open(json_file, 'r') as f:
                    print(f"Processing {json_file} (repeat {repeat_idx + 1}/{repeat_count})")
                    try:
                        data = json.load(f)
                    except json.JSONDecodeError as e:
                        print(f"Error: Failed to parse {json_file}: {e}")
                        continue
                    
                    # Generate samples for this gesture
                    for sample_idx in range(samples_per_gesture):
                        stamp += sub_interval
                        wr.add_value('/stamp', stamp, 'float64')
                        
                        # Write action data
                        if 'action' in data and 'position' in data['action']:
                            # Handle nested structure
                            action_data = data['action']['position']
                        elif 'action' in data:
                            # Handle flat structure (dict of joint values)
                            action_data = [value for value in data['action'].values()]
                        else:
                            print(f"Warning: No 'action' data in {json_file}")
                            action_data = []
                        
                        if action_data:
                            wr.add_object('/action/position', action_data, 'float32')
                        
                        # Write state data
                        if 'state' in data and 'position' in data['state']:
                            # Handle nested structure
                            state_data = data['state']['position']
                        elif 'state' in data:
                            # Handle flat structure (dict of joint values)
                            state_data = [value for value in data['state'].values()]
                        else:
                            print(f"Warning: No 'state' data in {json_file}")
                            state_data = []
                        
                        if state_data:
                            wr.add_object('/state/position', state_data, 'float32')
    
    source_desc = f"{len(json_files)} JSON files" if json_files else json_folder
    print(f"Converted {source_desc} to {h5_file}")

if __name__ == "__main__":
    import argparse
    
    parser = argparse.ArgumentParser(
        description='Convert JSON gesture files to HDF5 format',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Convert all JSON files from folder
  python json_to_h5.py --json-folder ./exports --output test.hdf5
  
  # Convert specific files with custom sorting
  python json_to_h5.py --json-files Gesture_0.json Gesture_2.json --output test.hdf5 --sort-by time
  
  # Append to existing file with custom interval
  python json_to_h5.py --json-folder ./exports --output test.hdf5 --append --interval 2.0 --interval-unit s
  
  # Repeat each gesture 3 times
  python json_to_h5.py --json-folder ./exports --output test.hdf5 --repeat 3
        """
    )
    
    # Input options
    input_group = parser.add_mutually_exclusive_group(required=True)
    input_group.add_argument('--json-folder', type=str, help='Folder containing JSON files')
    input_group.add_argument('--json-files', nargs='+', help='List of specific JSON file paths')
    
    # Output options
    parser.add_argument('--output', '-o', type=str, default='output.hdf5', help='Output HDF5 file path')
    parser.add_argument('--append', '-a', action='store_true', help='Append to existing HDF5 file')
    
    # Time options
    parser.add_argument('--interval', type=float, default=1.0, help='Time interval between gestures')
    parser.add_argument('--interval-unit', type=str, default='s', choices=['s', 'ms', 'us', 'ns', 'sec', 'second', 'seconds', 'millisecond', 'milliseconds', 'microsecond', 'microseconds', 'nanosecond', 'nanoseconds'], help='Time unit for interval')
    parser.add_argument('--samples-per-gesture', type=int, default=5, help='Number of time samples per gesture')
    parser.add_argument('--start-stamp', type=float, default=None, help='Starting timestamp (overrides auto-detection)')
    
    # Sorting options
    parser.add_argument('--sort-by', type=str, default='number', choices=['number', 'time', 'custom', 'none'], help='Sorting method for JSON files')
    
    # Repeat options
    parser.add_argument('--repeat', type=int, default=1, help='Number of times to repeat each gesture')
    
    args = parser.parse_args()
    
    gen_h5(
        json_folder=args.json_folder,
        json_files=args.json_files,
        h5_file=args.output,
        interval=args.interval,
        interval_unit=args.interval_unit,
        samples_per_gesture=args.samples_per_gesture,
        repeat_count=args.repeat,
        sort_by=args.sort_by,
        append=args.append,
        start_stamp=args.start_stamp
    )