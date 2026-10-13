#!/usr/bin/env python3
"""
Gesture Recording Sample File

Features:
1. Support user-defined joint data array to batch generate JSON files
2. Optional: Get current joint data from Sharpa device and save
3. Support custom storage path (optional, default: exports folder in current directory)

Usage:
    # Use handwritten data (define GESTURE_DATA array in file)
    python sample_record_gesture.py
    
    # Get data from device
    python sample_record_gesture.py --from-device [--hand-side <LEFT|RIGHT>]
    
    # Specify output directory
    python sample_record_gesture.py --output-dir /path/to/output
"""

import sys
import os
import json
import argparse
from datetime import datetime
from pathlib import Path

# Add SDK path (from sample/python/gesture_workflow to src/hardware/hand_sdk/src/python)
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../../../src/hardware/hand_sdk/src/python'))

# Import from sharpa module - handle both SharpaDeviceManager and SharpaWaveManager
try:
    # Try importing from sharpa.sharpa directly (for source code)
    from sharpa.sharpa import (
        SharpaDeviceManager,
        SharpaHand,
        HandSide,
        ControlMode,
        ControlSource,
        Error
    )
except (ImportError, AttributeError):
    try:
        # Try importing from sharpa module (for installed package)
        from sharpa import (
            SharpaDeviceManager,
            SharpaHand,
            HandSide,
            ControlMode,
            ControlSource,
            Error
        )
    except (ImportError, AttributeError):
        # Fallback: use SharpaWaveManager (if SharpaDeviceManager not available)
        try:
            from sharpa.sharpa import (
                SharpaWaveManager as SharpaDeviceManager,
                SharpaWave as SharpaHand,
                HandSide,
                ControlMode,
                ControlSource,
                Error
            )
        except (ImportError, AttributeError):
            try:
                from sharpa import (
                    SharpaWaveManager as SharpaDeviceManager,
                    SharpaWave as SharpaHand,
                    HandSide,
                    ControlMode,
                    ControlSource,
                    Error
                )
            except ImportError as e:
                raise ImportError(f"Could not import SharpaDeviceManager/SharpaWaveManager from sharpa module: {e}")

# Joint index to joint name mapping (based on HandJointKeyMapHA)
JOINT_INDEX_TO_NAME = [
    'thumb_CMC_FE',      # 0
    'thumb_CMC_AA',      # 1
    'thumb_MCP_FE',      # 2
    'thumb_MCP_AA',      # 3
    'thumb_IP',          # 4
    'index_MCP_FE',      # 5
    'index_MCP_AA',      # 6
    'index_PIP',         # 7
    'index_DIP',         # 8
    'middle_MCP_FE',     # 9
    'middle_MCP_AA',     # 10
    'middle_PIP',        # 11
    'middle_DIP',        # 12
    'ring_MCP_FE',       # 13
    'ring_MCP_AA',       # 14
    'ring_PIP',          # 15
    'ring_DIP',          # 16
    'pinky_CMC',         # 17
    'pinky_MCP_FE',      # 18
    'pinky_MCP_AA',      # 19
    'pinky_PIP',         # 20
    'pinky_DIP',         # 21
]

# ============================================================================
# User-defined gesture data
# Define a series of gestures here, each gesture is a dictionary containing 'action' and 'state'
# Each dictionary key is a joint name, value is angle in radians
# ============================================================================
GESTURE_DATA = [
    # Example gesture 1: Open hand (neutral position)
    {
        'action': {
            'thumb_CMC_FE': 0,
            'thumb_CMC_AA': 0,
            'thumb_MCP_FE': 0,
            'thumb_MCP_AA': 0,
            'thumb_IP': 0,
            'index_MCP_FE': 0,
            'index_MCP_AA': 0,
            'index_PIP': 0,
            'index_DIP': 0,
            'middle_MCP_FE': 0,
            'middle_MCP_AA': 0,
            'middle_PIP': 0,
            'middle_DIP': 0,
            'ring_MCP_FE': 0,
            'ring_MCP_AA': 0,
            'ring_PIP': 0,
            'ring_DIP': 0,
            'pinky_CMC': 0,
            'pinky_MCP_FE': 0,
            'pinky_MCP_AA': 0,
            'pinky_PIP': 0,
            'pinky_DIP': 0
        },
        'state': {
            'thumb_CMC_FE': 0.007,
            'thumb_CMC_AA': -0.001,
            'thumb_MCP_FE': 0.003,
            'thumb_MCP_AA': -0.025,
            'thumb_IP': 0.262,
            'index_MCP_FE': 0.006,
            'index_MCP_AA': 0,
            'index_PIP': 0.027,
            'index_DIP': 0.307,
            'middle_MCP_FE': 0.006,
            'middle_MCP_AA': 0,
            'middle_PIP': 0.004,
            'middle_DIP': 0.002,
            'ring_MCP_FE': 0.004,
            'ring_MCP_AA': 0,
            'ring_PIP': 0.003,
            'ring_DIP': 0.001,
            'pinky_CMC': 0.022,
            'pinky_MCP_FE': -0.002,
            'pinky_MCP_AA': 0.002,
            'pinky_PIP': 0.034,
            'pinky_DIP': 0.029
        }
    },
    # Example gesture 2: Thumb extended
    {
        'action': {
            'thumb_CMC_FE': 0.006981317007977318,
            'thumb_CMC_AA': -0.0017453292519943296,
            'thumb_MCP_FE': 0.7714355293814937,
            'thumb_MCP_AA': 0.11344640137963143,
            'thumb_IP': 0.2617993877991494,
            'index_MCP_FE': 0.005235987755982988,
            'index_MCP_AA': 0,
            'index_PIP': 0.026179938779914945,
            'index_DIP': 0.30717794835100204,
            'middle_MCP_FE': 0.005235987755982988,
            'middle_MCP_AA': 0,
            'middle_PIP': 0.003490658503988659,
            'middle_DIP': 0.0017453292519943296,
            'ring_MCP_FE': 0.003490658503988659,
            'ring_MCP_AA': 0,
            'ring_PIP': 0.003490658503988659,
            'ring_DIP': 0.0017453292519943296,
            'pinky_CMC': 0.022689280275926284,
            'pinky_MCP_FE': -0.0017453292519943296,
            'pinky_MCP_AA': 0.0017453292519943296,
            'pinky_PIP': 0.03316125578789226,
            'pinky_DIP': 0.029670597283903602
        },
        'state': {
            'thumb_CMC_FE': 0.007,
            'thumb_CMC_AA': -0.001,
            'thumb_MCP_FE': 0.769,
            'thumb_MCP_AA': 0.109,
            'thumb_IP': 0.262,
            'index_MCP_FE': 0.006,
            'index_MCP_AA': 0,
            'index_PIP': 0.027,
            'index_DIP': 0.307,
            'middle_MCP_FE': 0.006,
            'middle_MCP_AA': 0,
            'middle_PIP': 0.004,
            'middle_DIP': 0.002,
            'ring_MCP_FE': 0.004,
            'ring_MCP_AA': 0,
            'ring_PIP': 0.003,
            'ring_DIP': 0.002,
            'pinky_CMC': 0.022,
            'pinky_MCP_FE': -0.002,
            'pinky_MCP_AA': 0.002,
            'pinky_PIP': 0.034,
            'pinky_DIP': 0.029
        }
    },
    # Add more gestures here...
]


def get_joint_data_from_hand(hand: SharpaHand):
    """
    Get joint data from SharpaHand and convert to JSON format
    
    Args:
        hand: SharpaHand instance
        
    Returns:
        dict: Dictionary containing 'action' and 'state'
    """
    # Get current state (actual joint angles)
    state = hand.get_states()
    state_angles = state.angles if state.angles else []
    
    # Get joint packet JSON (contains more information)
    err, payload = hand.get_parameter(["get_joint_packet_json"])
    if err.code == 0 and payload:
        rsp = json.loads(payload)
        joint_packet_json_str = rsp.get("get_joint_packet_json", "")
    else:
        joint_packet_json_str = ""
    joint_packet = json.loads(joint_packet_json_str) if joint_packet_json_str else {}
    
    # Get angles from joint packet if available
    packet_angles = joint_packet.get('angles', [])
    
    # Prefer angles from joint_packet, otherwise use angles from state
    angles = packet_angles if packet_angles else state_angles
    
    # Build action and state dictionaries
    action_dict = {}
    state_dict = {}
    
    # Convert angle array to joint name dictionary
    for i, angle in enumerate(angles):
        if i < len(JOINT_INDEX_TO_NAME):
            joint_name = JOINT_INDEX_TO_NAME[i]
            # action and state use the same angle value (in practice, action may be target position, state is actual position)
            action_dict[joint_name] = float(angle)
            state_dict[joint_name] = float(angle)
    
    return {
        'action': action_dict,
        'state': state_dict
    }


def save_gesture_to_json(data: dict, output_path: str, filename: str = None):
    """
    Save gesture data to JSON file
    
    Args:
        data: Dictionary containing 'action' and 'state'
        output_path: Output directory path
        filename: Filename (optional, auto-generated if not provided)
        
    Returns:
        str: Full path of saved file
    """
    # Ensure output directory exists
    output_dir = Path(output_path)
    output_dir.mkdir(parents=True, exist_ok=True)
    
    # Generate filename
    if filename is None:
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        filename = f'gesture_{timestamp}.json'
    
    # Ensure filename ends with .json
    if not filename.endswith('.json'):
        filename += '.json'
    
    # Build full path
    file_path = output_dir / filename
    
    # Save JSON file
    with open(file_path, 'w', encoding='utf-8') as f:
        json.dump(data, f, indent=2, ensure_ascii=False)
    
    return str(file_path)


def parse_txt_file_to_gestures(txt_file_path: str):
    """
    Parse external txt file containing joint data and convert to gesture format
    
    Supports multiple formats:
    1. JSON format (one JSON object per line, or single JSON array)
    2. Key-value pairs format (joint_name=value, one per line)
    3. CSV format (comma-separated values)
    
    Args:
        txt_file_path: Path to the input txt file
        
    Returns:
        list: List of gesture dictionaries, each containing 'action' and 'state'
    """
    gestures = []
    
    try:
        with open(txt_file_path, 'r', encoding='utf-8') as f:
            content = f.read().strip()
        
        # Try to parse as JSON first
        try:
            data = json.loads(content)
            # If it's a list of gestures
            if isinstance(data, list):
                for item in data:
                    if isinstance(item, dict) and 'action' in item and 'state' in item:
                        gestures.append(item)
            # If it's a single gesture
            elif isinstance(data, dict) and 'action' in data and 'state' in data:
                gestures.append(data)
        except json.JSONDecodeError:
            # Not JSON, try to parse as key-value pairs or other formats
            # Split by gesture separator (--- or ===)
            gesture_blocks = []
            current_block = []
            
            for line in content.split('\n'):
                line_stripped = line.strip()
                # Check for gesture separator
                if line_stripped.startswith('---') or line_stripped.startswith('==='):
                    if current_block:
                        gesture_blocks.append('\n'.join(current_block))
                        current_block = []
                else:
                    current_block.append(line)
            
            # Add the last block
            if current_block:
                gesture_blocks.append('\n'.join(current_block))
            
            # If no separator found, treat entire content as one gesture
            if not gesture_blocks:
                gesture_blocks = [content]
            
            # Parse each gesture block
            for block in gesture_blocks:
                lines = block.split('\n')
                current_gesture = {'action': {}, 'state': {}}
                current_section = None
                
                for line in lines:
                    line = line.strip()
                    if not line or line.startswith('#'):
                        continue
                    
                    # Check if it's a section header
                    if line.lower() == 'action:' or line.lower() == '[action]':
                        current_section = 'action'
                        continue
                    elif line.lower() == 'state:' or line.lower() == '[state]':
                        current_section = 'state'
                        continue
                    
                    # Try to parse as key=value format
                    if '=' in line:
                        parts = line.split('=', 1)
                        if len(parts) == 2:
                            key = parts[0].strip()
                            try:
                                value = float(parts[1].strip())
                                if current_section:
                                    current_gesture[current_section][key] = value
                                else:
                                    # If no section specified, add to both
                                    current_gesture['action'][key] = value
                                    current_gesture['state'][key] = value
                            except ValueError:
                                continue
                    
                    # Try to parse as CSV format (joint_name,action_value,state_value)
                    elif ',' in line:
                        parts = [p.strip() for p in line.split(',')]
                        if len(parts) >= 2:
                            key = parts[0]
                            try:
                                action_val = float(parts[1]) if len(parts) > 1 else 0.0
                                state_val = float(parts[2]) if len(parts) > 2 else action_val
                                current_gesture['action'][key] = action_val
                                current_gesture['state'][key] = state_val
                            except ValueError:
                                continue
                
                # If we have collected data, add it as a gesture
                if current_gesture['action'] or current_gesture['state']:
                    gestures.append(current_gesture)
        
        if not gestures:
            print(f"Warning: No valid gesture data found in {txt_file_path}")
            return []
        
        return gestures
        
    except FileNotFoundError:
        print(f"Error: File not found: {txt_file_path}")
        return []
    except Exception as e:
        print(f"Error: Failed to parse file {txt_file_path}: {str(e)}")
        import traceback
        traceback.print_exc()
        return []


def save_gestures_from_array(gesture_array: list, output_path: str, filename_prefix: str = 'Gesture'):
    """
    Batch save gesture array to JSON files
    
    Args:
        gesture_array: Array of gesture data, each element is a dictionary containing 'action' and 'state'
        output_path: Output directory path
        filename_prefix: Filename prefix (default: 'Gesture')
        
    Returns:
        list: List of full paths of saved files
    """
    # Ensure output directory exists
    output_dir = Path(output_path)
    output_dir.mkdir(parents=True, exist_ok=True)
    
    saved_files = []
    
    for index, gesture_data in enumerate(gesture_array):
        # Validate data format
        if not isinstance(gesture_data, dict):
            print(f"Warning: Data at index {index} is not a dictionary, skipping")
            continue
        
        if 'action' not in gesture_data or 'state' not in gesture_data:
            print(f"Warning: Data at index {index} is missing 'action' or 'state' field, skipping")
            continue
        
        # Generate filename: Gesture_0.json, Gesture_1.json, ...
        filename = f'{filename_prefix}_{index}.json'
        file_path = output_dir / filename
        
        # Save JSON file
        with open(file_path, 'w', encoding='utf-8') as f:
            json.dump(gesture_data, f, indent=2, ensure_ascii=False)
        
        saved_files.append(str(file_path))
        print(f"✓ Saved: {filename}")
    
    return saved_files


def record_gesture(output_dir: str = None, filename: str = None, hand_side: HandSide = HandSide.RIGHT, discovery_timeout: float = 10.0):
    """
    Record current gesture
    
    Args:
        output_dir: Output directory (optional, default: exports folder in current directory)
        filename: Filename (optional, auto-generated if not provided)
        hand_side: Hand side (LEFT or RIGHT, default: RIGHT)
        discovery_timeout: Maximum time to wait for device discovery in seconds (default: 10.0)
        
    Returns:
        str: Full path of saved file, None if failed
    """
    # Set default output directory
    if output_dir is None:
        default_dir = Path.cwd() / 'exports'
        output_dir = str(default_dir)
    
    print(f"Output directory: {output_dir}")
    
    # Get device manager
    manager = SharpaDeviceManager.get_instance()
    
    try:
        # Wait for device discovery (devices need time to send heartbeat packets)
        print(f"\nWaiting for device discovery (this may take a few seconds)...")
        import time
        max_wait_time = discovery_timeout  # Maximum wait time in seconds
        check_interval = 0.5  # Check every 0.5 seconds
        waited_time = 0.0
        
        while waited_time < max_wait_time:
            device_sns = manager.get_all_device_sn()
            if device_sns:
                print(f"✓ Found {len(device_sns)} device(s): {', '.join(device_sns)}")
                break
            time.sleep(check_interval)
            waited_time += check_interval
            print(f"  Waiting... ({waited_time:.1f}s / {max_wait_time}s)", end='\r')
        
        print()  # New line after progress
        
        # Check if any devices were found
        device_sns = manager.get_all_device_sn()
        if not device_sns:
            print(f"⚠ Warning: No devices found after {max_wait_time}s. Possible reasons:")
            print("  - Device is not powered on")
            print("  - Device is not connected to the network")
            print("  - Firewall blocking UDP port 54321")
            print("  - Try increasing discovery timeout with --discovery-timeout option")
            return None
        
        # Connect to device
        print(f"Connecting to {hand_side.name} hand...")
        try:
            hand = manager.connect(hand_side)
        except RuntimeError as e:
            # If connection fails, show available devices
            all_devices = manager.get_all_devices()
            if all_devices:
                print(f"  Available devices:")
                for dev in all_devices:
                    print(f"    - SN: {dev.sn}, Side: {dev.hand_side}, Type: {dev.device_type}")
            raise e
        
        if hand is None:
            print(f"Error: Failed to connect to {hand_side.name} hand device")
            return None
        
        print(f"Successfully connected to device: {hand.get_device_info().sn}")
        
        # Wait a moment to ensure data is stable
        import time
        print("Waiting for data to stabilize...")
        time.sleep(0.5)
        
        # Get joint data
        print("Getting joint data...")
        joint_data = get_joint_data_from_hand(hand)
        
        # Print data summary
        print(f"\nJoint data retrieved:")
        print(f"  Action joints: {len(joint_data['action'])}")
        print(f"  State joints: {len(joint_data['state'])}")
        
        # Save to JSON file
        print(f"\nSaving to file...")
        file_path = save_gesture_to_json(joint_data, output_dir, filename)
        
        print(f"✓ Gesture successfully saved to: {file_path}")
        
        return file_path
        
    except Exception as e:
        print(f"Error: Exception occurred while recording gesture: {str(e)}")
        import traceback
        traceback.print_exc()
        return None
    
    finally:
        # Disconnect
        try:
            manager.disconnect_all()
            print("\nDisconnected all devices")
        except:
            pass


def main():
    """Main function"""
    parser = argparse.ArgumentParser(
        description='Record Sharpa gestures and save as JSON files',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Use handwritten data (define GESTURE_DATA array in file)
  python sample_record_gesture.py
  
  # Get data from device
  python sample_record_gesture.py --from-device
  
  # Specify output directory
  python sample_record_gesture.py --output-dir /path/to/output
  
  # Get data from device and specify hand side
  python sample_record_gesture.py --from-device --hand-side LEFT
  
  # Specify filename prefix (only for handwritten data mode)
  python sample_record_gesture.py --filename-prefix MyGesture
  
  # Read from external txt file
  python sample_record_gesture.py --input-file /path/to/gesture_data.txt
  
  # Read from txt file and specify output directory
  python sample_record_gesture.py --input-file /path/to/gesture_data.txt --output-dir /path/to/output
        """
    )
    
    parser.add_argument(
        '--output-dir',
        type=str,
        default=None,
        help='Output directory path (default: exports folder in current directory)'
    )
    
    parser.add_argument(
        '--filename',
        type=str,
        default=None,
        help='Output filename (only for --from-device mode, default: gesture_YYYYMMDD_HHMMSS.json)'
    )
    
    parser.add_argument(
        '--filename-prefix',
        type=str,
        default='Gesture',
        help='Filename prefix (only for handwritten data mode, default: Gesture, generates Gesture_0.json, Gesture_1.json, ...)'
    )
    
    parser.add_argument(
        '--from-device',
        action='store_true',
        help='Get data from device (if not specified, use GESTURE_DATA array defined in file)'
    )
    
    parser.add_argument(
        '--hand-side',
        type=str,
        choices=['LEFT', 'RIGHT'],
        default='RIGHT',
        help='Hand side (only for --from-device mode, LEFT or RIGHT, default: RIGHT)'
    )
    
    parser.add_argument(
        '--input-file',
        type=str,
        default=None,
        help='Input txt file path containing joint data (will be parsed and saved as JSON files)'
    )
    
    args = parser.parse_args()
    
    # Set default output directory
    if args.output_dir is None:
        default_dir = Path.cwd() / 'exports'
        output_dir = str(default_dir)
    else:
        output_dir = args.output_dir
    
    # Handle input file mode
    if args.input_file:
        print(f"Reading gesture data from: {args.input_file}")
        gestures = parse_txt_file_to_gestures(args.input_file)
        
        if not gestures:
            print("Error: No valid gestures found in input file")
            sys.exit(1)
        
        print(f"Found {len(gestures)} gesture(s) in input file")
        print(f"Output directory: {output_dir}")
        print(f"Filename prefix: {args.filename_prefix}\n")
        
        saved_files = save_gestures_from_array(
            gesture_array=gestures,
            output_path=output_dir,
            filename_prefix=args.filename_prefix
        )
        
        if saved_files:
            print(f"\n✓ Done! Saved {len(saved_files)} JSON files:")
            for file_path in saved_files:
                print(f"  - {file_path}")
            print(f"\nTip: You can use json_to_h5.py to convert these JSON files to h5 files for playback")
            sys.exit(0)
        else:
            print("\n✗ Failed: No files were saved successfully")
            sys.exit(1)
    
    if args.from_device:
        # Get data from device
        hand_side = HandSide.LEFT if args.hand_side == 'LEFT' else HandSide.RIGHT
        file_path = record_gesture(
            output_dir=output_dir,
            filename=args.filename,
            hand_side=hand_side
        )
        
        if file_path:
            print(f"\n✓ Done! File saved: {file_path}")
            sys.exit(0)
        else:
            print("\n✗ Failed: Unable to record gesture")
            sys.exit(1)
    else:
        # Use handwritten data
        if not GESTURE_DATA:
            print("Error: GESTURE_DATA array is empty, please define gesture data in file")
            print("Or use --from-device parameter to get data from device")
            sys.exit(1)
        
        print(f"Processing {len(GESTURE_DATA)} gestures...")
        print(f"Output directory: {output_dir}")
        print(f"Filename prefix: {args.filename_prefix}\n")
        
        saved_files = save_gestures_from_array(
            gesture_array=GESTURE_DATA,
            output_path=output_dir,
            filename_prefix=args.filename_prefix
        )
        
        if saved_files:
            print(f"\n✓ Done! Saved {len(saved_files)} JSON files:")
            for file_path in saved_files:
                print(f"  - {file_path}")
            print(f"\nTip: You can use json_to_h5.py to convert these JSON files to h5 files for playback")
            sys.exit(0)
        else:
            print("\n✗ Failed: No files were saved successfully")
            sys.exit(1)


if __name__ == '__main__':
    main()
