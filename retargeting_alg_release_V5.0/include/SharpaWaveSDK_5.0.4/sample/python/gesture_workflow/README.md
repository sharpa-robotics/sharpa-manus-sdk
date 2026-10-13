# Gesture Workflow Examples

This directory contains example scripts for gesture recording, H5 file generation, and playback.

## Files

- **`gesture_workflow_example.py`**: Complete workflow examples demonstrating gesture recording, H5 generation, and replay
- **`sample_record_gesture.py`**: Sample script for recording gestures from device or using predefined data
- **`json_to_h5.py`**: Utility for converting JSON gesture files to HDF5 format
- **`example_gesture_data.txt`**: Example gesture data file in key-value format

## Quick Start

### Grouped Gesture Workflow

This workflow demonstrates how to:
1. Read gestures from a data file
2. Group gestures with different intervals and repetitions
3. Generate H5 file
4. Replay on device

```bash
cd backend/sharpaRT/sample/python/gesture_workflow

# Run grouped workflow with default gesture data file
python gesture_workflow_example.py --workflow grouped

# Run with custom gesture data file
python gesture_workflow_example.py --workflow grouped --gesture-file ./example_gesture_data.txt
```

### Available Workflows

The script supports the following workflows:

- `record` - Record from Device and Convert to H5
- `predefined` - Use Predefined Data to H5
- `external` - Read from External File and Convert
- `advanced` - Advanced Workflow (Select, Sort, Repeat, Append)
- `batch` - Batch Process Multiple Gestures
- `pipeline` - Complete Pipeline with Error Handling
- `interactive` - Interactive Workflow
- `grouped` - Grouped Gesture Workflow (Different intervals and repetitions)
- `replay` - Complete Workflow with Replay (Generate + Replay)
- `replay-existing` - Replay Existing H5 File

### Configuration

Edit `gesture_workflow_example.py` to customize:
- Gesture groups and intervals
- Repetition counts
- Output paths

The default configuration in grouped workflow:
- **Group 1**: First 3 gestures, interval 2s, repeat 5 times
- **Group 2**: Middle 3 gestures, interval 3s, repeat 2 times  
- **Group 3**: Last 3 gestures, interval 3s, repeat 5 times

## Requirements

- Python 3.6+
- sharpa SDK (installed or in source tree)
- h5py
- numpy

## Usage

### Basic Examples

```bash
# Record gesture from device and convert to H5
python3 gesture_workflow_example.py --workflow record

# Use predefined gesture data
python3 gesture_workflow_example.py --workflow predefined

# Read gestures from external file
python3 gesture_workflow_example.py --workflow external --input-file ./example_gesture_data.txt

# Replay existing H5 file
python3 gesture_workflow_example.py --workflow replay-existing --h5-file ./output/predefined_gestures.hdf5
# Run all workflows (skips device-dependent ones)
python3 gesture_workflow_example.py --all
```

### Advanced Options

```bash
# Grouped workflow with custom gesture file and hand side
python3 gesture_workflow_example.py --workflow grouped \
    --gesture-file ./example_gesture_data.txt \
    --hand-side LEFT

# Replay with custom discovery timeout
python3 gesture_workflow_example.py --workflow replay-existing \
    --h5-file ./output/my_gestures.hdf5 \
    --discovery-timeout 15.0
```

See individual file docstrings for detailed usage instructions.

## Output

Generated files will be saved to:

- JSON files: `./exports/`
- H5 files: `./output/`

These directories are listed in `backend/sharpaRT/.gitignore` so they are not committed by default.
