"""
Standalone H5Replay implementation for SDK samples.

This is a simplified version of H5Replay that can be used independently
without requiring the full pilot SDK plugins. It provides the same interface
as the plugin version for compatibility.

Usage:
    from h5_replay import H5Replay
    
    # Create a mock service that implements set_angles
    class MockHandService:
        def set_angles(self, angles, source, direct_control=False):
            # Your implementation here
            pass
    
    service = MockHandService()
    replay = H5Replay(service)
    replay.start('path/to/file.hdf5')
    # ... wait for completion ...
    replay.close()
"""

import time
import logging
import h5py
import numpy as np
import threading


class H5Replay:
    """
    H5 file replay class for playing back recorded gesture data.
    
    This class reads HDF5 files containing gesture data and replays them
    by calling set_angles on the provided hand_service at the appropriate
    timestamps.
    
    Args:
        hand_service: Service object that implements set_angles(angles, source, direct_control)
    
    Example:
        class MockHandService:
            def set_angles(self, angles, source, direct_control=False):
                # Set joint angles on device
                pass
        
        service = MockHandService()
        replay = H5Replay(service)
        replay.start('gesture.hdf5')
        # Wait for completion...
        replay.close()
    """
    
    def __init__(self, hand_service):
        """
        Initialize H5Replay.
        
        Args:
            hand_service: Service object with set_angles method
        """
        self.lock = threading.Lock()
        self.hand_service = hand_service
        self.h5_file = None
        self.replay_index = 0
        self.active = False
        self.running = False
        self.thread = None
        self.stamp = None
        self.states = None
        self.actions = None

    def start(self, file_path):
        """
        Start replaying from an H5 file.
        
        Args:
            file_path: Path to the HDF5 file to replay
        
        Raises:
            FileNotFoundError: If the file doesn't exist
            KeyError: If required datasets are missing from the file
        """
        logging.info(f'start replay {file_path}')
        with self.lock:
            # Stop existing thread if any
            self.running = False
            if self.thread and self.thread.is_alive():
                self.thread.join()
            
            # Close existing file if any
            if self.h5_file:
                self.h5_file.close()
                self.h5_file = None
            
            # Load H5 file and data FIRST (before starting thread)
            try:
                self.h5_file = h5py.File(file_path, 'r')
                self.get_data_from_h5(self.h5_file)
            except FileNotFoundError:
                logging.error(f'H5 file not found: {file_path}')
                raise
            except KeyError as e:
                logging.error(f'Missing required dataset in H5 file: {e}')
                if self.h5_file:
                    self.h5_file.close()
                    self.h5_file = None
                raise
            
            # Reset state
            self.replay_index = 0
            self.active = True
            self.running = True
            
            # Start thread AFTER data is loaded
            self.thread = threading.Thread(target=self.run)
            self.thread.start()

    def pause(self):
        """Pause the replay (can be resumed with resume())."""
        with self.lock:
            if self.active:
                self.active = False

    def resume(self):
        """Resume a paused replay."""
        with self.lock:
            if not self.active:
                self.active = True

    def close(self):
        """
        Stop replay and clean up resources.
        
        This should be called when done with the replay to properly
        close the H5 file and stop the replay thread.
        """
        with self.lock:
            self.active = False
            self.running = False
            if self.thread and self.thread.is_alive():
                self.thread.join()
            self.states = None
            self.actions = None
            self.stamp = None
            if self.h5_file:
                self.h5_file.close()
                self.h5_file = None

    def run(self):
        """
        Internal method that runs the replay loop in a separate thread.
        
        This method reads timestamps and actions from the loaded H5 data
        and calls set_angles on the hand_service at the appropriate times.
        """
        current_index = None
        current_action = None
        has_next = False
        sleep_time = 0.0
        
        while self.running:
            # Get data within lock
            with self.lock:
                if not self.active:
                    # Not active, release lock and sleep
                    current_index = None
                    current_action = None
                elif self.stamp is None or len(self.stamp) == 0:
                    # Data not loaded yet, release lock and sleep
                    current_index = None
                    current_action = None
                elif self.replay_index >= len(self.stamp):
                    # Reached end, mark as inactive
                    self.active = False
                    current_index = None
                    current_action = None
                else:
                    # Get current index and data (within lock)
                    current_index = self.replay_index
                    current_stamp = float(self.stamp[current_index])
                    current_action = self.actions[current_index].copy()  # Copy to avoid holding lock
                    
                    # Check if there's a next frame
                    has_next = current_index + 1 < len(self.stamp)
                    if has_next:
                        next_stamp = float(self.stamp[current_index + 1])
                        sleep_time = max(0.0, next_stamp - current_stamp)  # Ensure non-negative
                    else:
                        sleep_time = 0.0
                    
                    # Increment index before releasing lock
                    self.replay_index += 1
            
            # Execute set_angles outside of lock to avoid blocking
            if (self.active and 
                self.stamp is not None and 
                len(self.stamp) > 0 and 
                current_index is not None and 
                current_index < len(self.stamp) and
                current_action is not None):
                try:
                    self.hand_service.set_angles(current_action, f'REPLAY_H5', False)
                except Exception as e:
                    logging.error(f'Error setting angles at index {current_index}: {e}')
                    import traceback
                    traceback.print_exc()
                
                # Sleep for the calculated time
                if has_next and sleep_time > 0:
                    time.sleep(sleep_time)
                elif not has_next:
                    # Last frame played, mark as inactive
                    with self.lock:
                        self.active = False
                    logging.info(f'Replay completed: played all {len(self.stamp)} frames')
                    break
            
            # If not active or data not ready, sleep briefly
            if not self.active or self.stamp is None or len(self.stamp) == 0:
                time.sleep(0.1)

    def get_data_from_h5(self, h5_file):
        """
        Load data from H5 file.
        
        Args:
            h5_file: Open h5py.File object
        
        Raises:
            KeyError: If required datasets are missing
        """
        try:
            self.stamp = np.array(h5_file[f'/stamp'])
            self.states = np.array(h5_file[f'/state/position'])
            self.actions = np.array(h5_file[f'/action/position'])
        except KeyError as e:
            logging.error(f'Missing required dataset in H5 file: {e}')
            raise
