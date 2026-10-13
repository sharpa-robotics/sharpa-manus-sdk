import sys
sys.path.insert(0, '/opt/sharpa-wave-sdk/python')
from sharpa import SharpaWaveManager, HandSide, DeviceType

import cv2
import numpy as np
import time
import json

# Global variable declarations
latency = []
frame = {}
frame_count = 0
start_time = 0

def device_has_tactile(device_info):
    checker = getattr(device_info, "has_fingertip_tactile", None)
    if callable(checker):
        return bool(checker())
    return bool(checker)

def callback(frames):
    global latency, frame, frame_count, start_time
    try:
        ch = frames["channel"]
        ts = frames["ts"]

        # Process RAW image if available
        img_reshaped = None
        raw_data = frames["content"].get("RAW")
        if raw_data is not None:
            img = raw_data
            img = img.squeeze()
            height = frames["shape"]["RAW"][1]
            width = frames["shape"]["RAW"][2]
            img_reshaped = img.reshape(height, width).astype(np.uint8)

        f, deform_reshaped, contact_points = None, None, None
        f6_data = frames["content"].get("F6")
        deform_data = frames["content"].get("DEFORM")
        contact_point_data = frames["content"].get("CONTACT_POINT")
        if f6_data is not None and deform_data is not None:
            f = f6_data
            deform = deform_data
            height = frames["shape"]["DEFORM"][1]
            width = frames["shape"]["DEFORM"][2]
            deform_reshaped = deform.reshape(height, width).astype(np.uint8)
        
        # Extract CONTACT_POINT data if available
        if contact_point_data is not None:
            try:
                # Convert to numpy array if it's a list
                if isinstance(contact_point_data, list):
                    contact_points = np.array(contact_point_data, dtype=np.float32)
                else:
                    contact_points = np.asarray(contact_point_data, dtype=np.float32)
                
                # Squeeze and reshape
                contact_points = contact_points.squeeze()
                if contact_points.size > 0:
                    contact_points = contact_points.reshape(-1, contact_points.shape[-1] if contact_points.ndim > 1 else 3)
            except Exception as e:
                print(f"Error processing CONTACT_POINT data: {e}")
                contact_points = None

        # Update global frame dictionary
        frame[ch] = (img_reshaped, f, deform_reshaped, contact_points)
        
        # Calculate latency
        current_time = time.time()
        latency.append(current_time - ts)
        
    except Exception as e:
        print(f"Callback function processing error: {e}")

def main():
    global frame_count, start_time, latency, frame
    try:
        print("=== Sharpa Tactile Frame Data Acquisition Callback Example ===")
        
        # Create combined window - support two hands
        window_name = "Sharpa Tactile Data - LEFT (Left) | RIGHT (Right)"
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(window_name, 1280, 1200)  # 2 hands x 2 columns x 5 rows, each 320x240
        
        # Initialize image data - two hands, each hand 5 channels
        raw_images = [[np.zeros((240, 320), dtype=np.uint8) for _ in range(5)] for _ in range(2)]
        deform_images = [[np.zeros((240, 240), dtype=np.uint8) for _ in range(5)] for _ in range(2)]
        
        # Fixed hand position mapping: 0=LEFT, 1=RIGHT
        hand_positions = {HandSide.LEFT: 0, HandSide.RIGHT: 1}
        hand_names = ["LEFT", "RIGHT"]
        
        # Create combined image
        def create_combined_image():
            # Create blank canvas: 2 hands x 2 columns x 5 rows, each 320x240
            canvas = np.zeros((1200, 1280, 3), dtype=np.uint8)
            
            for hand_side in [HandSide.LEFT, HandSide.RIGHT]:
                hand_idx = hand_positions[hand_side]
                hand_offset_x = hand_idx * 640  # Each hand takes 640 pixels width
                hand_name = "LEFT" if hand_side == HandSide.LEFT else "RIGHT"
                
                for ch in range(5):
                    # RAW image (left column)
                    raw_y = ch * 240
                    raw_x = hand_offset_x + 0
                    if raw_images[hand_idx][ch] is not None:
                        if raw_images[hand_idx][ch].ndim == 2:
                            raw_bgr = cv2.cvtColor(raw_images[hand_idx][ch], cv2.COLOR_GRAY2BGR)
                        else:
                            raw_bgr = raw_images[hand_idx][ch]
                        canvas[raw_y:raw_y+240, raw_x:raw_x+320] = raw_bgr
                    
                        # Add RAW label with original channel number
                        original_ch = ch + 5 if hand_side == HandSide.LEFT else ch
                        cv2.putText(canvas, f'{hand_name}Ch{original_ch}-RAW', (raw_x+5, raw_y+20), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                    
                    # DEFORM image (right column)
                    deform_y = ch * 240
                    deform_x = hand_offset_x + 320
                    if deform_images[hand_idx][ch].ndim == 2:
                        deform_bgr = cv2.cvtColor(deform_images[hand_idx][ch], cv2.COLOR_GRAY2BGR)
                    else:
                        deform_bgr = deform_images[hand_idx][ch]
                    canvas[deform_y:deform_y+240, deform_x:deform_x+240] = deform_bgr
                    if raw_images[hand_idx][ch] is not None:
                    # Add DEFORM label with original channel number
                        cv2.putText(canvas, f'{hand_name}Ch{original_ch}-DEFORM', (deform_x+5, deform_y+20), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            
            # Add hand titles
            # cv2.putText(canvas, 'LEFT HAND', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 0), 3)
            # cv2.putText(canvas, 'RIGHT HAND', (650, 30), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 0), 3)
            
            return canvas
        
        # Display initial combined image
        initial_canvas = create_combined_image()
        cv2.imshow(window_name, initial_canvas)
        
        print("OpenCV combined window created, please ensure window is visible")
        cv2.waitKey(100)
        
        # Get SharpaWaveManager instance
        manager = SharpaWaveManager.get_instance()
        time.sleep(1)
        print("Touch good")
        
        # Wait for device connection
        device_infos = manager.get_all_devices()
        while not device_infos or not any(info.device_type == DeviceType.HAND for info in device_infos):
            print("Waiting for device connection...")
            time.sleep(1)
            device_infos = manager.get_all_devices()
        
        # Filter for HAND devices only
        device_infos = [info for info in device_infos if info.device_type == DeviceType.HAND]
        print(f"Found {len(device_infos)} HAND devices: {[info.sn for info in device_infos]}")
        
        # Connect devices and get hand types
        waves = []
        device_hand_sides = {}
        
        for i, device_info in enumerate(device_infos):
            device_sn = device_info.sn
            print(f"Connecting to device {i}: {device_sn}")
            wave = manager.connect(device_sn)
            connected_info = wave.get_device_info()
            if not device_has_tactile(connected_info):
                print(f"ERROR: Device {device_sn} firmware does not support tactile feature, exiting.")
                manager.disconnect_all()
                return 1
            waves.append(wave)
            print(f"Device {i} connected successfully")
            
            device_info = wave.get_device_info()
            print(f"Device {i} info: {device_info}")
            
            # Get hand type
            hand_side = device_info.hand_side
            device_hand_sides[device_sn] = hand_side
            hand_name = "LEFT" if hand_side == HandSide.LEFT else "RIGHT"
            print(f"Device {i} hand_side: {hand_side} ({hand_name})")
            
            # Update hand position mapping - ensure LEFT is always 0, RIGHT is always 1
            if hand_side == HandSide.LEFT:
                hand_positions[HandSide.LEFT] = 0
                hand_names[0] = "LEFT"
            elif hand_side == HandSide.RIGHT:
                hand_positions[HandSide.RIGHT] = 1
                hand_names[1] = "RIGHT"
            else:
                print(f"Warning: Unknown hand_side '{hand_side}' for device {i}, using default position")
                # Assign unknown hands to available positions
                if hand_positions[HandSide.LEFT] is None:
                    hand_positions[HandSide.LEFT] = 0
                    hand_names[0] = f"UNKNOWN_{i}"
                else:
                    hand_positions[HandSide.RIGHT] = 1
                    hand_names[1] = f"UNKNOWN_{i}"

        # Optional factory tactile JSON: pass SharpaWaveConfig when connecting, e.g.
        #   cfg = SharpaWaveConfig()
        #   cfg.tactile_config_file = "/path/to/tactile_config.json"
        #   wave = manager.connect(sn, cfg)

        # Set tactile callback for all devices
        for i, wave in enumerate(waves):
            wave.set_tactile_callback(callback)
            print(f"Set callback for device {i}")

        # Start devices
        for i, wave in enumerate(waves):
            if not wave.start():
                print(f"Wave {i} startup failed: check tactile port / host binding")
                # raise RuntimeError(f'Wave {i} startup failed')
            else:
                print(f"Wave {i} started successfully")
        
        # Initialize frame dictionary for all channels (0-9)
        frame = {ch: (None, None, None, None) for ch in range(10)}
        
        # Initialize latency statistics
        latency = []
        
        print("\nStarting to acquire tactile frame data...")
        print("Press 't' key to calibrate tactile sensors, 'j' key to toggle RAW_JPEG, 's' key to disable RAW_JPEG, press ESC key to exit")
        
        # Main loop: display images
        key = 0
        start_time = time.time()
        
        while key != 27:  # ESC key to exit
            key = cv2.waitKey(1) & 0xFF
            frame_count += 1
            
            if key == ord('t'):
                print("Starting tactile calibration...")
                for i, wave in enumerate(waves):
                    if not wave.calib_tactile(): 
                        print(f'Hand {hand_names[i]} tactile calibration failed')
                    else:
                        print(f'Hand {hand_names[i]} tactile calibration successful')
            
            if key == ord('j'):
                print("Toggling JPEG enable...")
                for i, wave in enumerate(waves):
                    try:
                        # Toggle JPEG (enable/disable)
                        # First get current state by trying to read it
                        # For simplicity, we'll just toggle between True and False
                        # You can modify this to read current state first if needed
                        print(f'Hand {hand_names[i]}: Attempting to enable JPEG...')
                        err = wave.set_parameter(
                            json.dumps(
                                {
                                    "secret_function": "set_tactile_jpeg_enable",
                                    "enable": True,
                                    "channel": -1,
                                }
                            )
                        )
                        if err.code == 0:
                            print(f'Hand {hand_names[i]} JPEG enabled successfully')
                            print(f'  Note: Changes may take a few seconds to take effect')
                            print(f'  RAW images will be updated automatically when new frames arrive')
                        else:
                            print(f'Hand {hand_names[i]} Failed to enable JPEG:')
                            print(f'  Error code: {err.code}')
                            print(f'  Error message: {err.message}')
                    except Exception as e:
                        print(f'Hand {hand_names[i]} Error toggling JPEG: {e}')
                        import traceback
                        traceback.print_exc()
            if key == ord('s'):
                print("Toggling JPEG disabled...")
                for i, wave in enumerate(waves):
                    try:
                        # Toggle JPEG (enable/disable)
                        # First get current state by trying to read it
                        # For simplicity, we'll just toggle between True and False
                        # You can modify this to read current state first if needed
                        print(f'Hand {hand_names[i]}: Attempting to disable JPEG...')
                        err = wave.set_parameter(
                            json.dumps(
                                {
                                    "secret_function": "set_tactile_jpeg_enable",
                                    "enable": False,
                                    "channel": -1,
                                }
                            )
                        )
                        if err.code == 0:
                            print(f'Hand {hand_names[i]} JPEG disabled successfully')
                            print(f'  Note: Changes may take a few seconds to take effect')
                            
                            # Clear RAW images for this hand
                            hand_side = wave.get_device_info().hand_side
                            hand_idx = hand_positions[hand_side]
                            print(f'  Clearing RAW images for {hand_names[i]} (hand_idx={hand_idx})...')
                            
                            # Determine channel range based on hand type
                            if hand_side == HandSide.LEFT:
                                channel_range = range(5, 10)  # Left hand: channels 5-9
                            else:
                                channel_range = range(0, 5)  # Right hand: channels 0-4
                            
                            # Clear frame dictionary and raw_images for this hand's channels
                            for ch in channel_range:
                                # Clear frame dictionary: set RAW image to None
                                if ch in frame:
                                    img, f, deform, contact_points = frame[ch]
                                    frame[ch] = (None, f, deform, contact_points)
                                
                                # Clear raw_images display
                                display_ch = ch - 5 if hand_side == HandSide.LEFT else ch
                                raw_images[hand_idx][display_ch] = np.zeros((240, 320), dtype=np.uint8)
                            
                            # Immediately refresh the display to show cleared images
                            combined_canvas = create_combined_image()
                            cv2.imshow(window_name, combined_canvas)
                            print(f'  RAW images cleared and display refreshed')
                        else:
                            print(f'Hand {hand_names[i]} Failed to disable JPEG:')
                            print(f'  Error code: {err.code}')
                            print(f'  Error message: {err.message}')
                            print(f'  This may indicate the device does not support disabling JPEG')
                    except Exception as e:
                        print(f'Hand {hand_names[i]} Error toggling JPEG: {e}')
                        import traceback
                        traceback.print_exc()
            
            image_updated = False
            
            # Process all channel data and update display
            for ch, (img, f, deform, contact_points) in frame.items():
                if img is None and deform is None: 
                    continue
                
                # Determine hand type and display position
                if ch >= 5:
                    # Channels 5-9 correspond to left hand
                    hand_side = HandSide.LEFT
                    hand_idx = hand_positions[HandSide.LEFT]
                    hand_name = "LEFT"
                    display_ch = ch - 5  # Channels 5-9 mapped to display positions 0-4
                else:
                    # Channels 0-4 correspond to right hand
                    hand_side = HandSide.RIGHT
                    hand_idx = hand_positions[HandSide.RIGHT]
                    hand_name = "RIGHT"
                    display_ch = ch  # Channels 0-4 directly mapped to display positions 0-4
                
                # Update RAW image
                raw_images[hand_idx][display_ch] = img
                image_updated = True
                
                # Update DEFORM image
                if f is not None and deform is not None:
                    try:
                        deform_img = cv2.cvtColor(deform, cv2.COLOR_GRAY2BGR)
                        
                        # Display F6 data
                        y_pos = 40
                        for i, var in enumerate(f):
                            if i < 6:  # Ensure only 6 values
                                text = f'F{i}: {float(var):.3f}'
                                cv2.putText(deform_img, text,
                                          (5, y_pos), cv2.FONT_HERSHEY_SIMPLEX, 
                                          0.6, (255, 0, 0), 2)
                                y_pos += 20
                        
                        # Draw CONTACT_POINT if available
                        if contact_points is not None:
                            try:
                                # Ensure contact_points is numpy array
                                if isinstance(contact_points, list):
                                    contact_points = np.array(contact_points, dtype=np.float32)
                                else:
                                    contact_points = np.asarray(contact_points, dtype=np.float32)
                                
                                if contact_points.size > 0:
                                    for point in contact_points:
                                        if len(point) >= 3:
                                            x, y = int(point[0]), int(point[1])
                                            confidence = point[2]
                                            # Draw circle at contact point
                                            cv2.circle(deform_img, (x, y), 2, (0, 0, 255), -1)
                                            # Draw confidence value
                                            cv2.putText(deform_img, "CONTACT",
                                                      (x + 10, y + 10), cv2.FONT_HERSHEY_SIMPLEX,
                                                      0.4, (0, 0, 255), 1)
                            except Exception as e:
                                print(f"Hand {hand_name} Channel {ch}: Error drawing CONTACT_POINT: {e}")
                        
                        deform_images[hand_idx][display_ch] = deform_img
                        image_updated = True
                        
                    except Exception as e:
                        print(f"Hand {hand_name} Channel {ch}: DEFORM image processing failed: {e}")
                        continue
            
            # Update combined image display
            if image_updated:
                combined_canvas = create_combined_image()
                cv2.imshow(window_name, combined_canvas)
            
            # Display statistics every 100 frames
            if frame_count % 100 == 0 and latency:
                current_time = time.time()
                elapsed_time = current_time - start_time
                avg_latency = sum(latency) / len(latency)
                print(f"--- Stats: total_frames={frame_count}, avg_latency={avg_latency:.3f}s, runtime={elapsed_time:.1f}s ---")
        
        # Get statistics
        try:
            user_got_frame = 0
            total_time = 0
            for i, wave in enumerate(waves):
                summary = wave.tactile_summary()
                if summary:
                    summary = json.loads(summary)
                else:
                    summary = {}
                print("\n=== Device Statistics ===")
                print(json.dumps(summary, indent=2))

                # Throughput statistics
                for item in summary['fingers']:
                    if item['recv_info'] is not None and item['recv_info']['start_ts'] > 0:
                        user_got_frame += item['recv_info']['frame_got']
                        total_time += time.time() - item['recv_info']['start_ts']
                
            if total_time > 0:
                print(f'\n=== Performance Statistics ===')
                print(f'SDK received frames: {user_got_frame} in {total_time:.1f} seconds, frame rate: {user_got_frame / total_time:.1f} fps')
                print(f'User received frames: {len(latency)} in {total_time:.1f} seconds, frame rate: {len(latency) / total_time:.1f} fps')
                
        except Exception as e:
            print(f"Unable to get statistics: {e}")
        
        # Stop all devices
        for i, wave in enumerate(waves):
            wave.stop()
            wave.destroy()
            print(f"\nWave {i} ({hand_names[i]}) stopped")
        manager.disconnect_all()
        
    except Exception as e:
        print(f"Program execution error: {e}")
        import traceback
        traceback.print_exc()
        return 1
    finally:
        # Ensure windows are closed
        cv2.destroyAllWindows()
    
    return 0

if __name__ == "__main__":
    exit(main())
