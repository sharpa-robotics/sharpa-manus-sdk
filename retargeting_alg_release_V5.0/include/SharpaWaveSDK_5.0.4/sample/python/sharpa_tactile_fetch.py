import sys
sys.path.insert(0, '/opt/sharpa-wave-sdk/python')
from sharpa import SharpaWaveManager, HandSide, DeviceType

import cv2
import numpy as np
import time
import json
from datetime import datetime

def format_timestamp(timestamp):
    dt = datetime.fromtimestamp(timestamp)
    ms = int((timestamp - int(timestamp)) * 1000)
    return f"{dt.strftime('%H:%M:%S')}.{ms:03d}"

def print_data_info(data_type, data):
    if data is None:
        print(f"  {data_type}: NULL")
        return
    
    if hasattr(data, 'shape'):
        print(f"  {data_type}: shape{list(data.shape)}, size: {data.size}")
    else:
        print(f"  {data_type}: {type(data)} - {data}")

def device_has_tactile(device_info):
    checker = getattr(device_info, "has_fingertip_tactile", None)
    if callable(checker):
        return bool(checker())
    return bool(checker)

def main():
    try:
        print("=== Sharpa Tactile Frame Data Acquisition Python Example ===")
        
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
        
        manager = SharpaWaveManager.get_instance()
        print("Touch good")
        while not manager.get_all_device_sn():
            print("Waiting for device connection...")
            time.sleep(1)

        device_infos = [info for info in manager.get_all_devices() if info.device_type == DeviceType.HAND]
        print(f"Found {len(device_infos)} HAND devices: {[info.sn for info in device_infos]}")
        
        # Connect devices and get hand types
        waves = []
        
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

        # Optional factory tactile JSON: use manager.connect(sn, SharpaWaveConfig(...))
        # with tactile_config_file set before start().

        # Start devices
        for i, wave in enumerate(waves):
            if not wave.start():
                print(f"Wave {i} startup failed: check tactile port / host binding")
                # raise RuntimeError(f'Wave {i} startup failed')
            else:
                print(f"Wave {i} started successfully")

        frame_count = 0
        latencies = []
        
        print("\nStarting to acquire tactile frame data...")
        print("Press 't' key to calibrate tactile sensors, 'j' key to toggle JPEG, 's' key to disable JPEG, press ESC key to exit")
        
        key = 0
        while key != 27:
            key = cv2.waitKey(1) & 0xFF
            
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
                print("Toggling JPEG disable...")
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
                            print(f'  You may need to wait or restart the tactile connection')
                            
                            # Clear RAW images for this hand
                            hand_side = wave.get_device_info().hand_side
                            hand_idx = hand_positions[hand_side]
                            print(f'  Clearing RAW images for {hand_names[i]} (hand_idx={hand_idx})...')
                            for ch in range(5):
                                raw_images[hand_idx][ch] = np.zeros((240, 320), dtype=np.uint8)
                            
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
            
            frame_count += 1
            image_updated = False  # Mark if image is updated
            
            # Process data for each hand
            for wave in waves:
                device_sn = wave.get_device_info().sn
                hand_side = wave.get_device_info().hand_side
                hand_idx = hand_positions[hand_side]
                hand_name = "LEFT" if hand_side == HandSide.LEFT else "RIGHT"
                
                # Determine channel range based on hand type
                if hand_side == HandSide.LEFT:
                    # Left hand corresponds to channels 5-9, mapped to display positions 0-4
                    channel_range = range(5, 10)
                    channel_offset = -5  # Channels 5-9 mapped to display positions 0-4
                else:
                    # Right hand corresponds to channels 0-4, mapped to display positions 0-4
                    channel_range = range(0, 5)
                    channel_offset = 0   # Channels 0-4 directly mapped to display positions 0-4
                
                for ch in channel_range:
                    try:
                        ret = wave.fetch_tactile_frame(ch)
                        if ret is None: 
                            continue
                        
                        if not isinstance(ret, dict) or "content" not in ret or "ts" not in ret:
                            print(f"Hand {hand_name} Channel {ch}: Invalid data format")
                            continue

                        ts = ret["ts"]
                        current_time = time.time()
                        latency = current_time - ts
                        latencies.append(latency)
                        # Calculate display position: channels 5-9 mapped to 0-4, channels 0-4 directly mapped to 0-4
                        display_ch = ch + channel_offset

                        raw_data = ret["content"].get("RAW")
                        if raw_data is not None:
                            img = raw_data
                            img = img.squeeze()
                            try:
                                if img.size == 76800:
                                    img_reshaped = img.reshape(240, 320).astype(np.uint8)
                                elif img.size == 57600:
                                    img_reshaped = img.reshape(240, 240).astype(np.uint8)
                                else:
                                    height = int(np.sqrt(img.size * 4/3))
                                    width = int(img.size / height)
                                    img_reshaped = img.reshape(height, width).astype(np.uint8)
                                # Update RAW image
                                raw_images[hand_idx][display_ch] = img_reshaped
                                image_updated = True
                                print(f"DEBUG: Updated {hand_name} RAW image for channel {ch} -> display_ch {display_ch}, shape: {img_reshaped.shape}, range: [{img_reshaped.min()}, {img_reshaped.max()}]")
                                if frame_count % 100 == 0:
                                    print(f"Hand {hand_name} Channel {ch} (display {display_ch}): RAW image updated, size: {img_reshaped.shape}, data type: {img_reshaped.dtype}")
                                    print(f"Hand {hand_name} Channel {ch} (display {display_ch}): Image data range: [{img_reshaped.min()}, {img_reshaped.max()}]")
                            except Exception as reshape_error:
                                print(f"Hand {hand_name} Channel {ch}: Image reshape failed: {reshape_error}")
                        
                        deform_data = ret["content"].get("DEFORM")
                        f6_data = ret["content"].get("F6")
                        contact_point_data = ret["content"].get("CONTACT_POINT")
                        
                        if deform_data is not None and f6_data is not None:
                            if deform_data.size > 0:
                                try:
                                    deform_reshaped = deform_data.reshape(240, 240).astype(np.uint8)
                                    
                                    # Create image with F6 data
                                    deform_img = cv2.cvtColor(deform_reshaped, cv2.COLOR_GRAY2BGR)
                                    
                                    y_pos = 40
                                    for i, value in enumerate(f6_data):
                                        text = f'F{i}: {float(value):.3f}'
                                        cv2.putText(deform_img, text,
                                                  (5, y_pos), cv2.FONT_HERSHEY_SIMPLEX, 
                                                  0.6, (255, 0, 0), 2)
                                        y_pos += 20
                                    
                                    # Draw CONTACT_POINT if available
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
                                        except Exception as contact_error:
                                            print(f"Hand {hand_name} Channel {ch}: CONTACT_POINT processing failed: {contact_error}")
                                    
                                    # Update DEFORM image
                                    deform_images[hand_idx][display_ch] = deform_img
                                    image_updated = True
                                    print(f"DEBUG: Updated {hand_name} DEFORM image for channel {ch} -> display_ch {display_ch}")
                                        
                                except Exception as deform_error:
                                    print(f"Hand {hand_name} Channel {ch}: DEFORM image processing failed: {deform_error}")
                                    continue
                        
                        if frame_count % 50 == 0:
                            print(f"Hand {hand_name} Channel {ch} (display {display_ch}) - Frame {frame_count} - Timestamp: {ts:.6f} - Latency: {latency:.3f}s")
                            
                            print("Data content:")
                            for data_type, data in ret["content"].items():
                                print_data_info(data_type, data)
                        
                        if len(latencies) > 100:
                            latencies = latencies[-50:]
                        
                        if frame_count % 200 == 0 and latencies:
                            avg_latency = sum(latencies) / len(latencies)
                            print(f"--- Statistics: Total frames={frame_count}, Average latency={avg_latency:.3f}s ---")
                            
                    except Exception as e:
                        print(f"Error processing Hand {hand_name} Channel {ch}: {e}")
                        import traceback
                        traceback.print_exc()
                        continue
            
            # Only regenerate and display combined image when there are image updates
            if image_updated:
                print(f"DEBUG: Updating combined image, frame {frame_count}")
                combined_canvas = create_combined_image()
                cv2.imshow(window_name, combined_canvas)
                print(f"DEBUG: Combined canvas shape: {combined_canvas.shape}, range: [{combined_canvas.min()}, {combined_canvas.max()}]")

        # Stop all devices
        for i, wave in enumerate(waves):
            wave.stop()
            wave.destroy()
            print(f"\nWave {i} ({hand_names[i]}) stopped")
        manager.disconnect_all()
        
        print("\n=== Statistics ===")
        print(f"Total frames: {frame_count}")
        if latencies:
            avg_latency = sum(latencies) / len(latencies)
            min_latency = min(latencies)
            max_latency = max(latencies)
            
            print("Latency statistics:")
            print(f"  Average latency: {avg_latency:.3f} seconds")
            print(f"  Minimum latency: {min_latency:.3f} seconds")
            print(f"  Maximum latency: {max_latency:.3f} seconds")
        
        print("\nProgram execution completed!")
        
    except Exception as e:
        print(f"Program execution error: {e}")
        import traceback
        traceback.print_exc()
        return 1
    finally:
        cv2.destroyAllWindows()
    
    return 0

if __name__ == "__main__":
    exit(main())
