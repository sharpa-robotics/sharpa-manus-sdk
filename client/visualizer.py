#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Sharpa Manus Client 3D 手部关键点可视化器
订阅 ZMQ 消息并实时显示左右手关键点位置
"""

import zmq
import time
import threading
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.animation as animation
from matplotlib.patches import Circle
import sharpa_hand_pb2 as proto

class HandVisualizer:
    def __init__(self, zmq_host="tcp://192.168.10.222:2044"):
        """
        初始化手部可视化器
        
        Args:
            zmq_host: ZMQ 订阅地址
        """
        self.zmq_host = zmq_host
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.SUB)
        self.socket.connect(self.zmq_host)
        self.socket.setsockopt_string(zmq.SUBSCRIBE, "")
        
        # Hand keypoints data
        self.right_hand_points = []
        self.left_hand_points = []
        self.timestamp = None
        self.frame_id = None
        
        # Status information
        self.connection_status = "Disconnected"
        self.last_receive_time = None
        self.receive_count = 0
        
        # Hand keypoint names (based on finger structure, total 24 points)
        self.joint_names = [
            "0",           # 0: Hand root
            "1",       # 1: Thumb first joint
            "2",       # 2: Thumb second joint  
            "3",       # 3: Thumb third joint
            "4",       # 4: Thumb fingertip
            "5",       # 5: Extra point 1 (insertion position)
            "6",       # 6: Index finger first joint
            "7",       # 7: Index finger second joint
            "8",       # 8: Index finger third joint
            "9  ",       # 9: Index finger fingertip
            "10",       # 10: Extra point 2 (insertion position)
            "11",      # 11: Middle finger first joint
            "12",      # 12: Middle finger second joint
            "13",      # 13: Middle finger third joint
            "14",      # 14: Middle finger fingertip
            "15",       # 15: Extra point 3 (insertion position)
            "16",        # 16: Ring finger first joint
            "17",        # 17: Ring finger second joint
            "18",        # 18: Ring finger third joint
            "19",        # 19: Ring finger fingertip
            "20",       # 20: Extra point 4 (insertion position)
            "21",       # 21: Pinky first joint
            "22",       # 22: Pinky second joint
            "23",       # 23: Pinky third joint
            "24"        # 24: Pinky fingertip
        ]
        
        # Finger connection relationships (for drawing finger lines, considering extra insertion points)
        self.finger_connections = [
            # Thumb
            (0, 1), (1, 2), (2, 3), (3, 4),
            (0, 5), (5, 6), (6, 7), (7, 8), (8, 9),
            (0, 10), (10, 11), (11, 12), (12, 13), (13, 14),
            (0, 15), (15, 16), (16, 17), (17, 18), (18, 19),
            (0, 20), (20, 21), (21, 22), (22, 23), (23, 24),
        ]
        
        # Setup matplotlib
        plt.rcParams['font.sans-serif'] = ['SimHei', 'DejaVu Sans']
        plt.rcParams['axes.unicode_minus'] = False
        
        # Create 3D figure - split into two subplots
        self.fig = plt.figure(figsize=(20, 10))
        # Left hand subplot (left side)
        self.ax_left = self.fig.add_subplot(121, projection='3d')
        # Right hand subplot (right side)
        self.ax_right = self.fig.add_subplot(122, projection='3d')
        
        # Setup plot properties
        self.setup_plot()
        
        # Thread control
        self.running = False
        self.data_lock = threading.Lock()
        

        
    def setup_plot(self):
        """Setup basic properties of 3D plot"""
        
        # Setup coordinate axis range (based on hand size in code)
        for ax in [self.ax_left, self.ax_right]:
            ax.set_xlim(-0.15, 0.15)
            ax.set_ylim(-0.15, 0.15)
            ax.set_zlim(-0.08, 0.08)
            
            # Lock coordinate axis ratio to ensure proportional display
            ax.set_box_aspect([1, 1, 0.5])  # Set x:y:z ratio to 1:1:0.5
            
            # Setup view angle (based on Manus SDK coordinate system: right-handed, Z-up)
            # Adjust elevation and azimuth for better perspective effect
            ax.view_init(elev=15, azim=30)
            
            # Add grid
            ax.grid(True, alpha=0.3)
            
            # Setup projection type to perspective projection
            ax.set_proj_type('persp', focal_length=0.1)
            
            # Optimize perspective effect
            ax.dist = 8  # Set observation distance
            
            # Hide coordinate axis ticks and lines
            ax.set_xticks([])
            ax.set_yticks([])
            ax.set_zticks([])
            ax.set_axis_off()
        
        
        
        # Adjust layout to minimize blank areas
        self.fig.tight_layout()
        self.fig.subplots_adjust(left=0.05, right=0.95, top=0.9, bottom=0.05, wspace=0.1)
        
    def receive_data(self):
        """Receive ZMQ data thread function"""
        print(f"Starting to listen to ZMQ messages: {self.zmq_host}")
        
        while self.running:
            try:
                # Non-blocking receive
                message = self.socket.recv(flags=zmq.NOBLOCK)
                
                # Parse protobuf message
                mocap_msg = proto.MocapKeypoints()
                mocap_msg.ParseFromString(message)
                
                # Extract right and left hand data
                with self.data_lock:
                    self.right_hand_points = []
                    self.left_hand_points = []
                    
                    # Process right hand data
                    for pose in mocap_msg.right_mocap_pose:
                        point = {
                            'position': (
                                pose.position.x,
                                pose.position.y,
                                pose.position.z
                            ),
                            'orientation': (
                                pose.orientation.w,
                                pose.orientation.x,
                                pose.orientation.y,
                                pose.orientation.z
                            )
                        }
                        self.right_hand_points.append(point)
                    
                    # Process left hand data
                    for pose in mocap_msg.left_mocap_pose:
                        point = {
                            'position': (
                                pose.position.x,
                                pose.position.y,
                                pose.position.z
                            ),
                            'orientation': (
                                pose.orientation.w,
                                pose.orientation.x,
                                pose.orientation.y,
                                pose.orientation.z
                            )
                        }
                        self.left_hand_points.append(point)
                    
                    self.timestamp = mocap_msg.header.stamp
                    self.frame_id = mocap_msg.header.frame_id
                    self.last_receive_time = time.time()
                    self.receive_count += 1
                    self.connection_status = "Connected"
                    
                    print(f"Received frame {self.frame_id}: Right hand {len(self.right_hand_points)} points, Left hand {len(self.left_hand_points)} points")
                    
            except zmq.Again:
                # No message, continue loop
                time.sleep(0.01)
                continue
            except Exception as e:
                print(f"Error receiving data: {e}")
                time.sleep(0.1)
                
    def draw_hand(self, positions, hand_color, hand_label, target_ax, is_left=False):
        """Draw keypoints and connections for a single hand"""
        if len(positions) == 0:
            return
            
        # Draw keypoints
        x, y, z = positions[:, 0], positions[:, 1], positions[:, 2]
        
        # Draw hand root (large point)
        target_ax.scatter(x[0], y[0], z[0], c='c', s=100, marker='o', 
                           label=f'{hand_label} Hand Root')
        
        # Draw finger joint points (skip extra points and fingertips)
        finger_joints = []
        fingertip_indices = [4, 9, 14, 19, 24]  # Fingertip indices
        for i in range(1, len(positions)):
            if i not in fingertip_indices:  # Skip extra points and fingertips
                finger_joints.append(i)
        
        if finger_joints:
            target_ax.scatter(x[finger_joints], y[finger_joints], z[finger_joints], 
                              c=hand_color, s=50, marker='o', alpha=0.8, 
                              label=f'{hand_label} Finger Joints')
        
        # Draw fingertip points (green, large points)
        fingertip_valid = [i for i in fingertip_indices if i < len(positions)]
        if fingertip_valid:
            target_ax.scatter(x[fingertip_valid], y[fingertip_valid], z[fingertip_valid], 
                          c='green', s=80, marker='o', alpha=0.9, 
                          label=f'{hand_label} Fingertips')
        
        # Draw finger connection lines
        for start_idx, end_idx in self.finger_connections:
            if start_idx < len(positions) and end_idx < len(positions):
                start_pos = positions[start_idx]
                end_pos = positions[end_idx]
                target_ax.plot([start_pos[0], end_pos[0]], 
                           [start_pos[1], end_pos[1]], 
                           [start_pos[2], end_pos[2]], 
                           color=hand_color, linewidth=2, alpha=0.7)
        
        # Add keypoint labels
        main_joints = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24]
        for i in main_joints:
            if i < len(positions) and i < len(self.joint_names):
                pos = positions[i]
                name = f"{self.joint_names[i]}"
                # Adjust label position to avoid overlap
                offset = 0.01 if is_left else -0.01
                target_ax.text(pos[0] + offset, pos[1] + offset, pos[2] + offset, 
                           name, fontsize=10, color='black')
        
        # 绘制手腕和指尖的坐标轴（使用方向信息）
        if len(positions) > 0:
            # 手腕点坐标轴（红色、绿色、蓝色分别代表X、Y、Z）
            wrist_pos = positions[0]
            axis_length = 0.03  # 坐标轴长度
            
            # X轴（红色）
            target_ax.quiver(wrist_pos[0], wrist_pos[1], wrist_pos[2], 
                           axis_length, 0, 0, color='red', arrow_length_ratio=0.3, linewidth=2)
            
            # Y轴（绿色）
            target_ax.quiver(wrist_pos[0], wrist_pos[1], wrist_pos[2], 
                           0, axis_length, 0, color='green', arrow_length_ratio=0.3, linewidth=2)
            
            # Z轴（蓝色）
            target_ax.quiver(wrist_pos[0], wrist_pos[1], wrist_pos[2], 
                           0, 0, axis_length, color='blue', arrow_length_ratio=0.3, linewidth=2)
            
            # 为每个指尖点绘制局部坐标轴（根据手指方向）
            for fingertip_idx in fingertip_valid:
                if fingertip_idx < len(positions) and fingertip_idx < len(self.right_hand_points):
                    fingertip_pos = positions[fingertip_idx]
                    tip_axis_length = 0.02  # 指尖坐标轴稍短
                    
                    # 获取指尖点的方向信息（四元数）
                    fingertip_data = self.right_hand_points[fingertip_idx] if not is_left else self.left_hand_points[fingertip_idx]
                    if 'orientation' in fingertip_data:
                        # 从四元数计算局部坐标轴方向
                        qw, qx, qy, qz = fingertip_data['orientation']
                        
                        # 计算旋转矩阵（简化版本）
                        # X轴方向（手指指向方向）
                        x_dir = np.array([
                            1 - 2*qy*qy - 2*qz*qz,
                            2*qx*qy + 2*qw*qz,
                            2*qx*qz - 2*qw*qy
                        ])
                        
                        # Y轴方向（手指侧面方向）
                        y_dir = np.array([
                            2*qx*qy - 2*qw*qz,
                            1 - 2*qx*qx - 2*qz*qz,
                            2*qy*qz + 2*qw*qx
                        ])
                        
                        # Z轴方向（手指背面方向）
                        z_dir = np.array([
                            2*qx*qz + 2*qw*qy,
                            2*qy*qz - 2*qw*qx,
                            1 - 2*qx*qx - 2*qy*qy
                        ])
                        
                        # 绘制局部坐标轴
                        # X轴（红色，手指指向方向）
                        target_ax.quiver(fingertip_pos[0], fingertip_pos[1], fingertip_pos[2], 
                                       x_dir[0] * tip_axis_length, x_dir[1] * tip_axis_length, x_dir[2] * tip_axis_length, 
                                       color='red', arrow_length_ratio=0.2, linewidth=1, alpha=0.7)
                        
                        # Y轴（绿色，手指侧面方向）
                        target_ax.quiver(fingertip_pos[0], fingertip_pos[1], fingertip_pos[2], 
                                       y_dir[0] * tip_axis_length, y_dir[1] * tip_axis_length, y_dir[2] * tip_axis_length, 
                                       color='green', arrow_length_ratio=0.2, linewidth=1, alpha=0.7)
                        
                        # Z轴（蓝色，手指背面方向）
                        target_ax.quiver(fingertip_pos[0], fingertip_pos[1], fingertip_pos[2], 
                                       z_dir[0] * tip_axis_length, z_dir[1] * tip_axis_length, z_dir[2] * tip_axis_length, 
                                       color='blue', arrow_length_ratio=0.2, linewidth=1, alpha=0.7)
                
    def update_plot(self, frame):
        """Update 3D plot animation function"""
        with self.data_lock:
            # 清除之前的图形
            self.ax_left.clear()
            self.ax_right.clear()
            self.setup_plot()
            
            # Draw right hand (red) to right subplot
            if self.right_hand_points:
                right_positions = np.array([point['position'] for point in self.right_hand_points])
                self.draw_hand(right_positions, 'red', 'Right', self.ax_right, False)
            
            # Draw left hand (blue) to left subplot
            if self.left_hand_points:
                left_positions = np.array([point['position'] for point in self.left_hand_points])
                self.draw_hand(left_positions, 'blue', 'Left', self.ax_left, True)
            
            # Update title to show frame information
            right_count = len(self.right_hand_points) if self.right_hand_points else 0
            left_count = len(self.left_hand_points) if self.left_hand_points else 0
            total_count = right_count + left_count
            
            if self.frame_id:
                # Update main title
                self.fig.suptitle(f'Sharpa Manus Left-Right Hand Keypoints Real-time Visualization - Frame: {self.frame_id} (Right: {right_count} points, Left: {left_count} points, Total: {total_count} points)', 
                                fontsize=14, y=0.95)
            
            # Keep fixed coordinate axis range, no auto-adjustment
            # This ensures consistent view ratio for better hand motion observation
            for ax in [self.ax_left, self.ax_right]:
                ax.set_xlim(-0.15, 0.15)
                ax.set_ylim(-0.15, 0.15)
                ax.set_zlim(0.00, 0.15)
            
            # Add legends to respective subplots
            self.ax_left.legend(loc='upper right')
            self.ax_right.legend(loc='upper right')
            
            # Adjust layout to minimize blank areas
            self.fig.tight_layout()
            self.fig.subplots_adjust(left=0.05, right=0.95, top=0.9, bottom=0.05, wspace=0.1)
                    
    def start(self):
        """Start visualizer"""
        print("Starting left-right hand keypoints visualizer...")
        
        self.running = True
        
        # Start data receiving thread
        self.receive_thread = threading.Thread(target=self.receive_data)
        self.receive_thread.daemon = True
        self.receive_thread.start()
        
        # Start matplotlib animation
        self.ani = animation.FuncAnimation(
            self.fig, self.update_plot, 
            interval=50,  # 20 FPS
            blit=False
        )
        
        # Adjust layout to reduce blank areas
        plt.tight_layout()
        plt.subplots_adjust(left=0.05, right=0.95, top=0.9, bottom=0.05, wspace=0.1)
        
        plt.show()
        
    def stop(self):
        """Stop visualizer"""
        print("Stopping visualizer...")
        self.running = False
        
        if hasattr(self, 'receive_thread'):
            self.receive_thread.join(timeout=1)
            
        if hasattr(self, 'socket'):
            self.socket.close()
            
        if hasattr(self, 'context'):
            self.context.term()
            
        plt.close('all')

def main():
    """Main function"""
    import argparse
    
    parser = argparse.ArgumentParser(description='Sharpa Manus Left-Right Hand Keypoints Visualizer')
    parser.add_argument('--host', default='192.168.10.222', 
                       help='ZMQ server IP address (default: 192.168.10.222)')
    parser.add_argument('--port', default='2044', 
                       help='ZMQ server port (default: 2044)')
    
    args = parser.parse_args()
    
    # Build ZMQ address
    zmq_host = f"tcp://{args.host}:{args.port}"
    
    try:
        print(f"Connecting to ZMQ server: {zmq_host}")
        print("Press Ctrl+C to exit")
        
        # Create visualizer instance
        visualizer = HandVisualizer(zmq_host)
        
        # Start visualizer
        visualizer.start()
        
    except KeyboardInterrupt:
        print("\nUser interrupted, exiting...")
    except Exception as e:
        print(f"Runtime error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        if 'visualizer' in locals():
            visualizer.stop()

if __name__ == "__main__":
    main()
