import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation
from scipy.spatial.transform import Rotation as R

class HandKinematicVisualizer:
    def __init__(self, ax=None, linestyle='solid', plot_orientation=False):
        if ax is None:
            self.fig = plt.figure(figsize=(12, 10))
            self.ax = self.fig.add_subplot(111, projection='3d')
        else:
            self.fig = ax.figure
            self.ax = ax
        self.linestyle = linestyle
        self.plot_orientation = plot_orientation
        """
        Initialize kinematic hand visualizer
        Initialize kinematic hand visualizer
        """
        # Set up matplotlib figure
        # Set up matplotlib figure
        # self.fig = plt.figure(figsize=(12, 10))
        # self.ax = self.fig.add_subplot(111, projection='3d')
        
        # Define finger chains (Sharpa Wave URDF keypoint order)
        # Thumb: 0-3, Index: 4-7, Middle: 8-11, Ring: 12-15
        # Define finger chains (Sharpa Wave URDF keypoint order)
        # Thumb: 0-3, Index: 4-7, Middle: 8-11, Ring: 12-15
        self.thumb_chain = [0, 1, 2, 3]      # right_thumb_C_VL, MC_VL, DP, TIP
        self.index_chain = [4, 5, 6, 7]       # right_index_MC_VL, MP, DP, TIP
        self.middle_chain = [8, 9, 10, 11]    # right_middle_MC_VL, MP, DP, TIP
        self.ring_chain = [12, 13, 14, 15]    # right_ring_MC_VL, MP, DP, TIP
        self.pinky_chain = [16, 17, 18, 19]  # right_pinky_MC_VL, MP, DP, TIP
        self.all_chains = [self.thumb_chain, self.index_chain, self.middle_chain, self.ring_chain, self.pinky_chain]
        
        # Finger colors and names
        self.finger_colors = ['red']*4  # Hand links in red
        self.node_color = 'yellow'      # Hand nodes in yellow
        self.arrow_length = 0.02        # Unified arrow length
        # Finger colors and names
        self.finger_colors = ['red']*4  # Hand links in red
        self.node_color = 'yellow'      # Hand nodes in yellow
        self.arrow_length = 0.02        # Unified arrow length
        self.finger_names = ['Thumb', 'Index', 'Middle', 'Ring', 'Pinky']
        
        # Current keypoint data
        self.current_keypoints = np.zeros((16, 7))  # 16 keypoints, each 7D [x, y, z, qw, qx, qy, qz]
        # Current keypoint data
        self.current_keypoints = np.zeros((16, 7))  # 16 keypoints, each 7D [x, y, z, qw, qx, qy, qz]
        
        # Initialize drawing objects
        # Initialize drawing objects
        self.scatter = None
        self.lines = []
        
        # Initialize axis arrow objects
        # Initialize axis arrow objects
        self.axis_quivers = []
        
        # Initialize keypoint orientation arrow objects
        # Initialize keypoint orientation arrow objects
        self.keypoint_orientation_quivers = []
        
        # Initialize text label object list
        # Initialize text label object list
        self.text_labels = []
        
        # Initialize plot
        # Initialize plot
        self.setup_plot()
        
    def setup_plot(self):
        """Set up basic plot properties"""
        # 清空旧的线条列表，避免重复累积
        self.lines.clear()
        
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_zlabel('Z')
        self.ax.set_title('Kinematic Hand Visualization')
        
        # Set axis ranges
        # Set axis ranges
        self.ax.set_xlim([-0.1, 0.1])
        self.ax.set_ylim([-0.1, 0.1])
        self.ax.set_zlim([-0.1, 0.1])
        
        # Add legend
        # Add legend
        for i, (chain, name) in enumerate(zip(self.all_chains, self.finger_names)):
            line, = self.ax.plot([], [], [], color='red', linewidth=3, label=name, linestyle=self.linestyle)
            self.lines.append(line)
        
        self.ax.legend()
        self.ax.grid(True)
        
    def update_visualization(self, frame, plot_orientation=False):
        """FuncAnimation update function"""
        # Clear previous scatter plot
        """FuncAnimation update function"""
        # Clear previous scatter plot
        if self.scatter is not None:
            self.scatter.remove()
        
        # Clear previous axis arrows
        # Clear previous axis arrows
        for quiver in self.axis_quivers:
            quiver.remove()
        self.axis_quivers.clear()
        
        # Clear previous keypoint orientation arrows
        # Clear previous keypoint orientation arrows
        for quiver in self.keypoint_orientation_quivers:
            quiver.remove()
        self.keypoint_orientation_quivers.clear()
        
        # Clear previous text labels
        # Clear previous text labels
        for text in self.text_labels:
            text.remove()
        self.text_labels.clear()
        
        # Draw all keypoints (red dots) - using position information (first 3 columns)
        # Draw all keypoints (red dots) - using position information (first 3 columns)
        self.scatter = self.ax.scatter(
            self.current_keypoints[:, 0],
            self.current_keypoints[:, 1],
            self.current_keypoints[:, 2],
            c=self.node_color,
            s=50,  # Point size
            alpha=0.8,
            label='Keypoints'
        )
        
        # Draw world coordinate system arrows
        # Draw world coordinate system arrows
        origin = np.array([0, 0, 0])
        length = 0.02
        x_axis, y_axis, z_axis = np.array([1,0,0]), np.array([0,1,0]), np.array([0,0,1])
        
        self.axis_quivers.append(
            self.ax.quiver(origin[0], origin[1], origin[2], 
                          x_axis[0]*length, x_axis[1]*length, x_axis[2]*length, 
                          color='r', linewidth=2, label='X')
        )
        self.axis_quivers.append(
            self.ax.quiver(origin[0], origin[1], origin[2], 
                          y_axis[0]*length, y_axis[1]*length, y_axis[2]*length, 
                          color='g', linewidth=2, label='Y')
        )
        self.axis_quivers.append(
            self.ax.quiver(origin[0], origin[1], origin[2], 
                          z_axis[0]*length, z_axis[1]*length, z_axis[2]*length, 
                          color='b', linewidth=2, label='Z')
        )
        
        # Draw keypoint orientation arrows
        # Draw keypoint orientation arrows
        if self.plot_orientation:
            # Draw orientation for each fingertip
            # Draw orientation for each fingertip
            finger_tip_indices = [chain[-1] for chain in self.all_chains]
            for idx in finger_tip_indices:
                if np.any(self.current_keypoints[idx, 3:7] != 0):
                    quat = self.current_keypoints[idx, 3:7][[1,2,3,0]]
                    rotation_matrix = R.from_quat(quat).as_matrix()
                    x_dir = rotation_matrix[:, 0]
                    y_dir = rotation_matrix[:, 1]
                    z_dir = rotation_matrix[:, 2]
                    keypoint_pos = self.current_keypoints[idx, :3]
                    self.keypoint_orientation_quivers.append(
                        self.ax.quiver(keypoint_pos[0], keypoint_pos[1], keypoint_pos[2], 
                                    x_dir[0]*self.arrow_length, x_dir[1]*self.arrow_length, x_dir[2]*self.arrow_length, 
                                    color='red', linewidth=2, alpha=0.6)
                    )
                    self.keypoint_orientation_quivers.append(
                        self.ax.quiver(keypoint_pos[0], keypoint_pos[1], keypoint_pos[2], 
                                    y_dir[0]*self.arrow_length, y_dir[1]*self.arrow_length, y_dir[2]*self.arrow_length, 
                                    color='green', linewidth=2, alpha=0.6)
                    )
                    self.keypoint_orientation_quivers.append(
                        self.ax.quiver(keypoint_pos[0], keypoint_pos[1], keypoint_pos[2], 
                                    z_dir[0]*self.arrow_length, z_dir[1]*self.arrow_length, z_dir[2]*self.arrow_length, 
                                    color='blue', linewidth=2, alpha=0.6)
                    )
        
        # Update connection lines for each finger - using position information (first 3 columns)
        # Update connection lines for each finger - using position information (first 3 columns)
        for i, (chain, line) in enumerate(zip(self.all_chains, self.lines)):
            chain_points = self.current_keypoints[chain, :3]  # Use position information
            chain_points = self.current_keypoints[chain, :3]  # Use position information
            line.set_data(chain_points[:, 0], chain_points[:, 1])
            line.set_3d_properties(chain_points[:, 2])
        
        # Add keypoint number labels
        # Add keypoint number labels
        for i, point in enumerate(self.current_keypoints):
            text = self.ax.text(point[0], point[1], point[2], f'{i}', fontsize=8)
            self.text_labels.append(text)
        
        return [self.scatter] + self.lines + self.axis_quivers + self.keypoint_orientation_quivers + self.text_labels
    
    def update_keypoints(self, keypoints):
        """Update keypoint data"""
        """Update keypoint data"""
        self.current_keypoints = keypoints # xyz qw qx qy qz

    def start_animation(self):
        # Create FuncAnimation, 20Hz update frequency
        # Create FuncAnimation, 20Hz update frequency
        self.ani = FuncAnimation(self.fig, self.update_visualization, 
                               interval=100, blit=False, cache_frame_data=False)
        plt.show()

class RawManusVisualizer:
    def __init__(self, ax=None, linestyle='dashed', plot_orientation=False):
        """Visualizer for displaying raw data (25 joints)"""
        """Visualizer for displaying raw data (25 joints)"""
        if ax is None:
            self.fig = plt.figure(figsize=(10, 8))
            self.ax = self.fig.add_subplot(111, projection='3d')
        else:
            self.fig = ax.figure
            self.ax = ax
        self.linestyle = linestyle
        self.plot_orientation = plot_orientation
        
        # Define raw finger chains (25 joints)
        # Define raw finger chains (25 joints)
        self.raw_thumb_chain = [0, 1, 2, 3, 4]
        self.raw_index_chain = [5, 6, 7, 8, 9]
        self.raw_middle_chain = [10, 11, 12, 13, 14]
        self.raw_ring_chain = [15, 16, 17, 18, 19]
        self.raw_pinky_chain = [20, 21, 22, 23, 24]
        self.all_chains = [self.raw_thumb_chain, self.raw_index_chain, self.raw_middle_chain, self.raw_ring_chain, self.raw_pinky_chain]
        
        # Finger colors
        # Finger colors
        self.finger_colors = ['blue']*5
        self.node_color = 'green'
        self.arrow_length = 0.02
        self.finger_names = ['Thumb', 'Index', 'Middle', 'Ring', 'Pinky']
        
        # Store keypoint data
        # Store keypoint data
        self.current_keypoints = np.zeros((25, 7))
        self.current_orientations = {}
        
        # Initialize drawing objects
        # Initialize drawing objects
        self.scatter = None
        self.lines = []
        self.axis_quivers = []
        self.tip_orientation_quivers = []
        
        # Initialize plot
        # Initialize plot
        self.setup_plot()
        
    def setup_plot(self):
        """Set up basic plot properties"""
        # 清空旧的线条列表，避免重复累积
        self.lines.clear()
        
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_zlabel('Z')
        self.ax.set_title('Raw Manus Data Visualization')
        
        # Set axis ranges
        # Set axis ranges
        self.ax.set_xlim([-0.2, 0.2])
        self.ax.set_ylim([-0.2, 0.2])
        self.ax.set_zlim([-0.2, 0.2])
        
        # Add legend
        # Add legend
        for i, (chain, name) in enumerate(zip(self.all_chains, self.finger_names)):
            line, = self.ax.plot([], [], [], color='blue', linewidth=3, label=name, linestyle=self.linestyle)
            self.lines.append(line)
        
        self.ax.legend()
        self.ax.grid(True)
        
    def update_keypoints(self, key_points):
        """External direct call, pass in keypoint data, refresh visualization"""
        """External direct call, pass in keypoint data, refresh visualization"""
        self.current_keypoints = key_points

    def update_visualization(self, frame):
        """FuncAnimation update function"""        
        # Clear previous scatter plot
        """FuncAnimation update function"""        
        # Clear previous scatter plot
        if self.scatter is not None:
            self.scatter.remove()
        
        # Clear previous axis arrows
        # Clear previous axis arrows
        for quiver in self.axis_quivers:
            quiver.remove()
        self.axis_quivers.clear()
        
        # Clear previous fingertip orientation arrows
        # Clear previous fingertip orientation arrows
        for quiver in self.tip_orientation_quivers:
            quiver.remove()
        self.tip_orientation_quivers.clear()
        
        # Draw all joint points (green dots)
        # Draw all joint points (green dots)
        self.scatter, = self.ax.plot(
            self.current_keypoints[:, 0],
            self.current_keypoints[:, 1],
            self.current_keypoints[:, 2],
            'o',
            color=self.node_color,
            markersize=6,
            alpha=0.8,
            label='Raw Joints'
        )
        
        # Draw world coordinate system
        # Draw world coordinate system
        x_axis, y_axis, z_axis = np.array([1,0,0]), np.array([0,1,0]), np.array([0,0,1])
        origin = np.array([0, 0, 0])
        length = 0.03
        self.axis_quivers.append(
            self.ax.quiver(origin[0], origin[1], origin[2], x_axis[0]*length, x_axis[1]*length, x_axis[2]*length, color='r', linewidth=2)
        )
        self.axis_quivers.append(
            self.ax.quiver(origin[0], origin[1], origin[2], y_axis[0]*length, y_axis[1]*length, y_axis[2]*length, color='g', linewidth=2)
        )
        self.axis_quivers.append(
            self.ax.quiver(origin[0], origin[1], origin[2], z_axis[0]*length, z_axis[1]*length, z_axis[2]*length, color='b', linewidth=2)
        )
        
        # Draw fingertip orientation arrows
        # Draw fingertip orientation arrows
        finger_tip_indices = [4, 9, 14, 19, 24]
        self.current_orientations = {}
        for idx in finger_tip_indices:
            quat = self.current_keypoints[idx, 3:7][[1,2,3,0]]
            if np.any(quat != 0):
                rotation_matrix = R.from_quat(quat).as_matrix()
                self.current_orientations[idx] = rotation_matrix
        
        if self.plot_orientation:
            for idx in finger_tip_indices:
                if idx in self.current_orientations:
                    tip_pos = self.current_keypoints[idx, :3]
                    orientation_matrix = self.current_orientations[idx]
                    
                    # Extract axis directions
                    # Extract axis directions
                    x_dir = orientation_matrix[:, 0]
                    y_dir = orientation_matrix[:, 1]
                    z_dir = orientation_matrix[:, 2]
                    
                    # Draw orientation arrows
                    # Draw orientation arrows
                    self.tip_orientation_quivers.append(
                        self.ax.quiver(tip_pos[0], tip_pos[1], tip_pos[2], 
                                      x_dir[0]*self.arrow_length, x_dir[1]*self.arrow_length, x_dir[2]*self.arrow_length, 
                                      color='red', linewidth=3, alpha=0.8)
                    )
                    self.tip_orientation_quivers.append(
                        self.ax.quiver(tip_pos[0], tip_pos[1], tip_pos[2], 
                                      y_dir[0]*self.arrow_length, y_dir[1]*self.arrow_length, y_dir[2]*self.arrow_length, 
                                      color='green', linewidth=3, alpha=0.8)
                    )
                    self.tip_orientation_quivers.append(
                        self.ax.quiver(tip_pos[0], tip_pos[1], tip_pos[2], 
                                      z_dir[0]*self.arrow_length, z_dir[1]*self.arrow_length, z_dir[2]*self.arrow_length, 
                                      color='blue', linewidth=3, alpha=0.8)
                    )
            
        # Update connection lines for each finger
        # Update connection lines for each finger
        for i, (chain, line) in enumerate(zip(self.all_chains, self.lines)):
            chain_points = self.current_keypoints[chain, :3]
            line.set_data(chain_points[:, 0], chain_points[:, 1])
            line.set_3d_properties(chain_points[:, 2])
        
        return [self.scatter] + self.lines + self.axis_quivers + self.tip_orientation_quivers

class DualHandVisualizer:
    def __init__(self, display_mode='both', plot_orientation=False):
        """Dual hand real-time visualizer, create independent figures for left and right hands"""
        """Dual hand real-time visualizer, create independent figures for left and right hands"""
        self.display_mode = display_mode
        
        # Create figure for left hand
        # Create figure for left hand
        self.left_fig = plt.figure(figsize=(10, 8))
        self.left_fig.suptitle('Left Hand Visualization')
        self.left_ax = self.left_fig.add_subplot(111, projection='3d')
        
        # Create figure for right hand  
        # Create figure for right hand  
        self.right_fig = plt.figure(figsize=(10, 8))
        self.right_fig.suptitle('Right Hand Visualization')
        self.right_ax = self.right_fig.add_subplot(111, projection='3d')
        
        # Try to set window positions
        # Try to set window positions
        try:
            self.left_fig.canvas.manager.window.wm_geometry("+100+100")
            self.right_fig.canvas.manager.window.wm_geometry("+700+100")
        except:
            pass  # Ignore if setting position fails
            pass  # Ignore if setting position fails
        
        # Left hand visualizers
        # Left hand visualizers
        self.left_raw_visualizer = RawManusVisualizer(ax=self.left_ax, linestyle='dashed', plot_orientation=plot_orientation)
        self.left_hand_visualizer = HandKinematicVisualizer(ax=self.left_ax, linestyle='solid', plot_orientation=plot_orientation)
        
        # Right hand visualizers
        # Right hand visualizers
        self.right_raw_visualizer = RawManusVisualizer(ax=self.right_ax, linestyle='dashed', plot_orientation=plot_orientation)
        self.right_hand_visualizer = HandKinematicVisualizer(ax=self.right_ax, linestyle='solid', plot_orientation=plot_orientation)
        
        # Current frame data
        # Current frame data
        self.left_raw_keypoints = None
        self.left_hand_keypoints = None
        self.right_raw_keypoints = None
        self.right_hand_keypoints = None
        
    def update(self, left_raw_keypoints=None, left_hand_keypoints=None, 
               right_raw_keypoints=None, right_hand_keypoints=None):
        """Update dual hand data"""
        # Update left hand data
        """Update dual hand data"""
        # Update left hand data
        if left_raw_keypoints is not None:
            self.left_raw_keypoints = left_raw_keypoints
            self.left_raw_visualizer.update_keypoints(left_raw_keypoints)
        if left_hand_keypoints is not None:
            self.left_hand_keypoints = left_hand_keypoints
            self.left_hand_visualizer.update_keypoints(left_hand_keypoints)
            
        # Update right hand data
        # Update right hand data
        if right_raw_keypoints is not None:
            self.right_raw_keypoints = right_raw_keypoints
            self.right_raw_visualizer.update_keypoints(right_raw_keypoints)
        if right_hand_keypoints is not None:
            self.right_hand_keypoints = right_hand_keypoints
            self.right_hand_visualizer.update_keypoints(right_hand_keypoints)
            
    def _draw_left(self, frame):
        """Draw left hand"""
        """Draw left hand"""
        artists = []
        if self.display_mode in ['raw', 'both'] and self.left_raw_keypoints is not None:
            artists += self.left_raw_visualizer.update_visualization(frame)
        if self.display_mode in ['hand', 'both'] and self.left_hand_keypoints is not None:
            artists += self.left_hand_visualizer.update_visualization(frame)
        return artists
        
    def _draw_right(self, frame):
        """Draw right hand"""
        """Draw right hand"""
        artists = []
        if self.display_mode in ['raw', 'both'] and self.right_raw_keypoints is not None:
            artists += self.right_raw_visualizer.update_visualization(frame)
        if self.display_mode in ['hand', 'both'] and self.right_hand_keypoints is not None:
            artists += self.right_hand_visualizer.update_visualization(frame)
        return artists
        
    def start_animation(self, interval=100):
        """Start dual hand animation """
 
        plt.ion()
        

        self.left_fig.show()
        self.right_fig.show()

        print("Visualizer started in manual update mode")
    
    def update_display(self):
        try:

            if self.left_raw_keypoints is not None or self.left_hand_keypoints is not None:
                self.left_ax.clear()
                self.left_raw_visualizer.setup_plot()
                self.left_hand_visualizer.setup_plot()
                self._draw_left(0)
                self.left_fig.canvas.draw_idle()
                self.left_fig.canvas.flush_events()
            
  
            if self.right_raw_keypoints is not None or self.right_hand_keypoints is not None:
                self.right_ax.clear()
                self.right_raw_visualizer.setup_plot()
                self.right_hand_visualizer.setup_plot()
                self._draw_right(0)
                self.right_fig.canvas.draw_idle()
                self.right_fig.canvas.flush_events()
        except Exception as e:
            print(f"Error updating display: {e}")

