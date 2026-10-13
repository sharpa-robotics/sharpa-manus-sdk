import time
import zmq
import sys
import os

sys.path.append(os.path.dirname(os.path.abspath(__file__)))
import sharpa_hand_pb2

class MocapKeypointsSender:
    """MocapKeypoints消息发送者"""
    def __init__(self, address="tcp://*:6667"):
        self.address = address
        
        self.context = zmq.Context.instance()
        self.socket = self.context.socket(zmq.PUB)
        
        self.socket.setsockopt(zmq.SNDHWM, 1)  
        
        self.socket.bind(self.address)
        
        time.sleep(0.2)
        print(f"[MocapKeypoints Sender] Bound to {address}")

    def send_mocap_keypoints(self, left_poses, right_poses):
        """发送MocapKeypoints消息"""
        try:
            if self.socket.getsockopt(zmq.SNDBUF) > 8:  
                print("[MocapKeypoints Sender] Warning: send buffer full, dropping message")
                return False
            
            msg = sharpa_hand_pb2.MocapKeypoints()
            
            msg.header.stamp.sec = int(time.time())
            msg.header.stamp.nanosec = int((time.time() % 1) * 1e9)
            msg.header.frame_id = "camera_frame"
            
            for pose_data in left_poses:
                pose = sharpa_hand_pb2.Pose()
                pose.position.x = pose_data['x']
                pose.position.y = pose_data['y']
                pose.position.z = pose_data['z']
                pose.orientation.w = pose_data['qw']
                pose.orientation.x = pose_data['qx']
                pose.orientation.y = pose_data['qy']
                pose.orientation.z = pose_data['qz']
                msg.left_mocap_pose.append(pose)
            
            for pose_data in right_poses:
                pose = sharpa_hand_pb2.Pose()
                pose.position.x = pose_data['x']
                pose.position.y = pose_data['y']
                pose.position.z = pose_data['z']
                pose.orientation.w = pose_data['qw']
                pose.orientation.x = pose_data['qx']
                pose.orientation.y = pose_data['qy']
                pose.orientation.z = pose_data['qz']
                msg.right_mocap_pose.append(pose)
            
            payload = msg.SerializeToString()
            
            try:
                self.socket.send(payload, flags=zmq.NOBLOCK)
                return True
            except zmq.Again:
                print("[MocapKeypoints Sender] Send buffer full, dropping message")
                return False
            
        except Exception as e:
            print(f"[MocapKeypoints Sender] Send failed: {e}")
            return False

    def close(self):
        self.socket.close()
        self.context.term()

class HandActionSender:
    """HandAction消息发送者"""
    def __init__(self, address="tcp://*:6666"):
        self.address = address
        
        self.context = zmq.Context.instance()
        self.socket = self.context.socket(zmq.PUB)
        
        self.socket.setsockopt(zmq.SNDHWM, 1)  
        
        self.socket.bind(self.address)
        
        time.sleep(0.2)
        print(f"[HandAction Sender] Bound to {address}")

    def send_hand_action(self, left_joint_names, left_positions, right_joint_names, right_positions):
        """发送HandAction消息"""
        try:
            if self.socket.getsockopt(zmq.SNDBUF) > 8:  
                print("[HandAction Sender] Warning: send buffer full, dropping message")
                return False
            
            msg = sharpa_hand_pb2.HandAction()
            
            msg.header.stamp.sec = int(time.time())
            msg.header.stamp.nanosec = int((time.time() % 1) * 1e9)
            msg.header.frame_id = "hand_base"
            
            msg.joint_left.name.extend(left_joint_names)
            msg.joint_left.position.extend(left_positions)
            
            msg.joint_right.name.extend(right_joint_names)
            msg.joint_right.position.extend(right_positions)
            
            payload = msg.SerializeToString()
            
            try:
                self.socket.send(payload, flags=zmq.NOBLOCK)
                return True
            except zmq.Again:
                print("[HandAction Sender] Send buffer full, dropping message")
                return False
            
        except Exception as e:
            print(f"[HandAction Sender] Send failed: {e}")
            return False

    def close(self):
        self.socket.close()
        self.context.term()

def main():
    print("=== 启动两个发送者 ===")
    
    mocap_sender = MocapKeypointsSender("tcp://*:6667")
    hand_sender = HandActionSender("tcp://*:6666")
    
    try:
        left_poses = [
            {'x': 1.0, 'y': 2.0, 'z': 3.0, 'qw': 1.0, 'qx': 0.0, 'qy': 0.0, 'qz': 0.0},
            {'x': 1.1, 'y': 2.1, 'z': 3.1, 'qw': 1.0, 'qx': 0.0, 'qy': 0.0, 'qz': 0.0},
        ]
        right_poses = [
            {'x': 4.0, 'y': 5.0, 'z': 6.0, 'qw': 1.0, 'qx': 0.0, 'qy': 0.0, 'qz': 0.0},
            {'x': 4.1, 'y': 5.1, 'z': 6.1, 'qw': 1.0, 'qx': 0.0, 'qy': 0.0, 'qz': 0.0},
        ]
        
        left_joints = ["left_joint_1", "left_joint_2", "left_joint_3"]
        left_positions = [10.5, 20.3, 30.7]  
        
        right_joints = ["right_joint_1", "right_joint_2", "right_joint_3"]
        right_positions = [15.2, 25.8, 35.1]  
        
        for i in range(5):
            print(f"\n--- 发送第 {i+1} 条消息 ---")
            
            mocap_sender.send_mocap_keypoints(left_poses, right_poses)
            
            hand_sender.send_hand_action(left_joints, left_positions, right_joints, right_positions)
            
            for pose in left_poses:
                pose['x'] += 0.1
                pose['y'] += 0.1
                pose['z'] += 0.1
            
            for pose in right_poses:
                pose['x'] += 0.1
                pose['y'] += 0.1
                pose['z'] += 0.1
            
            left_positions = [pos + 1.0 for pos in left_positions]
            right_positions = [pos + 1.5 for pos in right_positions]
            
            time.sleep(1.0)
            
    except KeyboardInterrupt:
        print("\n[Sender] User interrupted")
    finally:
        mocap_sender.close()
        hand_sender.close()

if __name__ == "__main__":
    main() 