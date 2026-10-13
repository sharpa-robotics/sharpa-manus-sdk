import zmq
import sys
import os
import time
from typing import Optional
from numpy import rad2deg
sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/include")
import sharpa_hand_pb2

class MocapKeypointsReceiver:
    """MocapKeypoints消息接收者"""
    def __init__(self, address="tcp://localhost:6667"):
        self.address = address
        
        self.context = zmq.Context.instance()
        self.socket = self.context.socket(zmq.SUB)
        
        self.socket.setsockopt(zmq.RCVHWM, 1)  
        self.socket.setsockopt(zmq.LINGER, 0)    
        
        self.socket.connect(self.address)
        self.socket.setsockopt_string(zmq.SUBSCRIBE, "")
        print(f"[MocapKeypoints Receiver] Connected to {address}, no topic mode")
        print("[MocapKeypoints Receiver] Waiting for messages...")

    def receive_mocap_keypoints(self) -> Optional[sharpa_hand_pb2.MocapKeypoints]:
        """
        接收MocapKeypoints消息
        """
        try:
            if self.socket.getsockopt(zmq.RCVBUF) > 8:  
                print("[MocapKeypoints Receiver] Warning: receive buffer full, dropping old messages")
                while self.socket.getsockopt(zmq.RCVBUF) > 0:
                    try:
                        self.socket.recv(flags=zmq.NOBLOCK)
                    except zmq.Again:
                        break
            
            payload = self.socket.recv(flags=zmq.NOBLOCK)
            
            msg = sharpa_hand_pb2.MocapKeypoints()
            msg.ParseFromString(payload)  
            
            if hasattr(self, '_debug_counter'):
                self._debug_counter += 1
            else:
                self._debug_counter = 0
            
                
                
            
            return msg
                
        except zmq.Again:
            # 非阻塞模式下没有数据可读，这是正常情况
            return None
        except Exception as e:
            print(f"[MocapKeypoints Receiver] Receive failed: {e}")
            return None

    def close(self):
        self.socket.close()
        self.context.term()

class HandActionReceiver:
    """HandAction消息接收者"""
    def __init__(self, address="tcp://localhost:6666"):
        self.address = address
        
        self.context = zmq.Context.instance()
        self.socket = self.context.socket(zmq.SUB)
        
        self.socket.setsockopt(zmq.RCVHWM, 1)  
        self.socket.setsockopt(zmq.LINGER, 0)    
        
        self.socket.connect(self.address)
        self.socket.setsockopt_string(zmq.SUBSCRIBE, "")
        print(f"[HandAction Receiver] Connected to {address}, no topic mode")
        print("[HandAction Receiver] Waiting for messages...")

    def receive_hand_action(self) -> Optional[sharpa_hand_pb2.HandAction]:
        """接收HandAction消息"""
        try:
            if self.socket.getsockopt(zmq.RCVBUF) > 8:  
                print("[HandAction Receiver] Warning: receive buffer full, dropping old messages")
                while self.socket.getsockopt(zmq.RCVBUF) > 0:
                    try:
                        self.socket.recv(flags=zmq.NOBLOCK)
                    except zmq.Again:
                        break
            
            payload = self.socket.recv(flags=zmq.NOBLOCK)
            
            msg = sharpa_hand_pb2.HandAction()
            msg.ParseFromString(payload)  
            
            if hasattr(self, '_debug_counter'):
                self._debug_counter += 1
            else:
                self._debug_counter = 0
            
                
                
            
            return msg
                
        except zmq.Again:
            # 非阻塞模式下没有数据可读，这是正常情况
            return None
        except Exception as e:
            print(f"[HandAction Receiver] Receive failed: {e}")
            return None

    def close(self):
        self.socket.close()
        self.context.term()

def main():
    print("=== 启动两个接收者（无topic模式） ===")
    
    mocap_receiver = MocapKeypointsReceiver("tcp://127.0.0.1:6667")
    hand_receiver = HandActionReceiver("tcp://127.0.0.1:6668")
    
    try:
        while True:
            # 接收 MocapKeypoints 消息
            mocap_msg = mocap_receiver.receive_mocap_keypoints()
            if mocap_msg is not None:
                # 处理接收到的 MocapKeypoints 消息
                pass
            
            # 接收 HandAction (Joint) 消息
            hand_msg = hand_receiver.receive_hand_action()
            if hand_msg is not None:
                print("\n=== 接收到 Joint 数据 ===")
                print(f"时间戳: {hand_msg.header.stamp.sec}.{hand_msg.header.stamp.nanosec:09d}")
                print(f"Frame ID: {hand_msg.header.frame_id}")
                
                # 打印左手关节数据
                if len(hand_msg.joint_left.name) > 0:
                    print("\n左手关节 (Left Joints):")
                    for i, name in enumerate(hand_msg.joint_left.name):
                        pos_rad = hand_msg.joint_left.position[i] if i < len(hand_msg.joint_left.position) else 0.0
                        pos_deg = rad2deg(pos_rad)
                        print(f"  {name}: {pos_rad:.4f} rad ({pos_deg:.2f} deg)")
                
                # 打印右手关节数据
                if len(hand_msg.joint_right.name) > 0:
                    print("\n右手关节 (Right Joints):")
                    for i, name in enumerate(hand_msg.joint_right.name):
                        pos_rad = hand_msg.joint_right.position[i] if i < len(hand_msg.joint_right.position) else 0.0
                        pos_deg = rad2deg(pos_rad)
                        print(f"  {name}: {pos_rad:.4f} rad ({pos_deg:.2f} deg)")
            
            time.sleep(0.01)  
            
    except KeyboardInterrupt:
        print("\n[Receiver] User interrupted")
    finally:
        mocap_receiver.close()
        hand_receiver.close()

if __name__ == "__main__":
    main() 