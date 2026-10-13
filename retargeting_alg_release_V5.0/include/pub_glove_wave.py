    
"""

"""

import time
import numpy as np
import sys
import threading
import socket
import struct
import zlib
from heartbeat_wave import send_heartbeat
from enum import Enum

class HandType(Enum):
    RIGHT = 0x01
    LEFT = 0x00

DEFAULT_WAVE_PORT_RIGHT = 50030  
DEFAULT_WAVE_PORT_LEFT = 50020   

def calculate_broadcast_ip(ip_address):
    """
    Calculate broadcast address for a given IP address (assuming /24 subnet)
    
    Args:
        ip_address: IP address string like "192.168.1.100"
        
    Returns:
        Broadcast address string like "192.168.1.255"
    """
    try:
        parts = ip_address.split('.')
        if len(parts) != 4:
            raise ValueError(f"Invalid IP address format: {ip_address}")
        
        # For /24 subnet, broadcast is x.x.x.255
        broadcast = f"{parts[0]}.{parts[1]}.{parts[2]}.255"
        return broadcast
    except Exception as e:
        print(f"Error calculating broadcast address for {ip_address}: {e}")
        # Fallback to common broadcast address
        return "255.255.255.255"

WAVE_NUM_JOINTS = 22  
WAVE_JOINT_ORDER = {
    "thumb_CMC_FE": 0,    
    "thumb_CMC_AA": 1,    
    "thumb_MCP_FE": 2,    
    "thumb_MCP_AA": 3,    
    "thumb_IP": 4,     
    "index_MCP_FE": 5,    
    "index_MCP_AA": 6,    
    "index_PIP": 7,     
    "index_DIP": 8,     
    "middle_MCP_FE": 9,   
    "middle_MCP_AA": 10,  
    "middle_PIP": 11,   
    "middle_DIP": 12,   
    "ring_MCP_FE": 13,    
    "ring_MCP_AA": 14,    
    "ring_PIP": 15,     
    "ring_DIP": 16,     
    "pinky_CMC": 17,    
    "pinky_MCP_FE": 18,   
    "pinky_MCP_AA": 19,   
    "pinky_PIP": 20,    
    "pinky_DIP": 21     
}

NUM_JOINTS = 22  

class Header:
    format = "<BBHBHQBB4x"  

    def __init__(self, identifier0, identifier1, protocolVersion, packetType, sequenceNumber, timestamp, operationMode, numberOfJoints):
        self.identifier0 = identifier0
        self.identifier1 = identifier1
        self.protocolVersion = protocolVersion
        self.packetType = packetType
        self.sequenceNumber = sequenceNumber
        self.timestamp = timestamp
        self.operationMode = operationMode
        self.numberOfJoints = numberOfJoints

    def pack(self):
        return struct.pack(self.format,
                          self.identifier0,
                          self.identifier1,
                          self.protocolVersion,
                          self.packetType,
                          self.sequenceNumber,
                          self.timestamp,
                          self.operationMode,
                          self.numberOfJoints)

class JointInfo:
    format = "<fff"  

    def __init__(self, angle, velocity, torque):
        self.angle = angle
        self.velocity = velocity
        self.torque = torque

    def pack(self):
        return struct.pack(self.format, self.angle, self.velocity, self.torque)

class IMUStatus:
    format = "<fff4f"  

    def __init__(self, velocity_x, velocity_y, velocity_z, quaternion):
        self.velocity_x = velocity_x
        self.velocity_y = velocity_y
        self.velocity_z = velocity_z
        self.quaternion = quaternion

    def pack(self):
        return struct.pack(self.format, self.velocity_x, self.velocity_y, self.velocity_z, *self.quaternion)

class Payload:
    def __init__(self, imuStatus, jointInfos):
        self.imuStatus = imuStatus
        self.jointInfos = jointInfos

    def pack(self):
        return self.imuStatus.pack() + b"".join(j.pack() for j in self.jointInfos)

class Tail:
    format = "<BiI"  

    def __init__(self, tdmId, tdmValue, checksum):
        self.tdmId = tdmId
        self.tdmValue = tdmValue
        self.checksum = checksum

    def pack(self):
        return struct.pack(self.format, self.tdmId, self.tdmValue, self.checksum)

class ProtocolPacket:
    def __init__(self, header, payload, tail):
        self.header = header
        self.payload = payload
        self.tail = tail

    def pack(self):
        return self.header.pack() + self.payload.pack() + self.tail.pack()

def calculate_crc32(data):
    return zlib.crc32(data) & 0xFFFFFFFF

class WAVEHeader0:
    format = "<4B"  

    def __init__(self, identifier0, identifier1, protocol_version):
        self.identifier0 = identifier0        
        self.identifier1 = identifier1        
        self.protocol_version = protocol_version  
        self.reserved = 0                     

    def pack(self):
        return struct.pack(self.format,
                         self.identifier0,
                         self.identifier1,
                         self.protocol_version,
                         self.reserved)

class WAVEHeader1:
    format = "<4B16sHHQQ"  

    def __init__(self, device_type, packet_type, payload_version, num_joints, device_sn, type_flag, sequence, timestamp):
        self.device_type = device_type          
        self.packet_type = packet_type          
        self.payload_version = payload_version  
        self.num_joints = num_joints            
        self.device_sn = device_sn.encode().ljust(16, b'\0')  
        self.type_flag = type_flag             
        self.sequence = sequence               
        self.timestamp = timestamp             
        self.reserved = 0                      

    def pack(self):
        return struct.pack(self.format,
                         self.device_type,
                         self.packet_type,
                         self.payload_version,
                         self.num_joints,
                         self.device_sn,
                         self.type_flag,
                         self.sequence,
                         self.timestamp,
                         self.reserved)

class WAVEHeader:
    def __init__(self, header0, header1):
        self.header0 = header0
        self.header1 = header1

    def pack(self):
        return self.header0.pack() + self.header1.pack()

class WAVEHandOrientation:
    format = "<7f"  

    def __init__(self, angular_velocity, quaternion):
        self.angular_velocity = angular_velocity  
        self.quaternion = quaternion  

    def pack(self):
        return struct.pack(self.format,
                          *self.angular_velocity,
                          *self.quaternion)

class WAVEJointData:
    format = "<fff"  

    def __init__(self, angle, velocity, torque):
        self.angle = angle
        self.velocity = velocity
        self.torque = torque

    def pack(self):
        return struct.pack(self.format, self.angle, self.velocity, self.torque)

class WAVEPayload:
    def __init__(self, handOrientation, jointData):
        self.handOrientation = handOrientation
        self.jointData = jointData

    def pack(self):
        return self.handOrientation.pack() + b"".join(j.pack() for j in self.jointData)

class WAVETail:
    format = "<3sBII"  

    def __init__(self, tdm_id, tdm_value, checksum):
        self.reserved = b'\x00' * 3    
        self.tdm_id = tdm_id           
        self.tdm_value = tdm_value     
        self.checksum = checksum       

    def pack(self):
        return struct.pack(self.format,
                         self.reserved,
                         self.tdm_id,
                         self.tdm_value,
                         self.checksum)

class WAVEPacket:
    def __init__(self, header, payload, tail):
        self.header = header
        self.payload = payload
        self.tail = tail

    def pack(self):
        return self.header.pack() + self.payload.pack() + self.tail.pack()

class DeviceData:
    """Holds per-device tracking data."""
    def __init__(self):
        self.joint_coords = None  
        self.joint_angles = None  
        self.timestamp = 0        
        self.is_updated = False   
        self.hand_type = "none"     
        self.palm_normal = None   
        self.palm_position = None 


class HandConfig:
    def __init__(self, device_ip, broadcast_ip, target_port):
        """
        Args:
            device_ip: Device identification IP (used in heartbeat packet)
            broadcast_ip: Broadcast IP for sending data (calculated from device_ip)
            target_port: Target port for sending joint data
        """
        self.device_ip = device_ip
        self.broadcast_ip = broadcast_ip
        self.target_port = target_port


class MockGlove():
    """UDP publisher: builds joint packets and sends them to left/right dexterous hands."""
    
    def __init__(self):
        super().__init__()
        self.devices_data = {}  
        self.active_device_ids = []  
        self.device_positions = {}  
        self.serial_to_position = {}  
        self.position_to_device_id = {}  
        self.serial_to_device_id = {}  
        self.last_refresh_time = 0  
        self.refresh_interval = 0.05  
        self.position_names = []
        
        self.hand_configs = {
        }
        self.udp_sockets = {}  
        self.sequence_numbers = {}  
        
        self.protocols = {
            "WAVE": self.send_WAVE_packet
        }
        self.device_protocols = {  
            "WAVE_RIGHT": "WAVE",
            "WAVE_LEFT": "WAVE"
        }
        
        self.verify_network_interfaces()
        
        self.lock = threading.Lock()
        
        self.previous_angles = {}  
        self.filter_alpha = 0.1    
        
    
    def verify_network_interfaces(self):
        """Optionally map local IPs to interfaces (for debugging)."""
        try:
            import netifaces
            
            interfaces = netifaces.interfaces()
            valid_ips = []
            self.ip_to_interface = {}  
            
            for iface in interfaces:
                addrs = netifaces.ifaddresses(iface)
                if netifaces.AF_INET in addrs:
                    for addr in addrs[netifaces.AF_INET]:
                        if 'addr' in addr:
                            ip = addr['addr']
                            valid_ips.append(ip)
                            self.ip_to_interface[ip] = iface
                
        except ImportError:
            self.ip_to_interface = {}
        except Exception as e:
            self.ip_to_interface = {}
    
    def create_glove(self, device_ip, target_port, hand_type):
        """
        Create a UDP socket for the given hand side.

        Args:
            device_ip: Device identity IP (used to derive /24 broadcast address).
            target_port: Destination port for joint packets.
            hand_type: HandType.LEFT or HandType.RIGHT.

        Returns:
            bool: True on success, False on failure.
        """
        try:
            # Create UDP socket
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            
            # Socket options
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)  
            
            # Bind to an ephemeral port on all interfaces
            sock.bind(('0.0.0.0', 0))  
            bound_address = sock.getsockname()
            
            # Store socket and sequence counter
            self.udp_sockets[hand_type] = sock
            self.sequence_numbers[hand_type] = 0
            
            # Broadcast address from device IP (/24)
            broadcast_ip = calculate_broadcast_ip(device_ip)
            
            # Persist config
            self.hand_configs[hand_type] = HandConfig(device_ip, broadcast_ip, target_port)
            
            print(f"UDP ready for {hand_type.name} hand: device_ip={device_ip}, broadcast={broadcast_ip}, port={target_port}")
            
            return True
            
        except Exception as e:
            print(f"Failed to create UDP socket for {hand_type}: {e}")
            import traceback
            traceback.print_exc()
            return False
    
    def limit_joint_angles(self, angles, protocol):
        limited_angles = []
        for i, angle in enumerate(angles):
            angle_deg = angle if isinstance(angle, (int, float)) else 0
            original_angle = angle_deg
            angle_deg = max(-90, min(90, angle_deg))
            limited_angles.append(angle_deg)
        
        return limited_angles
    
    def send_WAVE_packet(self, angles, hand_type):
        """Send one WAVE-format joint packet."""
        if hand_type not in self.udp_sockets:
            return False
        
        timestamp = int(time.time() * 1e6)
        
        try:
            limited_angles = self.limit_joint_angles(angles, "WAVE")
            
            sequence_number = self.sequence_numbers[hand_type]
            
            type_flag = 0x8001 if hand_type == HandType.RIGHT else 0x0001
            
            device_sn = "GlOVE-R-mock" if hand_type == HandType.RIGHT else "GlOVE-L-mock"
            
            header0 = WAVEHeader0(
                identifier0=0xbb,
                identifier1=0xee,
                protocol_version=0  
            )
            
            header1 = WAVEHeader1(
                device_type=0x01,      
                packet_type=0x00,      
                payload_version=0x00,   
                num_joints=WAVE_NUM_JOINTS,
                device_sn=device_sn,  
                type_flag=type_flag,   
                sequence=sequence_number,
                timestamp=timestamp
            )
            
            header = WAVEHeader(header0, header1)
            
            hand_orientation = WAVEHandOrientation(
                angular_velocity=[0.0, 0.0, 0.0],  
                quaternion=[1.0, 0.0, 0.0, 0.0]  
            )
            
            joint_data = []
            for i, angle in enumerate(limited_angles):
                angle_rad = angle * np.pi / 180.0
                joint_data.append(WAVEJointData(angle_rad, 0, 0))
            
            
            payload = WAVEPayload(hand_orientation, joint_data)
            
            tail = WAVETail(1, 0, 0)  
            
            packet = WAVEPacket(header, payload, tail)
            packet_data = packet.pack()
            
            
            checksum = calculate_crc32(packet_data[:-4])
            packet_data = packet_data[:-4] + struct.pack("<I", checksum)
            
            
            try:
                broadcast_ip = self.hand_configs[hand_type].broadcast_ip
                target_port = self.hand_configs[hand_type].target_port
                
                bytes_sent = self.udp_sockets[hand_type].sendto(packet_data, (broadcast_ip, target_port))
                
                self.sequence_numbers[hand_type] = (sequence_number + 1) % 65536
                
                return True
            except (socket.error, OSError) as e:
                print(f"Failed to send UDP packet: {e}")
                return False
                
        except Exception as e:
            print(f"Failed to build WAVE packet: {e}")
            import traceback
            traceback.print_exc()
            return False

def test():
    import math
    mock_glove = MockGlove()
    CONTROL_JOINT_ID = 20
    frequency = 0.5
    start_time = time.time()
    angle_send = []
    angle_receive = []
    angles = [0 for _ in range(22)]
    
    # Test with default device IPs - will auto-calculate broadcast addresses
    RIGHT_DEVICE_IP = "192.168.1.100"
    LEFT_DEVICE_IP = "192.168.1.99"
    
    if not mock_glove.create_glove(RIGHT_DEVICE_IP, DEFAULT_WAVE_PORT_RIGHT, HandType.RIGHT):
        return
    if not mock_glove.create_glove(LEFT_DEVICE_IP, DEFAULT_WAVE_PORT_LEFT, HandType.LEFT):  
        return

    
    
    frame_count = 0
    while True:
        try:
            frame_count += 1
            current_time = time.time()
            
            angle = 35 + 30 * math.sin(2 * math.pi * frequency * current_time) + 10
            angles[CONTROL_JOINT_ID] = angle
            
                
            
            if mock_glove.send_WAVE_packet(angles, HandType.RIGHT):
                if frame_count % 10 == 0:  
                    print(f"Right hand: joint {CONTROL_JOINT_ID} = {angle:.2f} deg")
            else:
                print("Right hand: send failed")
            if mock_glove.send_WAVE_packet(angles, HandType.LEFT):
                if frame_count % 10 == 0:  
                    print(f"Left hand: joint {CONTROL_JOINT_ID} = {angle:.2f} deg")
            else:
                print("Left hand: send failed")
                
            time.sleep(0.01)
        except KeyboardInterrupt:
            break
        except Exception as e:
            import traceback
            traceback.print_exc()
            break

if __name__ == "__main__":
    


    th2 = threading.Thread(target=send_heartbeat, args=('Glove-L-0002', '192.168.1.99'))
    th2.daemon = True
    th2.start()


    th = threading.Thread(target=send_heartbeat, args=('Glove-R-0002', '192.168.1.100'))
    th.daemon = True
    th.start()

    
    
    time.sleep(1)
    test()  