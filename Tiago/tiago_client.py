#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
@file tiago_client.py
@brief Implements the client for the Tiago robot, compatible with the LeRobot framework.
@details This class acts as a proxy to the real robot. It sends actions and receives
         observations over a simple TCP socket connection.
"""

import socket
import struct
import pickle
import logging
from functools import cached_property
from typing import Any

from lerobot.robots.robot import Robot
from lerobot.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError
from config_tiago import TiagoClientConfig

class TiagoClient(Robot):
    """
    A client for the Tiago robot that communicates over TCP sockets.
    This class adheres to the lerobot.robots.Robot interface.
    """

    config_class = TiagoClientConfig
    name = "tiago_client"

    def __init__(self, config: TiagoClientConfig):
        super().__init__(config)
        self.config = config
        self._is_connected = False
        self.socket = None

    @cached_property
    def observation_features(self) -> dict[str, Any]:
        """Defines the structure of the observation dictionary."""
        return {
            # Arm
            'arm_1_joint.pos': float,
            'arm_2_joint.pos': float,
            'arm_3_joint.pos': float,
            'arm_4_joint.pos': float,
            'arm_5_joint.pos': float,
            'arm_6_joint.pos': float,
            'arm_7_joint.pos': float,
            # Torso
            'torso_lift_joint.pos': float,
            # Gripper
            'gripper_left_finger_joint.pos': float,
            'gripper_right_finger_joint.pos': float,
            # Base
            'base_linear_x.vel': float,
            'base_angular_z.vel': float,
            # TODO: Add camera keys here later, e.g., 'camera_wrist': (480, 640, 3)
        }
    
    @cached_property
    def action_features(self) -> dict[str, Any]:
        """Defines the structure of the action dictionary."""
        # This needs to EXACTLY match the keys expected by the host's send_action method.
        return {
            'arm_joint_positions': list, # Expects a list of 5 floats
            'torso_lift_joint.pos': float,
            'base_linear_velocity': float,
            'base_angular_velocity': float,
            'gripper_command': int, # 0: no-op, 1: open, 2: close
        }
    
    @property
    def is_connected(self) -> bool:
        """Indicates whether the client is currently connected to the robot host."""
        return self._is_connected
    
    def connect(self):
        """Establishes a TCP socket connection to the robot host."""
        if self.is_connected:
            raise DeviceAlreadyConnectedError("This client is already connected.")
        
        try:
            logging.info(f"Connecting to Tiago host at {self.config.remote_ip}:{self.config.port}...")
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.settimeout(self.config.connect_timeout_s)
            self.socket.connect((self.config.remote_ip, self.config.port))
            self.socket.settimeout(None) # Reset timeout for blocking operations
            self._is_connected = True
            logging.info("Successfully connected to Tiago host.")
        except Exception as e:
            self.socket = None
            raise DeviceNotConnectedError(f"Failed to connect to Tiago host: {e}")
        
    def disconnect(self):
        """Closes the TCP socket connection to the robot host."""
        if not self.is_connected:
            # It's good practice to allow disconnecting even if not connected.
            return
        
        if self.socket:
            self.socket.close()
            self.socket = None
        self._is_connected = False
        logging.info("Disconnected from Tiago host.")

    def send_action(self, action: dict[str, Any]) -> dict[str, Any]:
        """Sends an action dictionary to the robot host."""
        if not self.is_connected:
            raise DeviceNotConnectedError("Cannot send action. Client is not connected.")
        
        try:
            self._send_msg(action)
        except Exception as e:
            # Handle broken pipe or other connection errors
            self.disconnect()
            raise DeviceNotConnectedError(f"Connection to host lost while sending action: {e}")
        
        # The lerobot framework expects the sent action to be returned.
        return action
    
    def get_observation(self) -> dict[str, Any]:
        """Receives an observation dictionary from the robot host."""
        if not self.is_connected:
            raise DeviceNotConnectedError("Cannot get observation. Client is not connected.")

        try:
            observation_data = self._recv_msg()
            if observation_data is None:
                raise DeviceNotConnectedError("Host closed the connection.")
            
            observation = pickle.loads(observation_data)
            return observation
        except (socket.error, pickle.UnpicklingError) as e:
            self.disconnect()
            raise DeviceNotConnectedError(f"Connection to host lost while receiving observation: {e}")
        
    def _send_msg(self, msg):
        """Helper to serialize and send a message with a 4-byte length prefix."""
        payload = pickle.dumps(msg, protocol=2)
        header = struct.pack('>I', len(payload))
        self.socket.sendall(header + payload)

    def _recv_msg(self):
        """Helper to receive a message with a 4-byte length prefix."""
        raw_msglen = self.socket.recv(4)
        if not raw_msglen:
            return None
        msglen = struct.unpack('>I', raw_msglen)[0]
        
        data = bytearray()
        while len(data) < msglen:
            packet = self.socket.recv(msglen - len(data))
            if not packet:
                return None
            data.extend(packet)
        return data
    
    # These methods are part of the Robot base class but are not needed for a simple client.
    def calibrate(self) -> None:
        pass

    def configure(self) -> None:
        pass
    
    @property
    def is_calibrated(self) -> bool:
        return True # A client is always considered "calibrated"