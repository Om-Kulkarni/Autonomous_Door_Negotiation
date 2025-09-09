# -*- coding: utf-8 -*-
"""
@file config_tiago.py
@brief Configuration dataclass for the TiagoClient.
"""

from dataclasses import dataclass
from lerobot.robots.config import RobotConfig

@RobotConfig.register_subclass("tiago_client")
@dataclass
class TiagoClientConfig(RobotConfig):
    """
    Configuration for the TiagoClient which connects to a Tiago robot host over TCP sockets.
    """
    # The IP address of the tiago_host.py script running on the robot.
    remote_ip: str = "10.68.0.1"
    
    # The port number for the tiago_host.py script.
    port: int = 65432
    
    # Timeout in seconds for establishing the initial connection.
    connect_timeout_s: float = 5.0

