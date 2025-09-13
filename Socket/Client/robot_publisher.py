#!/usr/bin/env python
import json
import socket
import time


class TCPClient:
    def __init__(self, ip, port):
        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.ip = ip
        self.port = port

    def connect(self):
        try:
            self.client_socket.connect((self.ip, self.port))
            print(f"Connected to server at {self.ip}:{self.port}")
        except OSError as e:
            print(f"Connection error: {e}")

    def send_message(self, message):
        try:
            self.client_socket.send(message.encode())
            print(f"Sent: {message}")
        except OSError as e:
            print(f"Error sending message: {e}")
            return False  # Return False to indicate failure
        return True

    def receive_ack(self, timeout=5):
        # Set socket timeout to avoid hanging indefinitely
        self.client_socket.settimeout(timeout)
        try:
            ack = self.client_socket.recv(1024).decode()
            print(f"Received from server: {ack}")
            if ack == "ACK":
                print("Acknowledgment received from server.")
                return True
            else:
                print(f"Unexpected response from server: {ack}")
                return False
        except socket.timeout:
            print("Timeout reached while waiting for acknowledgment.")
            return False
        except OSError as e:
            print(f"Error receiving acknowledgment: {e}")
            return False

    def close(self):
        self.client_socket.close()
        print("Connection closed.")


if __name__ == "__main__":
    client = TCPClient("10.68.0.1", 51003)  # Replace with robot's IP
    client.connect()

    joint_states = [
        {
            "arm_1_joint": 1.0,
            "arm_2_joint": -0.7457,
            "arm_3_joint": -2.9648,
            "arm_4_joint": 1.7901,
            "arm_5_joint": -2.0943,
            "arm_6_joint": -0.5314,
            "arm_7_joint": -0.1771,
        },
        {
            "arm_1_joint": 0.5,
            "arm_2_joint": -0.5,
            "arm_3_joint": -2.5,
            "arm_4_joint": 1.5,
            "arm_5_joint": -2.0,
            "arm_6_joint": -0.3,
            "arm_7_joint": -0.1,
        },
        # {"arm_1_joint": 0.1, "arm_2_joint": -1.5, "arm_3_joint": -3.4, "arm_4_joint": -0.3,
        # "arm_5_joint": -2.0, "arm_6_joint": -1.3, "arm_7_joint": -2.0},
        # {"arm_1_joint": 2.6, "arm_2_joint": 1.0, "arm_3_joint": 1.5, "arm_4_joint": 2.3,
        # "arm_5_joint": 2.0, "arm_6_joint": 1.3, "arm_7_joint": 2.0},
    ]

    for joint_state in joint_states:
        message = json.dumps(joint_state)
        t = time.time()
        if not client.send_message(message):
            print("Re-trying to send message...")
            time.sleep(5)
            continue  # Try sending the message again if failure occurs
        # time.sleep(3)
        # Wait for acknowledgment from the server with a 5-second timeout
        print("Waiting for acknowledgment from the server...")
        if not client.receive_ack(timeout=5):  # Adjust timeout here
            print("Retrying the action...")
            time.sleep(2)  # Give some time before retrying
            continue  # Wait for acknowledgment before proceeding
        print(time.time() - t, "seconds elapsed for sending message and receiving ACK")
        time.sleep(2)  # Wait 2 seconds before sending the next message

    client.close()
