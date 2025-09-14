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

    def close(self):
        self.client_socket.close()
        print("Connection closed.")


if __name__ == "__main__":
    # Connect to the robot's server at IP address (replace with actual IP)
    client = TCPClient("10.68.0.1", 51003)  # Replace with robot's IP
    client.connect()

    # Joint states to send (7 joints in total)
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
    ]

    for joint_state in joint_states:
        message = json.dumps(joint_state)
        if not client.send_message(message):
            print("Re-trying to send message...")
            time.sleep(1)
            continue  # Try sending the message again if failure occurs
        time.sleep(5)  # Wait 2 seconds between sending each message

    client.close()
