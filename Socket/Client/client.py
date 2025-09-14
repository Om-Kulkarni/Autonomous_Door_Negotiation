# Import socket module

import socket


class Client:
    def __init__(self, ip, port):
        self.client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.port = port
        self.ip = ip

    def connect(self):
        try:
            self.client.connect((self.ip, self.port))
            print(f"Connected to server at {self.ip}:{self.port}")
        except OSError as e:
            print(f"Connection error: {e}")

    def recieve_message(self):
        try:
            str = self.client.recv(1024).decode()
            print(f"Received from server: {str}")
        except OSError as e:
            print(f"Error receiving message: {e}")

    def close(self):
        self.client.close()
        print("Connection closed.")


if __name__ == "__main__":
    client = Client("127.0.0.2", 51003)
    client.connect()
    client.recieve_message()
    client.close()
    print("THE END")
