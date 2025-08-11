# first of all import the socket library 
import socket             


class Server:
  def __init__(self,port):
      self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
      self.port = port

  def start_stream(self):
      try:
        self.server.bind(('',self.port))
        print("Socket Server successfully created")
      except socket.error as e:
         print(f"Socket creation error: {e}")
      self.server.listen()

  def wait_connection(self):
    while True:
      print("WAITING.......")
      c, addr = self.server.accept()     
      print ('Got connection from', addr )
      # send a thank you message to the client. encoding to send byte type. 
      c.send('Thanks for connecting'.encode()) 
      c.close()

if __name__ == '__main__':
    server = Server(51003)
    server.start_stream()
    server.wait_connection()


