## A simple example of how to call sensor.py

# socket_echo_client.py
import socket
import sys
import numpy as np
import time

# requesting 10 samples on each call
number_of_samples = 10


# Create a TCP/IP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# Connect the socket to the port where the server is listening
server_address = ('127.0.0.3', 10000)
print('connecting to {} port {}'.format(*server_address))
sock.connect(server_address)

while True:
    # Request 10 samples from the sensor
    message_string = str(number_of_samples)
    message = message_string.encode()
    sock.sendall(message)

    byte_data = sock.recv(10000)
    data =  np.frombuffer(byte_data)
# Clean up the connection
print('closing socket')
sock.close()
