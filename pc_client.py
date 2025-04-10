import socket
import time
HOST = "192.168.59.190"  # The raspberry pi's hostname or IP address. Note this will change if you change the network connection
PORT = 65432  # The port used by the server

commands = [
    "ANGLES 0 0 0 0 0 0",
    "ANGLES 40 20 10 50 0 0",
    "ANGLES 0 0 0 0 0 0",
    "ANGLES 100 20 10 50 10 10",
    "ANGLES 0 0 0 0 0 0"
]

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.connect((HOST, PORT))
    for cmd in commands:
        print(f"Sending: {cmd}")
        s.sendall(cmd.encode('utf-8'))
        time.sleep(5)  # give the arm time to move before sending next