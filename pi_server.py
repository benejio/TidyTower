# pi_server.py

import socket
from pymycobot import MyCobot
import time

PORT_NAME = "/dev/ttyAMA0" 
mc = MyCobot(PORT_NAME, 1000000)


HOST = ''  # Listen on all interfaces
PORT = 65432  # Arbitrary non-privileged port


def handle_command(command):
    print(f"Received command: {command}")
    parts = command.split()
    
    if parts[0] == "MOVE" and len(parts) == 4:
        try:
            x = float(parts[1])
            y = float(parts[2])
            z = float(parts[3])
            
            # Send coordinates to MyCobot
            mc.send_coords([x, y, z, 0, 0, 0], 80, 0)  # [X,Y,Z,rx,ry,rz], speed, mode
            print(f"Moving to: x={x}, y={y}, z={z}")
        except ValueError:
            print("Invalid MOVE parameters.")
    else:
        print("Unknown command.")

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.bind((HOST, PORT))
    s.listen()
    print(f"Listening on port {PORT}...")

    conn, addr = s.accept()
    with conn:
        print(f"Connected by {addr}")
        while True:
            data = conn.recv(1024)
            if not data:
                break
            command = data.decode('utf-8').strip()
            conn.sendall(data)
            handle_command(command)