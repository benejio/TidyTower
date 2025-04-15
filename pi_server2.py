import socket
from pymycobot import MyCobot
import time

PORT_NAME = "/dev/ttyAMA0" 
mc = MyCobot(PORT_NAME, 1000000)

HOST = ''
PORT = 65432

def handle_command(command):
    print(f"Received command: {command}")
    parts = command.split()

    if parts[0] == "ANGLES" and len(parts) == 7:
        try:
            angles = [float(a) for a in parts[1:]]
            mc.send_angles(angles, 50)
            print(f"Moving to angles: {angles}")
        except ValueError:
            print("Invalid ANGLES parameters.")
    elif parts[0] == "GRIPPER":
        if parts[1] == "OPEN":
            mc.set_gripper_state(0, 70)
            print("Gripper opening.")
        elif parts[1] == "CLOSE":
            mc.set_gripper_state(1, 70)
            print("Gripper closing.")
        elif parts[1] == "VALUE" and len(parts) == 3:
            try:
                value = int(parts[2])
                mc.set_gripper_value(value, 70)
                print(f"Gripper set to value: {value}")
            except ValueError:
                print("Invalid GRIPPER VALUE parameter.")
        else:
            print("Invalid GRIPPER command.")
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
            try:
                data = conn.recv(1024)
                if not data:
                    print("Client disconnected.")
                    break
                command = data.decode('utf-8').strip()
                handle_command(command)
            except Exception as e:
                print("Server error:", e)
                break