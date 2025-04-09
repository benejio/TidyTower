import socket
HOST = "192.168.66.190"  # The raspberry pi's hostname or IP address. Note this will change if you change the network connection
PORT = 65432  # The port used by the server

commands = [
    "MOVE 100 0 100",
    "MOVE 120 50 60",
    "MOVE 150 -30 80"
]

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    for cmd in commands:
        print(f"Sending: {cmd}")
        s.sendall(cmd.encode('utf-8'))
        time.sleep(2)  # give the arm time to move before sending next
    

#     s.connect((HOST, PORT))
#     s.sendall(b"Hello")
#     data = s.recv(1024)

# print(f"Received {data!r}")
