import socket
import os

HOST = "192.168.0.115"  # Standard loopback interface address (localhost)
# HOST = socket.gethostbyname(socket.gethostname())

PORT = 8890  # Port to listen on (non-privileged ports are > 1023)

str = "111090\n"

print("--- Antenna Controller Server ---")
print(f"Started Server at {HOST} port {PORT}")

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.bind((HOST, PORT))
    s.listen()
    conn, addr = s.accept()
    with conn:
        print(f"Connected by {addr}")
        data = conn.sendall(str.encode('utf-8'))
        conn.close()
    s.close()