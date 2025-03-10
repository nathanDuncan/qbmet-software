import socket
import csv
from datetime import datetime

HOST = '0.0.0.0'  # Listen on all interfaces
PORT = 12345

def save_to_csv(data):
    with open('sensor_data.csv', 'a', newline='') as f:
        writer = csv.writer(f)
        writer.writerow([datetime.now().isoformat()] + data)

def main():
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind((HOST, PORT))
        s.listen()
        print(f"Listening on {HOST}:{PORT}")
        
        while True:
            conn, addr = s.accept()
            print(f"Connected by {addr}")
            try:
                buffer = ''
                while True:
                    data = conn.recv(1024).decode()
                    if not data:
                        break
                    buffer += data
                    
                    while '\n' in buffer:
                        line, buffer = buffer.split('\n', 1)
                        if line.strip():
                            values = list(map(float, line.split(',')))
                            print(f"Received: {values}")
                            save_to_csv(values)
            except ConnectionResetError:
                print("Client disconnected")
            finally:
                conn.close()

if __name__ == "__main__":
    main()