import socket

HOST = '0.0.0.0'  # Listen on all available interfaces
PORT = 1234       # Must match the port in the Arduino sketch

def get_dataset_filename(header_line):
    # Extract dataset name from header like "----- Dataset: Dataset_010 -----"
    parts = header_line.strip().split()
    for part in parts:
        if part.startswith("Dataset_"):
            return part + ".csv"
    return "dataset.csv"

def main():
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind((HOST, PORT))
        s.listen()
        print(f"Server listening on port {PORT}...")
        while True:
            conn, addr = s.accept()
            print(f"Connected by {addr}")
            current_file = None
            with conn:
                while True:
                    data = conn.recv(1024)
                    if not data:
                        break
                    # Decode data and split into lines
                    lines = data.decode().splitlines()
                    for line in lines:
                        print(line)
                        # Check for dataset header
                        if line.startswith("----- Dataset:"):
                            if current_file:
                                current_file.close()
                            filename = get_dataset_filename(line)
                            print(f"Starting new dataset file: {filename}")
                            current_file = open(filename, "w")
                            current_file.write(line + "\n")
                        else:
                            if current_file:
                                current_file.write(line + "\n")
                if current_file:
                    current_file.close()
                    print("Dataset file closed.")
            print("Connection closed.")

if __name__ == '__main__':
    main()
