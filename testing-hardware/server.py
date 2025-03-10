import serial
import csv
import time

# Set your ESP32's port (Check in Arduino Serial Monitor)
SERIAL_PORT = "/dev/cu.usbserial-0001"  # Change to "/dev/ttyUSB0" on Linux or Mac
BAUD_RATE = 115200

def log_data():
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    time.sleep(2)  # Allow time for the connection

    dataset_number = 1
    files = {
        "1": open(f"sensor-1_{dataset_number}.csv", "w", newline=""),
        "2": open(f"sensor-2_{dataset_number}.csv", "w", newline=""),
    }

    writers = {
        "1": csv.writer(files["1"]),
        "2": csv.writer(files["2"]),
    }

    # Write headers
    headers = ["Timestamp", "Accel_X", "Accel_Y", "Accel_Z", "Gyro_X", "Gyro_Y", "Gyro_Z", "Pitch", "Yaw", "Roll"]
    for writer in writers.values():
        writer.writerow(headers)

    try:
        while True:
            line = ser.readline().decode("utf-8").strip()
            if not line or "Sensor" in line:
                continue
            data = line.split(",")
            sensor_num = data[0]
            writers[sensor_num].writerow(data[1:])
    except KeyboardInterrupt:
        print("Stopping data logging.")
    finally:
        for f in files.values():
            f.close()
        ser.close()

if __name__ == "__main__":
    log_data()
