import serial
import time
import re
import matplotlib.pyplot as plt

# ---------------------------
# CONFIGURE THESE AS NEEDED:
SERIAL_PORT = "/dev/cu.usbmodem1101"  # e.g. "COM3" on Windows
BAUD_RATE   = 115200
COLLECTION_TIME = 5.0  # seconds to gather data
# ---------------------------

# Compile regular expressions to parse lines printed by Arduino
# Example lines:
#   "AccX: 0.012 g, AccY: 0.011 g, AccZ: 1.009 g"
#   "GyroX: 1.234 °/s, GyroY: 1.234 °/s, GyroZ: 1.234 °/s"
#   "ΔAccX: 0.001 g, ΔAccY: 0.002 g, ΔAccZ: 0.003 g"
#   "ΔGyroX: 0.111 °/s, ΔGyroY: 0.222 °/s, ΔGyroZ: 0.333 °/s"

acc_pattern  = re.compile(
    r"AccX:\s*([-+]?\d*\.?\d+)\s*g,\s*AccY:\s*([-+]?\d*\.?\d+)\s*g,\s*AccZ:\s*([-+]?\d*\.?\d+)\s*g"
)
gyro_pattern = re.compile(
    r"GyroX:\s*([-+]?\d*\.?\d+)\s*°/s,\s*GyroY:\s*([-+]?\d*\.?\d+)\s*°/s,\s*GyroZ:\s*([-+]?\d*\.?\d+)\s*°/s"
)
dacc_pattern  = re.compile(
    r"ΔAccX:\s*([-+]?\d*\.?\d+)\s*g,\s*ΔAccY:\s*([-+]?\d*\.?\d+)\s*g,\s*ΔAccZ:\s*([-+]?\d*\.?\d+)\s*g"
)
dgyro_pattern = re.compile(
    r"ΔGyroX:\s*([-+]?\d*\.?\d+)\s*°/s,\s*ΔGyroY:\s*([-+]?\d*\.?\d+)\s*°/s,\s*ΔGyroZ:\s*([-+]?\d*\.?\d+)\s*°/s"
)

def main():
    print(f"Opening {SERIAL_PORT} at {BAUD_RATE} baud...")
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
    time.sleep(2)  # Let the Pico reset after opening the port

    input(f"Press ENTER to collect data for {COLLECTION_TIME} seconds...")
    print("Collecting data...")

    start_time = time.time()

    # Storage for each reading
    t_vals        = []
    ax_vals       = []
    ay_vals       = []
    az_vals       = []
    gx_vals       = []
    gy_vals       = []
    gz_vals       = []
    dax_vals      = []
    day_vals      = []
    daz_vals      = []
    dgx_vals      = []
    dgy_vals      = []
    dgz_vals      = []

    # We'll sample time as we parse each pair of lines
    while (time.time() - start_time) < COLLECTION_TIME:
        line = ser.readline().decode("utf-8").strip()
        if not line:
            continue  # no data this pass

        # Check which type of line this is:
        acc_match  = acc_pattern.search(line)
        gyro_match = gyro_pattern.search(line)
        dacc_match = dacc_pattern.search(line)
        dgyro_match= dgyro_pattern.search(line)

        if acc_match:
            # This line has current acceleration data
            ax, ay, az = map(float, acc_match.groups())
            ax_vals.append(ax)
            ay_vals.append(ay)
            az_vals.append(az)
            
            # We'll store a time value for every reading. Alternatively,
            # you can store time only once per cycle if you want.
            t_vals.append(time.time() - start_time)

        elif gyro_match:
            # This line has current gyro data
            gx, gy, gz = map(float, gyro_match.groups())
            gx_vals.append(gx)
            gy_vals.append(gy)
            gz_vals.append(gz)

        elif dacc_match:
            # This line has delta acceleration
            dax, day, daz = map(float, dacc_match.groups())
            dax_vals.append(dax)
            day_vals.append(day)
            daz_vals.append(daz)

        elif dgyro_match:
            # This line has delta gyro
            dgx, dgy, dgz = map(float, dgyro_match.groups())
            dgx_vals.append(dgx)
            dgy_vals.append(dgy)
            dgz_vals.append(dgz)

        # Lines like "=== Current Readings ===" or "=== Delta Readings ===" are skipped

    ser.close()
    print("Data collection complete.")

    # Convert all lists to the same length, if they differ
    # (Because the Arduino prints them in separate lines, the arrays may differ by a few samples.)
    # We'll simply handle them carefully in plotting.

    # -----------
    # Plot Accel vs Time
    # -----------
    plt.figure(figsize=(10, 8))

    plt.subplot(2, 2, 1)
    plt.title("Acceleration (g)")
    if len(t_vals) == len(ax_vals):
        plt.plot(t_vals, ax_vals, label="AccX")
    if len(t_vals) == len(ay_vals):
        plt.plot(t_vals, ay_vals, label="AccY")
    if len(t_vals) == len(az_vals):
        plt.plot(t_vals, az_vals, label="AccZ")
    plt.xlabel("Time (s)")
    plt.ylabel("Acceleration (g)")
    plt.legend()

    # -----------
    # Plot Gyro vs Time
    # -----------
    # We might not have 1-to-1 with t_vals if they arrived on a different loop iteration.
    # We'll just reuse t_vals for X-axis. In practice, you might store a separate time array for gyro.
    plt.subplot(2, 2, 2)
    plt.title("Gyro (°/s)")
    # For simplicity, assume we can use the same t_vals or you can do separate time arrays.
    n_gx = min(len(t_vals), len(gx_vals))
    n_gy = min(len(t_vals), len(gy_vals))
    n_gz = min(len(t_vals), len(gz_vals))
    plt.plot(t_vals[:n_gx], gx_vals[:n_gx], label="GyroX")
    plt.plot(t_vals[:n_gy], gy_vals[:n_gy], label="GyroY")
    plt.plot(t_vals[:n_gz], gz_vals[:n_gz], label="GyroZ")
    plt.xlabel("Time (s)")
    plt.ylabel("Rotation (°/s)")
    plt.legend()

    # -----------
    # Plot Delta Accel
    # -----------
    # Similarly, there's no direct time stamping for delta lines. We'll just use the existing t_vals indexing.
    plt.subplot(2, 2, 3)
    plt.title("ΔAcceleration (g)")
    ndax = min(len(t_vals), len(dax_vals))
    nday = min(len(t_vals), len(day_vals))
    ndaz = min(len(t_vals), len(daz_vals))
    plt.plot(t_vals[:ndax], dax_vals[:ndax], label="ΔAccX")
    plt.plot(t_vals[:nday], day_vals[:nday], label="ΔAccY")
    plt.plot(t_vals[:ndaz], daz_vals[:ndaz], label="ΔAccZ")
    plt.xlabel("Time (s)")
    plt.ylabel("ΔAcceleration (g)")
    plt.legend()

    # -----------
    # Plot Delta Gyro
    # -----------
    plt.subplot(2, 2, 4)
    plt.title("ΔGyro (°/s)")
    ndgx = min(len(t_vals), len(dgx_vals))
    ndgy = min(len(t_vals), len(dgy_vals))
    ndgz = min(len(t_vals), len(dgz_vals))
    plt.plot(t_vals[:ndgx], dgx_vals[:ndgx], label="ΔGyroX")
    plt.plot(t_vals[:ndgy], dgy_vals[:ndgy], label="ΔGyroY")
    plt.plot(t_vals[:ndgz], dgz_vals[:ndgz], label="ΔGyroZ")
    plt.xlabel("Time (s)")
    plt.ylabel("ΔRotation (°/s)")
    plt.legend()

    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
