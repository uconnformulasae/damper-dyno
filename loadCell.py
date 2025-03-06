import serial
import csv
import time
import re
import sys

# Configuration
SERIAL_PORT = "COM3"  # Change for your OS (Windows: "COM3", Linux/macOS: "/dev/ttyUSB0")
BAUD_RATE = 115200              # Set baud rate as per your device
CSV_FILE = "adc_logger_data.csv"  # Output CSV file
MAX_LINES = 60000  # Stop after this many lines

# List USB ports and exit if the 2nd argument is list
if (len(sys.argv) > 1 and sys.argv[1] == "list"):
    print("Available ports:")
    for port in serial.tools.list_ports.comports():
        print(port.device)
    sys.exit(0)

# Define regex pattern to extract data
DATA_PATTERN = re.compile(
    r'Linear_Pot_Voltage:([\d\.-]+),\s+Displacement_mm:([\d\.-]+),\s+Velocity_mm/s:([\d\.-]+),\s+'
    r'Rotary_Pot_Voltage:([\d\.-]+),\s+Load_Cell_Raw_Value:(\d+),\s+Average_Load_Cell_Value:(\d+)'
)

def parse_serial_data(line):
    """Extracts structured serial data and returns a dictionary or None if parsing fails."""
    match = DATA_PATTERN.search(line)
    if match:
        return {
            "Linear Pot Voltage (V)": float(match.group(1)),
            "Displacement (mm)": float(match.group(2)),
            "Velocity (mm/s)": float(match.group(3)),
            "Rotary Pot Voltage (V)": float(match.group(4)),
            "Load Cell Raw Value": int(match.group(5)),
            "Average Load Cell Value": int(match.group(6))
        }
    return None

def read_serial_data(serial_port, baud_rate, csv_filename, max_lines):
    line_count = 0  # Track number of lines written

    try:
        with serial.Serial(serial_port, baud_rate, timeout=1) as ser, open(csv_filename, 'w', newline='') as csvfile:
            csv_writer = csv.writer(csvfile)
            csv_writer.writerow([
                "Timestamp", "Linear Pot Voltage (V)", "Displacement (mm)", "Velocity (mm/s)",
                "Rotary Pot Voltage (V)", "Load Cell Raw Value", "Average Load Cell Value"
            ])  # Write header

            print(f"Reading from {serial_port} at {baud_rate} baud... (Logging up to {max_lines} lines)")

            while line_count < max_lines:
                line = ser.readline().decode("utf-8", errors="ignore").strip()
                if line:
                    parsed_data = parse_serial_data(line)
                    if parsed_data:
                        timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
                        csv_writer.writerow([
                            timestamp,
                            parsed_data["Linear Pot Voltage (V)"],
                            parsed_data["Displacement (mm)"],
                            parsed_data["Velocity (mm/s)"],
                            parsed_data["Rotary Pot Voltage (V)"],
                            parsed_data["Load Cell Raw Value"],
                            parsed_data["Average Load Cell Value"]
                        ])
                        line_count += 1
                        print(f"[{line_count}/{max_lines}] {timestamp}, {parsed_data}")

            print(f"\nReached {max_lines} lines. Stopping data logging.")

    except serial.SerialException as e:
        print(f"Serial error: {e}")
    except KeyboardInterrupt:
        print("\nStopped by user.")

if __name__ == "__main__":
    read_serial_data(SERIAL_PORT, BAUD_RATE, CSV_FILE, MAX_LINES)
