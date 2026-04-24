# CSV Data Capture Script

This script captures CSV sensor data from the nRF asset tracker device when a button is pressed. The device transmits environmental sensor data (accelerometer, gyroscope, pressure) via serial port.

**Supports:** Windows, Linux, macOS

## Prerequisites

### Windows

1. Install Python 3.8 or higher from https://www.python.org/
   - **Important:** Check "Add Python to PATH" during installation
2. Connect the device via USB
3. Find your serial port in Device Manager (e.g., `COM3`, `COM4`, etc.)

### Linux

Install Python and venv:
```bash
sudo apt install python3-venv python3-dev
```

The device typically appears as `/dev/ttyACM0` or `/dev/ttyUSB0`

### macOS

Python 3 is usually pre-installed. If not:
```bash
brew install python3
```

The device typically appears as `/dev/tty.usbmodem*` or `/dev/cu.usbmodem*`

---

## Setup (First Time Only)

### On Windows

**Option 1: Batch Script (Easiest)**
```batch
cd project\scripts
setup_env.bat
```

**Option 2: PowerShell**
```powershell
cd project\scripts
powershell -ExecutionPolicy Bypass -File setup_env.ps1
```

**Option 3: Manual Setup**
```batch
python -m venv .venv
.venv\Scripts\activate.bat
pip install -r requirements_csv_capture.txt
```

### On Linux/macOS

```bash
cd project/scripts
bash setup_env.sh
source .venv/bin/activate
```

---

## Usage

### Find Your Serial Port

**List all available ports:**
```bash
# Windows (Command Prompt)
python capture_csv_from_device.py --list

# Linux/macOS
python3 capture_csv_from_device.py --list
```

Look for your device (usually labeled as "Serial Port" or with your device name).

### Basic Usage - Thingy:91 X (1M Baud)

**Windows:**
```batch
.venv\Scripts\python.exe capture_csv_from_device.py -p COM3 -b 1000000
```

**Linux/macOS:**
```bash
python3 capture_csv_from_device.py -p /dev/ttyACM0 -b 1000000
```

Then press the button on the device when prompted.

### Advanced Options

#### Different Serial Port
```bash
python3 capture_csv_from_device.py -p COM5 -b 1000000      # Windows
python3 capture_csv_from_device.py -p /dev/ttyUSB0 -b 115200  # Linux
```

#### Different Output Directory
```bash
python3 capture_csv_from_device.py -p COM3 -b 1000000 -o ./data/
```

#### Different Timeout (seconds)
```bash
python3 capture_csv_from_device.py -p COM3 -b 1000000 -t 15
```

#### Full Example
```bash
python3 capture_csv_from_device.py --port COM3 --baudrate 1000000 --output ./sensor_data/ --timeout 10
```

#### Supported Baudrates
- 9600, 38400, 57600, 115200, 230400, 1000000 (for Thingy:91)

---

## How It Works

1. **Connect:** Establishes serial connection to the device
2. **Listen:** Waits for CSV data transmission
3. **Detect:** Recognizes the CSV header
4. **Capture:** Collects all data rows until transmission ends
5. **Save:** Writes timestamped CSV file (e.g., `sensor_data_20260424_153441.csv`)

---

## CSV Data Format

The captured data contains 11 columns:

```
timestamp_ms,accel_x_g,accel_y_g,accel_z_g,gyro_x_dps,gyro_y_dps,gyro_z_dps,pressure_kpa,accel_lp_x_g,accel_lp_y_g,accel_lp_z_g
31018,0.051,-0.049,9.847,0.00,0.00,0.00,100.22,-0.088,0.088,-9.475
```

### Column Descriptions

| Column | Description | Unit |
|--------|-------------|------|
| `timestamp_ms` | Sample timestamp | milliseconds |
| `accel_x_g` | X-axis acceleration (high-pass filtered) | g |
| `accel_y_g` | Y-axis acceleration (high-pass filtered) | g |
| `accel_z_g` | Z-axis acceleration (high-pass filtered) | g |
| `gyro_x_dps` | X-axis angular velocity | degrees/second |
| `gyro_y_dps` | Y-axis angular velocity | degrees/second |
| `gyro_z_dps` | Z-axis angular velocity | degrees/second |
| `pressure_kpa` | Air pressure | kilopascals |
| `accel_lp_x_g` | X-axis acceleration (low-pass filtered) | g |
| `accel_lp_y_g` | Y-axis acceleration (low-pass filtered) | g |
| `accel_lp_z_g` | Z-axis acceleration (low-pass filtered) | g |

---

## Troubleshooting

### "No serial ports found"

**Windows:**
- Check Device Manager for COM ports
- Verify device is connected via USB
- Try reinstalling device drivers

**Linux/macOS:**
- Check device connection: `lsusb` (Linux) or `system_report SPUSBDataType` (macOS)
- Look for `/dev/tty*` ports: `ls /dev/tty*`

### "Failed to connect to port"

- Make sure port name is correct (check with `--list`)
- Verify no other application has the port open
- Try different baudrate (e.g., 115200 instead of 1000000)
- **For Thingy:91:** Must use `-b 1000000`

### "No CSV data received (timeout)"

- Press the button firmly on the device
- Ensure device is powered on and ready
- Try increasing timeout: `-t 15` or `-t 20`
- Check that device is transmitting correctly

### Serial permission denied (Linux only)

Add your user to the `dialout` group:
```bash
sudo usermod -a -G dialout $USER
```

Then log out and back in.

---

## Output Files

CSV files are automatically saved with timestamps in the current directory:
```
sensor_data_YYYYMMDD_HHMMSS.csv
```

Example: `sensor_data_20260424_153441.csv` (460 KB, 7028 rows)

---

## Tips

- Keep the output file in a known location for easy access
- Use `--output` flag to organize files in a specific directory
- Each execution creates a new file with a unique timestamp
- The CSV header is included as the first row

You can then process these files with tools like:
- **Python pandas**: `pd.read_csv('sensor_data_*.csv')`
- **Excel/LibreOffice**: Open directly
- **Gnuplot**: Plot the data
- **MATLAB**: Import and analyze

## Script Features

- ✅ Automatic port detection
- ✅ Configurable baudrate
- ✅ Timeout handling
- ✅ Automatic file naming with timestamp
- ✅ Data validation
- ✅ User-friendly console output
- ✅ Error handling and recovery
- ✅ Works on Windows, Linux, and macOS
