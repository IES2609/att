# Quick Start Guide

**Cross-platform: Windows, Linux, macOS**

## 🚀 5-Minute Setup

### Windows

```batch
REM 1. Open Command Prompt and navigate to scripts folder
cd project\scripts

REM 2. Run setup (creates virtual environment)
setup_env.bat

REM 3. Run the script
.venv\Scripts\python.exe capture_csv_from_device.py -p COM3 -b 1000000

REM Replace COM3 with your device's COM port (check Device Manager)
```

### Linux / macOS

```bash
# 1. Navigate to scripts folder
cd project/scripts

# 2. Run setup (creates virtual environment)
bash setup_env.sh
source .venv/bin/activate

# 3. Run the script
python3 capture_csv_from_device.py -p /dev/ttyACM0 -b 1000000

# Replace /dev/ttyACM0 with your device port (check: ls /dev/tty*)
```

---

## 📋 Important Notes

- **Thingy:91 requires:** `-b 1000000` (1M baud)
- **Setup only needed once** - then just run step 3
- **Press the button** on your device when the script prompts

---

## 🔍 Don't Know Your Port?

```bash
# Windows (Command Prompt)
.venv\Scripts\python.exe capture_csv_from_device.py --list

# Linux/macOS
python3 capture_csv_from_device.py --list
```

Will show all available COM ports.

---

## 📊 Output

CSV file saved automatically:
```
sensor_data_20260424_153441.csv   ← Use this for analysis
```

Contains:
- Timestamp (ms)
- Accelerometer (X, Y, Z)
- Gyroscope (X, Y, Z)
- Pressure
- Low-pass filtered acceleration (X, Y, Z)

---

## ⚙️ Advanced Options

```bash
# Different output directory
-o ./my_data/

# Longer timeout (if slow device)
-t 20

# Different baudrate
-b 115200

# Full example
python3 capture_csv_from_device.py -p COM3 -b 1000000 -o ./sensor_data/ -t 15
```

---

## 📚 Full Documentation

- **Detailed setup:** See [WINDOWS_SETUP.md](WINDOWS_SETUP.md) (Windows) or [README_CSV_CAPTURE.md](README_CSV_CAPTURE.md) (all platforms)
- **Troubleshooting:** See README_CSV_CAPTURE.md → Troubleshooting section

---

## ✅ Checklist

- [ ] Python 3.8+ installed (Windows: verify with `python --version`)
- [ ] Device connected via USB
- [ ] Know your COM port (check Device Manager or use `--list`)
- [ ] Run setup script once (setup_env.bat or setup_env.sh)
- [ ] Data file created successfully

**You're ready to capture data!** 🎉
