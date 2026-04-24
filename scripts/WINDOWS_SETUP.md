# Windows Setup Guide

Complete step-by-step instructions for running the CSV capture script on Windows.

## Step 1: Install Python

1. Go to https://www.python.org/downloads/
2. Download **Python 3.11** or higher (click the big yellow button)
3. Run the installer
4. **IMPORTANT:** Check the box: "Add Python to PATH" (bottom of first window)
5. Click "Install Now"
6. Wait for installation to complete
7. Click "Close"

### Verify Python Installation

Open Command Prompt (Win+R, type `cmd`, press Enter) and run:
```cmd
python --version
```

You should see: `Python 3.11.x` (or higher)

---

## Step 2: Connect Your Device

1. Connect the nRF device via USB cable
2. Wait 2-3 seconds for Windows to recognize it
3. Open Device Manager (Win+X, select "Device Manager")
4. Look under "Ports (COM & LPT)" for your device
5. Note the COM port number (e.g., `COM3`, `COM5`)

---

## Step 3: Download the Script

The script folder is already in your project:
```
project/scripts/
```

---

## Step 4: Setup (Easy Method - Batch Script)

1. Open Command Prompt
2. Navigate to the scripts folder:
   ```cmd
   cd "path\to\project\scripts"
   ```
   
   Example:
   ```cmd
   cd "C:\Users\YourName\Documents\Bachelor\Code\ff\asset-tracker-template\project\scripts"
   ```

3. Run the setup script:
   ```cmd
   setup_env.bat
   ```

4. Wait for it to complete (will show green checkmarks)
5. When done, a summary will show your next command
6. Press Enter to close

**That's it!** Python virtual environment is now ready.

---

## Step 5: Run the Script

In Command Prompt, from the scripts folder:

```cmd
.venv\Scripts\python.exe capture_csv_from_device.py -p COM3 -b 1000000
```

**Replace `COM3` with your device's COM port** (check Device Manager if unsure)

### What To Expect

```
✓ Connected to COM3 at 1000000 baud (timeout=1.0s)

 Waiting for device to transmit CSV data...
   Press the button on the device to start data transmission
   (Listening for CSV header...)

✓ CSV header detected, capturing data...
  Captured 10 rows so far...
  Captured 20 rows so far...
  [data continues until button released]
```

After capture:
```
✓ Captured 7028 data rows
✓ Saved 7028 data rows to: C:\path\to\sensor_data_YYYYMMDD_HHMMSS.csv
```

---

## Step 6: Find Your Data File

The CSV file is saved in the scripts folder with a timestamp name:

```
sensor_data_20260424_153441.csv
```

You can:
- Open it with Excel
- Open it with any text editor
- Use it with Python/Pandas for analysis

---

## Troubleshooting

### "python: command not found"

**Solution:** Python wasn't added to PATH during installation
- Reinstall Python from https://www.python.org/downloads/
- **MUST check:** "Add Python to PATH" ✓

### "No serial ports found"

**Solution:** Device isn't recognized
1. Check Device Manager (Win+X → Device Manager)
2. Look in "Ports (COM & LPT)" section
3. If you see "Unknown Device" or with a ⚠, right-click and update driver
4. Restart computer

### "Failed to connect to port"

**Solution:** Wrong COM port or port is busy
1. Check Device Manager for correct COM port
2. Close any other programs using the COM port
3. Try: `.venv\Scripts\python.exe capture_csv_from_device.py --list`
   (This shows all available ports)

### "No CSV data received (timeout)"

**Solution:** Device didn't send data
1. Make sure device is powered on
2. Press the button firmly on the device
3. Try again with longer timeout: `-t 20`
   ```cmd
   .venv\Scripts\python.exe capture_csv_from_device.py -p COM3 -b 1000000 -t 20
   ```

---

## Quick Reference

### List Available Ports
```cmd
.venv\Scripts\python.exe capture_csv_from_device.py --list
```

### Run with Default Settings (115200 baud)
```cmd
.venv\Scripts\python.exe capture_csv_from_device.py -p COM3
```

### Run with 1M Baud (Thingy:91)
```cmd
.venv\Scripts\python.exe capture_csv_from_device.py -p COM3 -b 1000000
```

### Save to Specific Folder
```cmd
.venv\Scripts\python.exe capture_csv_from_device.py -p COM3 -b 1000000 -o C:\MyData\
```

### Longer Timeout (if data capture is slow)
```cmd
.venv\Scripts\python.exe capture_csv_from_device.py -p COM3 -b 1000000 -t 20
```

---

## Using PowerShell Alternative

If you prefer PowerShell instead of Command Prompt:

1. Open PowerShell (Win+X, select "Windows PowerShell")
2. Navigate to scripts folder:
   ```powershell
   cd "C:\path\to\project\scripts"
   ```
3. Run setup:
   ```powershell
   powershell -ExecutionPolicy Bypass -File setup_env.ps1
   ```
4. Run script:
   ```powershell
   .\activate_venv.ps1  # if it exists, or directly:
   .\venv\Scripts\python.exe capture_csv_from_device.py -p COM3 -b 1000000
   ```

---

## Need Help?

1. Check the main README: [README_CSV_CAPTURE.md](README_CSV_CAPTURE.md)
2. Verify Python is installed: `python --version`
3. Verify device is visible: Check Device Manager
4. Review script help: `.venv\Scripts\python.exe capture_csv_from_device.py -h`
