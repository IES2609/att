#!/usr/bin/env python3
"""
CSV Data Capture Script for nRF Device

This script reads CSV data from a serial port when the device's button is pressed.
The device prints environmental sensor data in CSV format:
    timestamp_ms, accel_x_g, accel_y_g, accel_z_g, gyro_x_dps, gyro_y_dps, 
    gyro_z_dps, pressure_kpa, accel_lp_x_g, accel_lp_y_g, accel_lp_z_g

For Thingy:91, the device uses 1M baud (1000000).

Usage:
    python3 capture_csv_from_device.py -p /dev/ttyACM0 -b 1000000
    python3 capture_csv_from_device.py -p COM3 -b 1000000
    python3 capture_csv_from_device.py --port /dev/ttyACM0 --baudrate 1000000 --output ./data/
"""

import serial
import serial.tools.list_ports
import argparse
import sys
import re
from datetime import datetime
from pathlib import Path
import time


class CSVDataCapture:
    """Captures CSV data from device serial port."""
    
    CSV_HEADER = "timestamp_ms,accel_x_g,accel_y_g,accel_z_g,gyro_x_dps,gyro_y_dps,gyro_z_dps,pressure_kpa,accel_lp_x_g,accel_lp_y_g,accel_lp_z_g"
    
    def __init__(self, port, baudrate=115200, timeout=5):
        """
        Initialize the CSV capture handler.
        
        Args:
            port: Serial port name (e.g., 'COM3' or '/dev/ttyACM0')
            baudrate: Serial communication baudrate (default: 115200)
                     Use 1000000 for Thingy:91
            timeout: Read timeout in seconds (default: 5)
        """
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.serial_conn = None
        self.csv_data = []
        self.capturing = False
        
    def connect(self):
        """Establish serial connection."""
        try:
            # For high baud rates (1M), increase timeout to get full lines
            timeout_val = 1.0 if self.baudrate >= 500000 else 0.5
            
            self.serial_conn = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=timeout_val
            )
            print(f"✓ Connected to {self.port} at {self.baudrate} baud (timeout={timeout_val}s)")
            return True
        except serial.SerialException as e:
            print(f"✗ Failed to connect to {self.port}: {e}")
            return False
    
    def disconnect(self):
        """Close serial connection."""
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
            print("✓ Disconnected from serial port")
    
    def wait_for_csv_data(self):
        """
        Wait for CSV header and capture all subsequent data rows.
        
        Returns:
            list: Captured CSV rows (including header), or None if no data received
        """
        print("\n Waiting for device to transmit CSV data...")
        print("   Press the button on the device to start data transmission")
        print("   (Listening for CSV header...)\n")
        
        self.csv_data = []
        self.capturing = False
        header_found = False
        timeout_count = 0
        # For 1M baud, data arrives very fast so we need more empty line tolerance
        # For slower bauds, use moderate timeout
        max_empty_lines = 100 if self.baudrate >= 500000 else 50  # Higher for 1M baud
        
        try:
            while timeout_count < max_empty_lines:
                line = self._read_line()
                
                if not line:
                    timeout_count += 1
                    # Print progress indicator every 10 reads
                    if timeout_count % 10 == 0:
                        print(f"  Waiting... ({timeout_count * 0.1:.1f}s)")
                    continue
                
                timeout_count = 0  # Reset timeout on any data
                
                # Check for CSV header (exact match or flexible match)
                if not header_found:
                    if line == self.CSV_HEADER:
                        print("✓ CSV header detected, capturing data...")
                        self.csv_data.append(line)
                        self.capturing = True
                        header_found = True
                        continue
                    elif "timestamp_ms" in line and "accel_x_g" in line:
                        # Flexible header matching - in case of minor formatting differences
                        print("✓ CSV header detected (flexible match), capturing data...")
                        print(f"  Header: {line}")
                        self.csv_data.append(line)
                        self.capturing = True
                        header_found = True
                        continue
                    else:
                        # Skip non-header lines before we find the header
                        continue
                
                # Capture data rows after header
                if header_found:
                    # Check if line looks like CSV data (contains comma-separated numbers)
                    if self._is_valid_csv_row(line):
                        self.csv_data.append(line)
                        row_num = len(self.csv_data) - 1
                        # Only print every 10th row to reduce clutter
                        if row_num % 10 == 0:
                            print(f"  Captured {row_num} rows so far...")
                    else:
                        # Line doesn't look like CSV, might be log message or end of data
                        if line and ("No environmental stream data" in line or "Error" in line or "error" in line.lower()):
                            print(f"\n⚠ Device message: {line}")
                            break
                        # Skip log lines (they start with letters or brackets)
                        elif line and (line[0].isalpha() or "[" in line):
                            continue
            
            if len(self.csv_data) > 1:
                print(f"\n✓ Captured {len(self.csv_data) - 1} data rows")
                return self.csv_data
            elif header_found:
                print("\n⚠ Header found but no data rows captured")
                return None
            else:
                print("\n✗ No CSV data received (timeout)")
                return None
                
        except KeyboardInterrupt:
            print("\n\n⚠ Capture interrupted by user")
            if len(self.csv_data) > 1:
                print(f"  {len(self.csv_data) - 1} rows captured before interruption")
                return self.csv_data
            return None
    
    def _read_line(self):
        """Read a line from serial port."""
        if not self.serial_conn or not self.serial_conn.is_open:
            return None
        
        try:
            line = self.serial_conn.readline()
            if line:
                # Decode and strip any trailing whitespace/newlines
                decoded = line.decode('utf-8', errors='replace').rstrip('\r\n')
                # Strip ANSI escape sequences and shell prompt
                decoded = self._strip_ansi_and_prompt(decoded)
                return decoded if decoded else None
            return None
        except serial.SerialException as e:
            print(f"✗ Serial read error: {e}")
            return None
    
    @staticmethod
    def _strip_ansi_and_prompt(line):
        """Remove ANSI escape codes and shell prompt from line."""
        # Remove ANSI escape sequences
        ansi_escape = re.compile(r'\x1b\[[0-9;]*[a-zA-Z]')
        line = ansi_escape.sub('', line)
        
        # Remove shell prompt (uart:~$ or similar)
        line = re.sub(r'^[a-z]+:~\$ ', '', line)
        line = re.sub(r'^[a-z]+@[a-z-]+:~\$ ', '', line)
        
        return line.strip()
    
    def _debug_validate_row(self, line):
        """Debug version of validation."""
        if not line or ',' not in line:
            return False, "No line or comma"
        
        try:
            parts = line.split(',')
            
            if len(parts) < 8:
                return False, f"Too few parts: {len(parts)}"
            
            first_val = float(parts[0])
            if first_val < 0:
                return False, f"Negative timestamp: {first_val}"
            
            float(parts[1])
            float(parts[2])
            
            return True, "Valid"
        except (ValueError, IndexError) as e:
            return False, f"Exception: {e}"
    
    @staticmethod
    def _is_valid_csv_row(line):
        """Check if line appears to be a valid CSV data row."""
        if not line or ',' not in line:
            return False
        
        try:
            parts = line.split(',')
            # Should have at least 8 columns (allow incomplete rows)
            # Full row has 11 columns: timestamp + 10 sensor values
            if len(parts) < 8:
                return False
            
            # First column should be numeric (timestamp)
            first_val = float(parts[0])
            # Timestamp should be reasonable (> 0 and not a text value)
            if first_val < 0:
                return False
            
            # At least 2 more columns should be numeric
            float(parts[1])
            float(parts[2])
            
            return True
        except (ValueError, IndexError):
            return False
    
    def save_to_file(self, output_dir=None):
        """
        Save captured CSV data to file.
        
        Args:
            output_dir: Output directory (default: current directory)
            
        Returns:
            str: Path to saved file, or None if save failed
        """
        if not self.csv_data or len(self.csv_data) < 2:
            print("✗ No data to save (need at least header + 1 data row)")
            return None
        
        if output_dir is None:
            output_dir = Path.cwd()
        else:
            output_dir = Path(output_dir)
        
        # Create output directory if it doesn't exist
        output_dir.mkdir(parents=True, exist_ok=True)
        
        # Generate filename with timestamp
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"sensor_data_{timestamp}.csv"
        filepath = output_dir / filename
        
        try:
            with open(filepath, 'w') as f:
                for line in self.csv_data:
                    f.write(line + '\n')
            
            print(f"\n✓ Saved {len(self.csv_data) - 1} data rows to: {filepath}")
            return str(filepath)
            
        except IOError as e:
            print(f"✗ Failed to save file: {e}")
            return None


def list_available_ports():
    """List all available serial ports."""
    ports = serial.tools.list_ports.comports()
    if not ports:
        print("✗ No serial ports found")
        return []
    
    print("\nAvailable serial ports:")
    for i, (port, desc, hwid) in enumerate(ports, 1):
        print(f"  {i}. {port:15} - {desc}")
    
    return [port for port, _, _ in ports]


def parse_arguments():
    """Parse command-line arguments."""
    parser = argparse.ArgumentParser(
        description="Capture CSV sensor data from nRF device serial port",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python3 capture_csv_from_device.py -p COM3
  python3 capture_csv_from_device.py --port /dev/ttyACM0 --baudrate 115200
  python3 capture_csv_from_device.py --list  # Show available ports
        """
    )
    
    parser.add_argument(
        '-p', '--port',
        help='Serial port name (e.g., COM3 or /dev/ttyACM0)',
        type=str
    )
    
    parser.add_argument(
        '-b', '--baudrate',
        help='Serial baudrate (default: 115200). Use 1000000 for Thingy:91',
        type=int,
        default=115200,
        choices=[9600, 38400, 57600, 115200, 230400, 1000000]
    )
    
    parser.add_argument(
        '-o', '--output',
        help='Output directory for CSV file (default: current directory)',
        type=str
    )
    
    parser.add_argument(
        '-l', '--list',
        help='List available serial ports and exit',
        action='store_true'
    )
    
    parser.add_argument(
        '-t', '--timeout',
        help='Read timeout in seconds (default: 5)',
        type=int,
        default=5
    )
    
    return parser.parse_args()


def main():
    """Main execution function."""
    args = parse_arguments()
    
    # Handle list ports request
    if args.list:
        list_available_ports()
        return 0
    
    # Validate port argument
    if not args.port:
        print("✗ Serial port required. Use -p/--port or --list to show available ports")
        return 1
    
    # Create capture handler
    capture = CSVDataCapture(args.port, args.baudrate, args.timeout)
    
    try:
        # Connect to device
        if not capture.connect():
            return 1
        
        # Wait for and capture CSV data
        csv_data = capture.wait_for_csv_data()
        if not csv_data:
            return 1
        
        # Save to file
        output_file = capture.save_to_file(args.output)
        if not output_file:
            return 1
        
        print("\n✓ CSV capture completed successfully")
        return 0
        
    except Exception as e:
        print(f"✗ Unexpected error: {e}")
        return 1
    finally:
        capture.disconnect()


if __name__ == "__main__":
    sys.exit(main())
