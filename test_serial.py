#!/usr/bin/env python3
import serial
import time

SERIAL_PORT = '/dev/ttyUSB2'
BAUD_RATE = 115200

print(f"Testing serial port {SERIAL_PORT}")

try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    print(f"Successfully connected to {SERIAL_PORT}")

    print("Waiting for data...")
    start_time = time.time()
    while time.time() - start_time < 5:
        if ser.in_waiting > 0:
            data = ser.read(ser.in_waiting).decode('utf-8', errors='ignore')
            print(f"Received data: {repr(data)}")
        time.sleep(0.1)

    ser.close()
    print("Test completed")

except Exception as e:
    print(f"Error: {e}")
