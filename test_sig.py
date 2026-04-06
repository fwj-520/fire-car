#!/usr/bin/env python3
import signal
import sys
import time

def handler(signum, frame):
    print(f"Received signal: {signum}")
    sys.exit(144)

signal.signal(signal.SIGALRM, handler)
signal.signal(signal.SIGTERM, handler)
signal.signal(signal.SIGINT, handler)

print("Waiting for signals...")
while True:
    time.sleep(1)
