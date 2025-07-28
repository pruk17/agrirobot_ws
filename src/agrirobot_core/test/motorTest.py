import serial
import time
import sys
import termios
import tty

# === Setup Serial ===
ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
time.sleep(2)  # Wait for ESP32 to reset

def get_key():
    """Get one key from keyboard (non-blocking, no enter needed)"""
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        return sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)

print("Press 'w' to go forward, 's' to go backward, 'x' to stop, 'q' to quit")

try:
    while True:
        key = get_key()
        if key in ['w', 's', 'x']:
            ser.write(key.encode())
            print(f"Sent: {key}")
        elif key == 'q':
            print("Exiting...")
            break
except KeyboardInterrupt:
    print("Interrupted")
finally:
    ser.close()
