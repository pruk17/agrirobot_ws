import serial
import time
import json
import sys
import termios
import tty
import pygame

# === Setup Serial Connection ===
# Open serial port at /dev/ttyUSB0 with 115200 baud rate and 1-second timeout
com = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
time.sleep(2)  # Wait 2 seconds for ESP32 to reset after serial connection opens

def get_key():
    """
    Read a single keypress from the terminal in raw mode (no Enter required).
    Returns the pressed key as a string.
    """
    fd = sys.stdin.fileno()  # File descriptor for standard input
    old_settings = termios.tcgetattr(fd)  # Save current terminal settings
    try:
        tty.setraw(fd)  # Set terminal to raw mode for instant keypress detection
        ch = sys.stdin.read(1)  # Read exactly one character from stdin
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)  # Restore terminal settings
    return ch

def keyboard_mode():
    """
    Keyboard mode loop:
    Listens for keys:
        'w' - send {"cmd": "forward"} JSON message
        's' - send {"cmd": "backward"} JSON message
        'e' - send {"cmd": "stop"} JSON message
        'q' - quit keyboard mode
    Sends JSON strings terminated by newline for ESP32 parsing.
    """
    print("Keyboard mode started. Press 'w' forward, 's' backward, 'e' brake," \
    " 'q' quit.")  
    try:
        while True:
            key = get_key()  # Get one keypress from terminal
            if key.lower() == 'w':
                msg = json.dumps({"cmd": "forward"}) + '\n'
                com.write(msg.encode('utf-8'))  # Send JSON command as bytes
                print("input: forward")
            elif key.lower() == 's':
                msg = json.dumps({"cmd": "backward"}) + '\n'
                com.write(msg.encode('utf-8'))
                print("input: backward")
            elif key.lower() == 'a':
                msg = json.dumps({"cmd": "left"}) + '\n'
                com.write(msg.encode('utf-8'))
                print("input: backward")
            elif key.lower() == 'd':
                msg = json.dumps({"cmd": "right"}) + '\n'
                com.write(msg.encode('utf-8'))
                print("input: backward")
            elif key.lower() == 'e':
                msg = json.dumps({"cmd": "stop"}) + '\n'
                com.write(msg.encode('utf-8'))
                print("input: brake")
            elif key.lower() == 'q':
                print("Quit keyboard mode.")
                break  # Exit loop to stop keyboard mode
    except KeyboardInterrupt:
        print("User interrupted (CTRL + C)")

def joystick_mode():
    """
    Joystick mode loop:
    Uses pygame to detect joystick input.
    Reads the first joystick's X and Y axis values.
    Sends JSON with rounded x,y values terminated by newline.
    Requires a joystick connected and pygame GUI environment.
    """
    pygame.init()
    pygame.joystick.init()
    if pygame.joystick.get_count() == 0:
        print("No joystick found.")
        return  # Exit if no joystick detected

    joy = pygame.joystick.Joystick(0)  # Use first joystick
    joy.init()
    print("Joystick mode started.")

    try:
        while True:
            pygame.event.pump()  # Process pygame event queue
            x = joy.get_axis(0)  # Get X-axis value of joystick
            y = joy.get_axis(1)  # Get Y-axis value of joystick

            joystick_data = {
                "x": round(x, 3), # Round to 3 decimal places
                "y": round(y, 3)
            }
            msg = json.dumps(joystick_data) + '\n'
            com.write(msg.encode('utf-8'))  # Send joystick data as JSON string
            print("Sent:", joystick_data)
            time.sleep(0.1)  # Delay to avoid flooding serial port
    except KeyboardInterrupt:
        print("User interrupted (CTRL + C)")

if __name__ == "__main__":
    # Prompt user to select mode before starting
    mode = input("Enter mode (1=Keyboard, 0=Joystick, other=Exit): ")
    if mode == "1":
        keyboard_mode()
    elif mode == "0":
        joystick_mode()
    else:
        print("Exiting program.")

    # Close serial port connection on program exit
    if com.is_open:
        com.close()
        print("Closed connection.")
