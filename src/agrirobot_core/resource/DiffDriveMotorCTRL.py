import serial
import time
import json #JavaScript Object Notation => transfer data 
            #between server and client with structure
import pygame

# === Setup Serial ===
com = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
time.sleep(2)  # Wait for ESP32 to reset

pygame.init()  # Initialize pygame for joystick handling
pygame.joystick.init()  # Initialize the joystick module
if pygame.joystick.get_count() == 0: # Check if any joystick is connected
    print("No joystick found.")
    exit()

joy = pygame.joystick.Joystick(0)  # Use the first joystick
joy.init()  # Initialize the joystick

def Controller():
    try:
        
        while True:
            pygame.event.pump()  # Process event queue to get joystick data
            x = joy.get_axis(0)  # Get x-axis value
            y = joy.get_axis(1)  # Get y-axis value
            buttonTest = joy.get_button(0)  # Get button state (0 for not pressed, 1 for pressed)
            joystick_data = {
                "x": round(x, 3),  # Round to 3 decimal places
                "y": round(y, 3),  # Round to 3 decimal places
                "press": bool(buttonTest)  # Convert to boolean
            }
            json_data = json.dumps(joystick_data)  # Convert to JSON string
            com.write(json_data.encode('utf-8'))  # Send JSON data to ESP32
            print("Sent:", json_data)
            time.sleep(0.1)  # Sleep to avoid flooding the serial port
    except KeyboardInterrupt:
        print("User interrupted (CTRL + C)")                
    finally:
        if 'com' in locals() and com.is_open:
                    com.close()
                    print("Closed connection")      

if __name__ == "__main__":
    Controller()          