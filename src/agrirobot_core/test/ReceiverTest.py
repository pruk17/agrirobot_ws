import serial

ser = serial.Serial('/dev/ttyUSB0', 115200)  # adjust port and baud rate

while True:
    line = ser.readline().decode('utf-8').strip()  # read one line from serial
    print("Received:", line)
