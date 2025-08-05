import serial
import time
#communicater is a variable of contain object of Serial
#serial is a name of module in serial library
#Serial is on of a class in serial (other are .tools.list_ports, .SerialException,...)
#serial.Serial is using a constructor class to create an object to connect ports
communicater = serial.Serial('/dev/ttyUSB0', 115200, timeout=2)
communicater.flush #delete junk or noise in buffer before running

try:
    while True:
        input_val = input("hello esp32: ")
        communicater.write((input_val + "\n").encode('utf-8'))
        time.sleep(0.1)
        #.in_waiting is from serial class
        if communicater.in_waiting > 0:
            response = communicater.readline().decode('utf-8').strip()
            # .strip() used for remove \n or \r from ESP32
            print("reveived")
except KeyboardInterrupt: #used for clean closing
    print("USer interrupted (CTRL +C)")
finally: #add finally after except for better exit handling
    if communicater.is_open:
        communicater.close
        print("closed connection")
