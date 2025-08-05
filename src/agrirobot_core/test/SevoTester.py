import serial
import time

serial_port = '/dev/ttyUSB0'
baud_rate = 115200
ser_input = serial.Serial(serial_port, baud_rate, timeout=1)

def input_transfer():
        
        try:
            while True:
                input_val = input("CTRL servo (a/s/d): ").strip()
                if len(input_val) > 0:
                    ser_input.write(input_val[0].encode('utf-8'))
                time.sleep(0.1)
                if ser_input.in_waiting > 0:
                    response = ser_input.readline().decode('utf-8').strip()
                    print("Received:", response)
        except KeyboardInterrupt:
            print("User interrupted (CTRL + C)")
        finally:
            if 'ser_input'in locals() and ser_input.is_open:
                ser_input.close()
                print("Closed connection")

if __name__ == "__main__":
    input_transfer()