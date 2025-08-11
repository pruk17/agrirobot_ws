import subprocess
url = "http://127.0.0.1:5000/"

def main():
    # Run showcam.py as a subprocess and wait until it finishes
    subprocess.run(["python3", "showcam.py"])

if __name__ == "__main__":
    main()


#ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0

