import subprocess  # Import subprocess to run other Python scripts as separate processes
import signal      # Import signal module to handle Ctrl+C interrupts
import time        # Import time module to allow sleep/delay

def main():
    # Start showcam.py as a subprocess
    cam_process = subprocess.Popen(["python3", "showcam.py"])  
    # Popen starts the script in the background and returns a Popen object to control it

    # Start MotorTest2.py as a subprocess
    motor_process = subprocess.Popen(["python3", "MotorTest2.py"])  
    # This also runs in the background independently

    try:
        # Keep the main program alive while subprocesses are running
        while True:
            # Check if both subprocesses have exited
            if cam_process.poll() is not None and motor_process.poll() is not None:
                # poll() returns None if still running, otherwise returns exit code
                break  # Exit loop if both processes finished
            
            time.sleep(0.1)  # Small delay to reduce CPU usage in this loop

    except KeyboardInterrupt:
        # Handle Ctrl+C pressed by the user
        print("\nCtrl+C pressed! Terminating subprocesses...")

        # Attempt to terminate subprocesses gently first
        for p in [cam_process, motor_process]:
            if p.poll() is None:  # Only if process is still running
                try:
                    p.terminate()  # Request process to terminate politely
                except Exception:
                    pass  # Ignore errors if terminate fails

        # Wait a short time to allow graceful termination
        time.sleep(1)

        # Force kill any subprocess that did not terminate
        for p in [cam_process, motor_process]:
            if p.poll() is None:  # Check again if still running
                try:
                    p.kill()  # Immediately kill the process
                except Exception:
                    pass  # Ignore errors if kill fails

        # Wait for subprocesses to finish completely
        for p in [cam_process, motor_process]:
            try:
                p.wait(timeout=2)  # Wait up to 2 seconds for process to exit
            except subprocess.TimeoutExpired:
                print(f"Process {p.pid} did not terminate in time.")  
                # Warn if process did not exit

        # Confirm all subprocesses terminated
        print("All subprocesses terminated safely.")

if __name__ == "__main__":
    main()  # Run the main function when the script is executed
