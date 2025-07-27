import subprocesshttp://127.0.0.1:5000/

def main():
    # Run showcam.py as a subprocess and wait until it finishes
    subprocess.run(["python3", "resource/showcam.py"])

if __name__ == "__main__":
    main()
