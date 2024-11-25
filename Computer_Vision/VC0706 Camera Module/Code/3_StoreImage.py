import serial


PORT = "/dev/ttyACM1"
BAUDRATE = 9600 
OUTPUT_FILE = "captured_image.jpg"  

try:
    print("Opening serial connection...")
    ser = serial.Serial(PORT, BAUDRATE, timeout=10)  

    with open(OUTPUT_FILE, "wb") as img_file:
        print(f"Waiting for image data from {PORT}...")
        while True:
            data = ser.read(32)  # Read in chunks of 32 bytes
            if not data:
                break 
            print(data[:10])
            img_file.write(data)

    print(f"Image saved as {OUTPUT_FILE}")

except serial.SerialException as e:
    print(f"Error: {e}")
finally:
    if 'ser' in locals() and ser.is_open:
        ser.close()
