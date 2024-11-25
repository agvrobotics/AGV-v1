import serial

# Configure the serial connection (adjust the port to match your Arduino's port)
PORT = "/dev/ttyACM1"  # Replace with your port (e.g., COM3 on Windows, /dev/ttyUSB0 on Linux/Mac)
BAUDRATE = 9600      # Match the baud rate in your Arduino sketch
OUTPUT_FILE = "captured_image.jpg"  # Name of the file to save the image

try:
    print("Opening serial connection...")
    ser = serial.Serial(PORT, BAUDRATE, timeout=10)  # 10 seconds timeout

    # Open a file to save the image
    with open(OUTPUT_FILE, "wb") as img_file:
        print(f"Waiting for image data from {PORT}...")
        while True:
            data = ser.read(32)  # Read in chunks of 32 bytes
            if not data:
                break  # Exit when no more data is received
            print(data[:10])
            img_file.write(data)

    print(f"Image saved as {OUTPUT_FILE}")

except serial.SerialException as e:
    print(f"Error: {e}")
finally:
    if 'ser' in locals() and ser.is_open:
        ser.close()
