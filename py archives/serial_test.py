import serial
import time
import platform

# Determine the correct serial port based on Raspberry Pi model
# pi_model = platform.machine()
# if pi_model == 'armv7l':
serial_port = '/dev/serial0'  # Raspberry Pi 3
# else:
#     serial_port = '/dev/ttyS0'     # Raspberry Pi 1 and 2

TDS = 0
PH = 0

# Configure the serial port
ser = serial.Serial(serial_port, 9600)

try:
    while True:
        try:
            # Read data from the serial port
            raw_data = ser.readline()
            # Decode data using utf-8 with error handling
            data = raw_data.decode('utf-8', errors='ignore').rstrip()

            # print(data)

            if 'TDS: ' in data:
                data = data.split('TDS: ')
                print(data[1].split(': ')[1])


            # Do something with the received data
            # print("Data from Arduino:", data)

            # Add data handling as needed for your project

        except serial.SerialException as e:
            print(f"Error reading data from serial port: {e}")

        # Wait a short time before reading the next data
        # time.sleep(0.2)

except KeyboardInterrupt:
    # Close the serial connection when the program is stopped with Ctrl+C
    ser.close()
    print("Serial connection closed.")
