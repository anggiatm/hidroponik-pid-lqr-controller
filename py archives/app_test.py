from gpiozero import PWMLED
from time import sleep
import serial
import time
import numpy as np
from scipy.linalg import solve_continuous_are

# Define system dynamics
A = np.array([[1, 0], [0, 1]])  # Assuming a simple identity matrix, update based on actual dynamics
B = np.array([[0.1, 0, 0, 0], [0, -0.1, 0, 0]])  # Update based on relationships mentioned

# Q and R matrices for the LQR controller
Q = np.diag([1, 1])  # Adjust weights for state variables
R = np.diag([1, 1, 1, 1])  # Adjust weights for control inputs

# Solve the continuous-time algebraic Riccati equation
P = solve_continuous_are(A, B, Q, R)

# Compute the LQR gain matrix K
K = np.linalg.inv(R) @ B.T @ P

print("LQR Gain Matrix (K):")
print(K)

# Define target values
target_ppm = 300
target_ph = 7.5

# Define deadband values
ppm_deadband = 50
ph_deadband = 0.5

serial_port = '/dev/serial0'
ser = serial.Serial(serial_port, 9600)

motor1 = PWMLED(23)
motor2 = PWMLED(24)
motor3 = PWMLED(25)
motor4 = PWMLED(26)

ppm = 0.0
ph = 0.0

try:
    while True:
        try:
            # Read data from the serial port
            raw_data = ser.readline()
            
            # Decode data using utf-8 with error handling
            data = raw_data.decode('utf-8', errors='replace').rstrip()
            


            
            # Split the data based on the comma delimiter
            values = data.split(',')
            
            # Check if there are at least two values
            if len(values) >= 2:
                # Convert the values to floats and round to 2 decimal places
                ppm = round(float(values[0]), 2)
                ph = round(float(values[1]), 2)
                
                # Check if ppm is within deadband
                if abs(ppm - target_ppm) <= ppm_deadband:
                    control_ppm = 0.0
                else:
                    # Calculate control input for ppm using LQR
                    state_ppm = np.array([ppm, ph])  # Corrected line
                    control_ppm = -K @ state_ppm  # Corrected line
                
                # Check if pH is within deadband
                if abs(ph - target_ph) <= ph_deadband:
                    control_ph = 0.0
                else:
                    # Calculate control input for pH using LQR
                    state_ph = np.array([ppm, ph])  # Corrected line
                    control_ph = -K @ state_ph  # Corrected line
                
                # Update motor PWM values based on control inputs
                # Update motor PWM values based on control inputs
                # Update motor PWM values based on control inputs
                motor1.value = max(0, min(1, (control_ppm[0] + 1) / 2))
                motor2.value = max(0, min(1, (-control_ppm[0] + 1) / 2))

                # Ensure control_ph is a numpy array
                control_ph = np.array([0.0, 0.0]) if not isinstance(control_ph, np.ndarray) else control_ph

                motor3.value = max(0, min(1, (control_ph[1] + 1) / 2))
                motor4.value = max(0, min(1, (-control_ph[1] + 1) / 2))

                
                # Print information
                print("PPM:", ppm)
                print("pH:", ph)
                print("Control Input (Motor 1):", motor1.value)
                print("Control Input (Motor 2):", motor2.value)
                print("Control Input (Motor 3):", motor3.value)
                print("Control Input (Motor 4):", motor4.value)

                # Menunggu beberapa saat sebelum membaca data berikutnya
                time.sleep(5)  # Ganti 5 dengan jumlah detik yang diinginkan

                # Matikan motor selama penundaan
                motor1.value = 0
                motor2.value = 0
                motor3.value = 0
                motor4.value = 0

                # Menunggu beberapa saat sebelum membaca data berikutnya
                time.sleep(5)  # Ganti 5 dengan jumlah detik yang diinginkan
                
                
        except serial.SerialException as e:
            print(f"Error reading data from serial port: {e}")

        # Wait a short time before reading the next data
        time.sleep(1)
      
except KeyboardInterrupt:
    print("Stop the program and turning off the LED")
    # Turn off motors when the program is stopped
    motor1.value = 0
    motor2.value = 0
    motor3.value = 0
    motor4.value = 0
    pass
