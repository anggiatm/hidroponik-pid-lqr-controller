import threading
import time
from gpiozero import PWMLED
import serial
import numpy as np
from scipy.linalg import solve_continuous_are

class SensorThread(threading.Thread):
    def __init__(self, ser, ppm_callback, ph_callback, loop_interval):
        threading.Thread.__init__(self)
        self.ser = ser
        self.ppm_callback = ppm_callback
        self.ph_callback = ph_callback
        self.loop_interval = loop_interval

    def run(self):
        while True:
            try:
                # Read data from the serial port
                raw_data = self.ser.readline()
                
                # Decode data using utf-8 with error handling
                data = raw_data.decode('utf-8', errors='replace').rstrip()
                if 'TDS: ' in data:
                    data = data.split('TDS: ')
                    data = data[1].split(',')

                    ppm = round(float(data[0]), 2)
                    ph = round(float(data[1].split('pH: ')[1]), 2)
                    # print(ppm, ph)
                    self.ppm_callback(ppm)
                    self.ph_callback(ph)

                # Menunggu sebelum membaca data berikutnya
                time.sleep(self.loop_interval)

            except serial.SerialException as e:
                print(f"Error reading data from serial port: {e}")

class DelayThread(threading.Thread):
    def __init__(self, motor1, motor2, motor3, motor4, target_ppm, target_ph, ppm_deadband, ph_deadband, K, loop_interval):
        threading.Thread.__init__(self)
        self.motor1 = motor1
        self.motor2 = motor2
        self.motor3 = motor3
        self.motor4 = motor4
        self.target_ppm = target_ppm
        self.target_ph = target_ph
        self.ppm_deadband = ppm_deadband
        self.ph_deadband = ph_deadband
        self.K = K
        self.loop_interval = loop_interval
        self.ppm_value = 0.0
        self.ph_value = 0.0

    def ppm_callback(self, ppm):
        self.ppm_value = ppm

    def ph_callback(self, ph):
        self.ph_value = ph

    def run(self):
        while True:
            try:
                x0 = np.array([self.ppm_value, self.ph_value])
                x_target = np.array([self.target_ppm, self.target_ph])

                u = -K @ (x0 - x_target)
                x_dot = A @ x0 + B @ u

                # Check if ppm is within deadband
                if abs(self.ppm_value - self.target_ppm) <= self.ppm_deadband:
                    control_ppm = 0
                else:
                    control_ppm = round(x_dot[0]) / 1000000
                
                # Check if pH is within deadband
                if abs(self.ph_value - self.target_ph) <= self.ph_deadband:
                    control_ph = 0
                else:
                    control_ph = round(x_dot[1]) / 18500

                if (self.ph_value > 0 and self.ppm_value > 0):
                    print("real ppm: ", self.ppm_value, "real ph: ", self.ph_value, " | ",
                    "target ppm: ", self.target_ppm, "target ph: ", self.target_ph, " | ",
                    "control ppm: ", control_ppm, "control ph: ", control_ph)

                    if (control_ppm > 0):
                        self.motor1.value = 1
                        print("motor 1 on")
                        time.sleep(abs(control_ppm))
                        self.motor1.value = 0
                        print("motor 1 off")
                    else:
                        self.motor2.value = 1
                        print("motor 2 on")
                        time.sleep(abs(control_ppm))
                        self.motor2.value = 0
                        print("motor 2 off")

                    if (control_ph > 0):
                        self.motor3.value = 1
                        print("motor 3 on")
                        time.sleep(abs(control_ph))
                        self.motor3.value = 0
                        print("motor 3 off")
                    else:
                        self.motor4.value = 1
                        print("motor 4 on")
                        time.sleep(abs(control_ph))
                        self.motor4.value = 0
                        print("motor 4 off")

                # Menunggu sebelum melakukan perhitungan LQR berikutnya
                time.sleep(self.loop_interval)

                self.motor1.value = 0
                self.motor2.value = 0
                self.motor3.value = 0
                self.motor4.value = 0

                # Menunggu sebelum melakukan perhitungan LQR berikutnya
                time.sleep(self.loop_interval)

            except Exception as e:
                print(f"Error: {e}")

if __name__ == "__main__":
    # Inisialisasi port serial dan motor PWM
    serial_port = '/dev/serial0'
    ser = serial.Serial(serial_port, 9600)

    motor1 = PWMLED(23)
    motor2 = PWMLED(24)
    motor3 = PWMLED(25)
    motor4 = PWMLED(26)

    # Inisialisasi matriks LQR dan target
    A = np.array([[1, 0], [0, 1]])
    B = np.array([[15, -0.01], [0.3, 0.5]])
    # Matriks kontrol optimal K dari LQR
    Q = np.array([[5000,0],[0,15000]])
    R = np.array([[0.03,0],[0,0.0005]])

    P = solve_continuous_are(A, B, Q, R)
    K = np.linalg.inv(R) @ B.T @ P

    target_ppm = 20
    target_ph = 6
    ppm_deadband = 20
    ph_deadband = 0.3

    # Buat thread untuk menjalankan fungsi SensorThread dan DelayThread secara terus menerus
    delay_thread = DelayThread(motor1, motor2, motor3, motor4, target_ppm, target_ph, ppm_deadband, ph_deadband, K, loop_interval=5)
    sensor_thread = SensorThread(ser, ppm_callback=delay_thread.ppm_callback, ph_callback=delay_thread.ph_callback, loop_interval=0.2)

    try:
        # Mulai kedua thread
        delay_thread.start()
        sensor_thread.start()
        

        # Tetapkan waktu yang lebih lama untuk memastikan kedua thread terus berjalan
        delay_thread.join()
        sensor_thread.join()

    except KeyboardInterrupt:
        print("Stop the program and turning off the LED")
        # Turn off motors when the program is stopped
        motor1.value = 0
        motor2.value = 0
        motor3.value = 0
        motor4.value = 0
        pass