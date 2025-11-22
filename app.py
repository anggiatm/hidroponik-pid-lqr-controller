from flask import Flask, render_template, request, jsonify
import threading
import time
import serial
import numpy as np
import random
from scipy.linalg import solve_continuous_are
from gpiozero import PWMLED
from collections import deque

app = Flask(__name__)

# Global variables
ppm_values = deque(maxlen=50)
ph_values = deque(maxlen=50)
target_ppm = 20
target_ph = 5

def web(num):
    app.run(host="0.0.0.0")

class SensorThread(threading.Thread):
    def __init__(self, ser, ppm_callback, ph_callback, loop_interval):
        threading.Thread.__init__(self)
        self.ser = ser
        self.ppm_callback = ppm_callback
        self.ph_callback = ph_callback
        self.loop_interval = loop_interval
        self.ppm = 0
        self.ph = 0

    def run(self):
        while True:
            try:
                raw_data = self.ser.readline()
                data = raw_data.decode('utf-8', errors='replace').rstrip()

                # print(raw_data)
                if 'TDS: ' in data:
                    data = data.split('TDS: ')
                    data = data[1].split(',')

                    ppm = round(float(data[0]), 2)
                    ph = round(float(data[1].split('pH: ')[1]), 2)

                    #self.ppm_callback(ppm)
                    #self.ph_callback(ph)

                # time.sleep(self.loop_interval)

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
        self.ppm_value = 1165
        self.ph_value = 5.8

    def ppm_callback(self, ppm):
        self.ppm_value = ppm

    def ph_callback(self, ph):
        self.ph_value = ph

    def run(self):
        global target_ppm, target_ph
        global ppm_values, ph_values
        global control_ppm, control_ph
        while True:
            self.target_ppm = target_ppm
            self.target_ph = target_ph


            try:
                x0 = np.array([self.ppm_value, self.ph_value])
                x_target = np.array([self.target_ppm, self.target_ph])

                u = -self.K @ (x0 - x_target)
                x_dot = A @ x0 + B @ u

                if abs(self.ppm_value - self.target_ppm) <= self.ppm_deadband:
                    control_ppm = 0
                else:
                    control_ppm = round(x_dot[0]) / 1000000
                
                if abs(self.ph_value - self.target_ph) <= self.ph_deadband:
                    control_ph = 0
                else:
                    control_ph = round(x_dot[1]) / 18500

                if (self.ph_value > 0 and self.ppm_value > 0):
                    print("real ppm: ", self.ppm_value, "real ph: ", self.ph_value, " | ",
                    "target ppm: ", self.target_ppm, "target ph: ", self.target_ph, " | ",
                    "control ppm: ", control_ppm, "control ph: ", control_ph)

                    if (self.ppm_value < self.target_ppm):
                        self.motor2.value = 0
                        print("motor 2 on")
                        time.sleep(abs(control_ppm/2))
                        self.motor2.value = 1
                        print("motor 2 off")
                        self.ppm_value += abs(control_ppm * 15)
                    else:
                        self.motor1.value = 0
                        print("motor 1 on")
                        time.sleep(abs(control_ppm/2))
                        self.motor1.value = 1
                        print("motor 1 off")
                        self.ppm_value -= abs(control_ppm * 15)

                    if (self.ph_value >  self.target_ph):
                        self.motor4.value = 0
                        print("motor 4 on")
                        time.sleep(abs(control_ph/2))
                        self.motor4.value = 1
                        print("motor 4 off")
                        self.ph_value -= abs(control_ph / 20)

                    else:
                        self.motor3.value = 0
                        print("motor 3 on")
                        time.sleep(abs(control_ph/2))
                        self.motor3.value = 1
                        print("motor 3 off")
                        self.ph_value += abs(control_ph / 20)
                        
                    # Emit real-time values to the web interface

                    self.ph_value += random.uniform(-0.07, 0.07)
                    self.ppm_value += random.uniform(-25, 25)

                    ppm_values.append(round(self.ppm_value, 2))
                    ph_values.append(round(self.ph_value, 2))

                    time.sleep(self.loop_interval)
            except Exception as e:
                print(f"Error: {e}")

@app.route('/')
def index():
    return render_template('index_polling.html', target_ppm=target_ppm, target_ph=target_ph)

@app.route('/set_target', methods=['POST'])
def set_target():
    global target_ppm, target_ph
    target_ppm = float(request.form['target_ppm'])
    target_ph = float(request.form['target_ph'])
    return render_template('index_polling.html', target_ppm=target_ppm, target_ph=target_ph)

@app.route('/graph')
def graph():
    return render_template('graph_polling.html')

@app.route('/update_values', methods=['GET'])
def update_values():
    global control_ppm, control_ph
    return jsonify({
        'real_ppm': round(delay_thread.ppm_value, 2),
        'real_ph': round(delay_thread.ph_value, 2),
        'target_ppm': target_ppm,
        'target_ph': target_ph,
        'control_ppm': control_ppm, 
        'control_ph': control_ph,
        'motor_air': 0 if (delay_thread.motor1.value == 1) else 1,
        'motor_nutrisi': 0 if (delay_thread.motor2.value == 1) else 1,
        'motor_phup': 0 if (delay_thread.motor3.value == 1) else 1,
        'motor_phdown': 0 if (delay_thread.motor4.value == 1) else 1,
    })

@app.route('/update_graph', methods=['GET'])
def update_graph():
    global ppm_values, ph_values, target_ppm, target_ph
    return jsonify({
        'ppm_values': list(ppm_values),
        'ph_values': list(ph_values),
        'target_ppm': target_ppm,
        'target_ph': target_ph
    })


if __name__ == '__main__':

    serial_port = '/dev/serial0'
    ser = serial.Serial(serial_port, 9600)

    motor1 = PWMLED(23)
    motor2 = PWMLED(24)
    motor3 = PWMLED(25)
    motor4 = PWMLED(26)

    motor1.value = True
    motor2.value = True
    motor3.value = True
    motor4.value = True

    A = np.array([[1, 0], [0, 1]])
    B = np.array([[15, -0.01], [0.3, 0.5]])
    Q = np.array([[5000, 0], [0, 15000]])
    R = np.array([[0.03, 0], [0, 0.0005]])

    P = solve_continuous_are(A, B, Q, R)
    K = np.linalg.inv(R) @ B.T @ P
    

    target_ppm = 1500
    target_ph = 6.5
    ppm_deadband = 20
    ph_deadband = 0.3

    delay_thread = DelayThread(motor1, motor2, motor3, motor4, target_ppm, target_ph, ppm_deadband, ph_deadband, K, loop_interval=2)
    sensor_thread = SensorThread(ser, ppm_callback=delay_thread.ppm_callback, ph_callback=delay_thread.ph_callback, loop_interval=0.2)
    web_thread = threading.Thread(target=web, args=(10,))

    delay_thread.start()
    sensor_thread.start()
    web_thread.start()

    delay_thread.join()
    sensor_thread.join()
    web_thread.join()