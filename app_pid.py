import os
from gpiozero import Device
# PERUBAHAN 1: Import MockPWMPin juga
from gpiozero.pins.mock import MockFactory, MockPWMPin 

if os.name == 'nt':
    print("⚠️  Terdeteksi Windows: Mengaktifkan mode Simulasi GPIO (PWM)")
    # PERUBAHAN 2: Tambahkan parameter pin_class=MockPWMPin
    Device.pin_factory = MockFactory(pin_class=MockPWMPin)

from flask import Flask, render_template, request, jsonify
import threading
import time
import serial
# import numpy as np  # Tidak perlu lagi untuk PID
import random
# from scipy.linalg import solve_continuous_are # Tidak perlu lagi untuk PID
from gpiozero import PWMLED
from collections import deque

app = Flask(__name__)

# --- KELAS PID BARU ---
# Kelas ini berisi logika PID
class PID:
    def __init__(self, Kp, Ki, Kd, setpoint, output_limits, sample_time=0.1):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.setpoint = setpoint
        self.output_min, self.output_max = output_limits
        self.sample_time = sample_time # Waktu minimum antar komputasi

        # --- TAMBAHAN BARU (Inisialisasi variabel monitoring) ---
        self.last_p = 0
        self.last_i = 0
        self.last_d = 0

        self.integral_sum = 0
        self.last_error = 0
        self.last_time = time.time()

    def update_setpoint(self, new_setpoint):
        """Memperbarui setpoint (target) dari luar."""
        self.setpoint = new_setpoint

    def compute(self, input_val):
        """Menghitung nilai output PID."""
        current_time = time.time()
        dt = current_time - self.last_time

        # Hanya hitung jika waktu sampel telah berlalu
        if dt < self.sample_time:
            return None

        error = self.setpoint - input_val
        
        # Proportional term
        P_term = self.Kp * error

        # Integral term (dengan anti-windup sederhana)
        self.integral_sum += error * dt
        I_term = self.Ki * self.integral_sum

        # Derivative term
        # (error - self.last_error) / dt
        derivative = (error - self.last_error) / dt
        D_term = self.Kd * derivative

        # --- TAMBAHAN BARU (Simpan nilai komponen ke self) ---
        self.last_p = P_term
        self.last_i = I_term
        self.last_d = D_term

        # Update state untuk iterasi berikutnya
        self.last_error = error
        self.last_time = current_time

        # Hitung total output
        output = P_term + I_term + D_term

        # Clamp output ke batas
        if output < self.output_min:
            output = self.output_min
        elif output > self.output_max:
            output = self.output_max

        # Anti-windup: Jika output di-clamp, sesuaikan integral
        # Ini mencegah integral "windup" saat aktuator jenuh
        if output == self.output_max and error > 0:
            self.integral_sum -= error * dt # Hentikan penambahan integral
        elif output == self.output_min and error < 0:
            self.integral_sum -= error * dt # Hentikan pengurangan integral

        return output
# --- AKHIR KELAS PID ---


# Global variables
ppm_values = deque(maxlen=50)
ph_values = deque(maxlen=50)
target_ppm = 20
target_ph = 5
# Kita perlu ini secara global agar bisa diakses oleh /update_values
control_ppm = 0
control_ph = 0


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

                if 'TDS: ' in data:
                    data = data.split('TDS: ')
                    data = data[1].split(',')

                    ppm = round(float(data[0]), 2)
                    ph = round(float(data[1].split('pH: ')[1]), 2)

                    # --- DI-AKTIFKAN ---
                    # Kirim data sensor nyata ke thread kontrol
                    # Jika Anda ingin simulasi murni, comment baris ini
                    self.ppm_callback(ppm)
                    self.ph_callback(ph)

                # time.sleep(self.loop_interval) # Sebaiknya tidak sleep di sini, biarkan readline() block

            except serial.SerialException as e:
                print(f"Error reading data from serial port: {e}")
            except Exception as e:
                print(f"Error parsing data: {e} (Data: {raw_data})")


# Ganti nama 'DelayThread' menjadi 'ControlThread' agar lebih jelas
class ControlThread(threading.Thread):
    def __init__(self, motor1, motor2, motor3, motor4, target_ppm, target_ph, ppm_deadband, ph_deadband, loop_interval):
        threading.Thread.__init__(self)
        self.motor1 = motor1
        self.motor2 = motor2
        self.motor3 = motor3
        self.motor4 = motor4
        
        self.ppm_deadband = ppm_deadband
        self.ph_deadband = ph_deadband
        self.loop_interval = loop_interval
        
        # Nilai awal untuk simulasi (akan ditimpa oleh sensor jika aktif)
        self.ppm_value = 726
        self.ph_value = 5.5

        # --- LOGIKA SIMULASI ---
        # Berdasarkan kode LQR Anda, motor yang menyala selama T detik
        # mengubah nilai dengan laju tertentu. Kita hitung laju itu.
        # PPM: T_sleep = abs(control_ppm/2), Perubahan = abs(control_ppm * 15)
        #      -> Perubahan = T_sleep * 2 * 15 = T_sleep * 30
        self.ppm_sim_rate = 30 # 30 PPM per detik motor menyala
        
        # PH: T_sleep = abs(control_ph/2), Perubahan = abs(control_ph / 20)
        #     -> Perubahan = T_sleep * 2 / 20 = T_sleep * 0.1
        self.ph_sim_rate = 0.1 # 0.1 pH per detik motor menyala
        
        # --- INISIALISASI PID ---
        # Nilai Kp, Ki, Kd
        #         
        # Output PID akan menjadi DURASI motor menyala (dalam detik)
        # Kita batasi maks 2 detik per siklus
        # Izinkan output negatif agar PID bisa merespon saat kelebihan nilai
        self.ppm_pid = PID(Kp=0.9, Ki=0.009, Kd=0.001, 
                           setpoint=target_ppm, 
                           output_limits=(-2, 2)) # Min -2, Max 2

        self.ph_pid = PID(Kp=1.5, Ki=0.3, Kd=0.05, 
                          setpoint=target_ph, 
                          output_limits=(-1.5, 1.5)) # Min -1.5, Max 1.5


    def ppm_callback(self, ppm):
        """Dipanggil oleh SensorThread untuk memperbarui nilai ppm"""
        self.ppm_value = ppm

    def ph_callback(self, ph):
        """Dipanggil oleh SensorThread untuk memperbarui nilai ph"""
        self.ph_value = ph

    def run(self):
        global target_ppm, target_ph
        global ppm_values, ph_values
        global control_ppm, control_ph
        
        while True:
            # Dapatkan target terbaru dari web UI
            current_target_ppm = target_ppm
            current_target_ph = target_ph
            
            # Update setpoint di controller PID
            self.ppm_pid.update_setpoint(current_target_ppm)
            self.ph_pid.update_setpoint(current_target_ph)

            # Hitung error
            ppm_error = current_target_ppm - self.ppm_value
            ph_error = current_target_ph - self.ph_value
            
            # --- LOGIKA KONTROL PID ---
            
            # --- LOGIKA KONTROL PPM ---
            if abs(ppm_error) <= self.ppm_deadband:
                control_ppm = 0
            else:
                control_ppm = self.ppm_pid.compute(self.ppm_value)
            
            if control_ppm is not None:
                duration = abs(control_ppm) # Ambil nilai positif untuk time.sleep

                if duration > 0.3:
                    # Cek tanda positif/negatif dari HASIL PID, bukan cuma error
                    if control_ppm > 0: 
                        # PID Positif = Butuh NAIKKAN PPM (Motor 2)
                        self.motor2.value = 0 # ON
                        time.sleep(duration)
                        self.motor2.value = 1 # OFF
                        self.ppm_value += duration * self.ppm_sim_rate # Simulasi naik

                    elif control_ppm < 0: 
                        # PID Negatif = Butuh TURUNKAN PPM (Motor 1)
                        self.motor1.value = 0 # ON
                        time.sleep(duration)
                        self.motor1.value = 1 # OFF
                        self.ppm_value -= duration * self.ppm_sim_rate # Simulasi turun
                else:
                    # Pastikan motor mati total jika dalam deadband / durasi 0
                    self.motor1.value = 1
                    self.motor2.value = 1

            # --- LOGIKA KONTROL pH ---
            if abs(ph_error) <= self.ph_deadband:
                control_ph = 0
            else:
                control_ph = self.ph_pid.compute(self.ph_value)
                
            if control_ph is not None:
                duration = abs(control_ph) # Ambil nilai positif
                if duration > 0.3:
                    if control_ph > 0: 
                        # PID Positif = Butuh NAIKKAN pH (Motor 3)
                        self.motor3.value = 0 # ON
                        time.sleep(duration)
                        self.motor3.value = 1 # OFF
                        self.ph_value += duration * self.ph_sim_rate

                    elif control_ph < 0: 
                        # PID Negatif = Butuh TURUNKAN pH (Motor 4)
                        self.motor4.value = 0 # ON
                        time.sleep(duration)
                        self.motor4.value = 1 # OFF
                        self.ph_value -= duration * self.ph_sim_rate
                else:
                    # Pastikan motor mati total jika dalam deadband / durasi 0
                    self.motor3.value = 1
                    self.motor4.value = 1


            # --- BAGIAN SIMULASI (Noise Acak) ---
            # (Comment baris ini jika pakai sensor nyata)
            self.ph_value += random.uniform(-0.04, 0.04)
            self.ppm_value += random.uniform(-18, 18)
            
            # Catat nilai untuk grafik
            ppm_values.append(round(self.ppm_value, 2))
            ph_values.append(round(self.ph_value, 2))
            
            print(f"PPM: {self.ppm_value:.2f} (Target: {current_target_ppm}) | pH: {self.ph_value:.2f} (Target: {current_target_ph})")
            print(self.motor1.value, self.motor2.value, self.motor3.value, self.motor4.value)
            
            time.sleep(self.loop_interval)


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
        'real_ppm': round(control_thread.ppm_value, 2), # Ganti delay_thread -> control_thread
        'real_ph': round(control_thread.ph_value, 2), # Ganti delay_thread -> control_thread
        'target_ppm': target_ppm,
        'target_ph': target_ph,
        'control_ppm': round(control_ppm, 2) if control_ppm is not None else 0,
        'control_ph': round(control_ph, 2) if control_ph is not None else 0,
        'motor_air': 0 if (control_thread.motor1.value == 1) else 1,
        'motor_nutrisi': 0 if (control_thread.motor2.value == 1) else 1,
        'motor_phup': 0 if (control_thread.motor3.value == 1) else 1,
        'motor_phdown': 0 if (control_thread.motor4.value == 1) else 1,
        # Mengambil nilai komponen PID yang disimpan tadi
        'tds_P': round(control_thread.ppm_pid.Kp, 5),
        'tds_I': round(control_thread.ppm_pid.Ki, 5),
        'tds_D': round(control_thread.ppm_pid.Kd, 5),
        'ph_P': round(control_thread.ph_pid.Kp, 5),
        'ph_I': round(control_thread.ph_pid.Ki, 5),
        'ph_D': round(control_thread.ph_pid.Kd, 5),
    })

@app.route('/set_pid', methods=['POST'])
def set_pid():
    # Ambil data dari JSON request atau Form data
    # Harapan format data: { 'tds_kp': 1.0, 'tds_ki': 0.1, ... }
    data = request.json or request.form

    try:
        # Update PID TDS (PPM)
        if 'tds_kp' in data: control_thread.ppm_pid.Kp = float(data['tds_kp'])
        if 'tds_ki' in data: control_thread.ppm_pid.Ki = float(data['tds_ki'])
        if 'tds_kd' in data: control_thread.ppm_pid.Kd = float(data['tds_kd'])

        # Update PID pH
        if 'ph_kp' in data: control_thread.ph_pid.Kp = float(data['ph_kp'])
        if 'ph_ki' in data: control_thread.ph_pid.Ki = float(data['ph_ki'])
        if 'ph_kd' in data: control_thread.ph_pid.Kd = float(data['ph_kd'])
        
        # Opsional: Reset integral sum agar perubahan Ki tidak membuat lonjakan tiba-tiba
        if 'reset_integral' in data and data['reset_integral'] == 'true':
            control_thread.ppm_pid.integral_sum = 0
            control_thread.ph_pid.integral_sum = 0

        return jsonify({
            "status": "success", 
            "message": "PID values updated",
            "current_tds": {
                "Kp": control_thread.ppm_pid.Kp,
                "Ki": control_thread.ppm_pid.Ki,
                "Kd": control_thread.ppm_pid.Kd
            },
            "current_ph": {
                "Kp": control_thread.ph_pid.Kp,
                "Ki": control_thread.ph_pid.Ki,
                "Kd": control_thread.ph_pid.Kd
            }
        })

    except Exception as e:
        return jsonify({"status": "error", "message": str(e)}), 400

@app.route('/update_graph', methods=['GET'])
def update_graph():
    global ppm_values, ph_values, target_ppm, target_ph
    return jsonify({
        'ppm_values': list(ppm_values),
        'ph_values': list(ph_values),
        'target_ppm': target_ppm,
        'target_ph': target_ph
    })


# if __name__ == '__main__':

#     serial_port = '/dev/serial0'
#     # ser = serial.Serial(serial_port, 9600)

#     motor1 = PWMLED(23)
#     motor2 = PWMLED(24)
#     motor3 = PWMLED(25)
#     motor4 = PWMLED(26)

#     motor1.value = True
#     motor2.value = True
#     motor3.value = True
#     motor4.value = True

#     # --- LQR DIHAPUS ---
#     # A = np.array([[1, 0], [0, 1]])
#     # B = np.array([[15, -0.01], [0.3, 0.5]])
#     # Q = np.array([[5000, 0], [0, 15000]])
#     # R = np.array([[0.03, 0], [0, 0.0005]])
#     # P = solve_continuous_are(A, B, Q, R)
#     # K = np.linalg.inv(R) @ B.T @ P
    
#     target_ppm = 1500
#     target_ph = 6.5
#     ppm_deadband = 20
#     ph_deadband = 0.1

#     # Buat thread kontrol
#     control_thread = ControlThread(motor1, motor2, motor3, motor4, target_ppm, target_ph, ppm_deadband, ph_deadband, loop_interval=2)
#     # Buat thread sensor, hubungkan callback-nya ke control_thread
#     # sensor_thread = SensorThread(ser, ppm_callback=control_thread.ppm_callback, ph_callback=control_thread.ph_callback, loop_interval=0.2)
#     web_thread = threading.Thread(target=web, args=(10,))

#     control_thread.start()
#     # sensor_thread.start()
#     web_thread.start()

#     control_thread.join()
#     # sensor_thread.join()
#     web_thread.join()

if __name__ == '__main__':
    serial_port = '/dev/serial0'

    motor1 = PWMLED(23)
    motor2 = PWMLED(24)
    motor3 = PWMLED(25)
    motor4 = PWMLED(26)

    target_ppm = 1500
    target_ph = 6.5
    ppm_deadband = 30
    ph_deadband = 0.1

    # Buat thread
    control_thread = ControlThread(motor1, motor2, motor3, motor4, target_ppm, target_ph, ppm_deadband, ph_deadband, loop_interval=2)
    web_thread = threading.Thread(target=web, args=(10,))

    # --- PERUBAHAN DISINI ---
    # Set daemon menjadi True SEBELUM start
    control_thread.daemon = True 
    web_thread.daemon = True
    
    control_thread.start()
    web_thread.start()

    # HAPUS atau COMMENT baris .join()
    # control_thread.join() 
    # web_thread.join()

    # Ganti dengan loop kosong agar program utama tetap hidup
    # sampai kita tekan Ctrl+C
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("Program berhenti...")
        # Disini Anda bisa tambahkan kode untuk mematikan motor (cleanup)
        motor1.close()
        motor2.close()
        motor3.close()
        motor4.close()