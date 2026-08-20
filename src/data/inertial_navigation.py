import numpy as np
from math import radians, cos, sin
import time


# titik galangan kapal PAL  0mdpl, anggap
class InertialNavigation:
    def __init__(self, init_lat=-7.204133, init_lon=112.741157, init_depth=0.0):
        self.lat = init_lat
        self.lon = init_lon
        self.depth = init_depth
        self.last_time = time.time()
        self.velocity = np.array([0.0, 0.0, 0.0])  # Vx, Vy, Vz (m/s)

        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0

    def update_position(self, data):
        # global last_time

        # time_now = time.time()
        # time_diff = time_now - self.last_time
        # last_time = time_now

        time_diff = data.get("time_diff", 0.0)

        # gyroscope (derajat/detik → rad/s) menghitung kemiringan kendaaraan sampai self.yaw
        gx = radians(data.get("gyroX", 0.0))
        gy = radians(data.get("gyroY", 0.0))
        gz = radians(data.get("gyroZ", 0.0))

        # Integrasi gyro → orientasi
        self.roll += gx * time_diff
        self.pitch += gy * time_diff
        self.yaw += gz * time_diff

        # akselerasi konversi ke m/s^2
        ax = data.get("accelerationX", 0.0) * 9.80665
        ay = data.get("accelerationY", 0.0) * 9.80665
        az = data.get("accelerationZ", 0.0) * 9.80665
        # Membuang Efek Gravitasi (Baris 43 - 50)
        gX = -sin(self.pitch) * 9.80665
        gY =  sin(self.roll) * cos(self.pitch) * 9.80665
        gZ =  cos(self.roll) * cos(self.pitch) * 9.80665

        # Hilangkan gravitasi
        ax -= gX
        ay -= gY
        az -= gZ

        # Threshold
        ACC_THRESHOLD = 0.25  # m/s²
        ACC_THRESHOLD_Z = 11.0
        ax = ax if abs(ax) >= ACC_THRESHOLD else 0.0
        ay = ay if abs(ay) >= ACC_THRESHOLD else 0.0
        az = az if abs(az) >= ACC_THRESHOLD else 0.0
        # Menghitung Kecepatan dan Jarak Tempuh (Baris 59 - 67)
        # kecepatan
        self.velocity[0] += ax * time_diff  # Vx
        self.velocity[1] += ay * time_diff  # Vy
        self.velocity[2] += az * time_diff  # Vz

        # perpindahan
        dx = self.velocity[0] * time_diff
        dy = self.velocity[1] * time_diff
        dz = self.velocity[2] * time_diff  # ke bawah (+)

        # Konversi perubahan lat/lon
        delta_lat = dy / 111320
        delta_lon = dx / (40075000 * cos(radians(self.lat)) / 360)

        # Update posisi dan kedalaman
        self.lat += delta_lat
        self.lon += delta_lon
        self.depth += dz 

        # Batas kedalaman minimum
        self.depth = max(0.0, self.depth)

        return (
            round(self.lat, 7),
            round(self.lon, 7),
            round(self.depth, 3)
        )

