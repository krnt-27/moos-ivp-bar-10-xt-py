# from filterpy.kalman import KalmanFilter
# import numpy as np
# import math

# class SensorFilterPyKalman:
#     def __init__(self):
#         # Buat satu filter kalman per channel sensor (acc_x, acc_y, acc_z, gyro_x, ... etc)
#         self.filters = {}
#         channels = [
#             "accelerationX", "accelerationY", "accelerationZ",
#             "angular_velocity_x", "angular_velocity_y", "angular_velocity_z",
#             "magnetic_field_x_uT", "magnetic_field_y_uT", "magnetic_field_z_uT"
#         ]
#         for ch in channels:
#             kf = KalmanFilter(dim_x=1, dim_z=1)
#             kf.x = np.array([[0.]])  # initial state
#             kf.F = np.array([[1.]])  # state transition matrix
#             kf.H = np.array([[1.]])  # measurement function
#             kf.P *= 1000.            # covariance matrix
#             kf.R = 5                # measurement noise
#             kf.Q = 0.01             # process noise
#             self.filters[ch] = kf

#         # Filter untuk sudut roll, pitch, yaw, menggunakan sin dan cos (angle wrapping)
#         self.filters['roll_sin'] = KalmanFilter(dim_x=1, dim_z=1)
#         self.filters['roll_cos'] = KalmanFilter(dim_x=1, dim_z=1)
#         self.filters['pitch_sin'] = KalmanFilter(dim_x=1, dim_z=1)
#         self.filters['pitch_cos'] = KalmanFilter(dim_x=1, dim_z=1)
#         self.filters['yaw_sin'] = KalmanFilter(dim_x=1, dim_z=1)
#         self.filters['yaw_cos'] = KalmanFilter(dim_x=1, dim_z=1)
#         for k in ['roll_sin','roll_cos','pitch_sin','pitch_cos','yaw_sin','yaw_cos']:
#             kf = self.filters[k]
#             kf.x = np.array([[0.]])
#             kf.F = np.array([[1.]])
#             kf.H = np.array([[1.]])
#             kf.P *= 1000
#             kf.R = 5
#             kf.Q = 0.01

#     def _filter_angle(self, angle_deg, kalman_sin, kalman_cos):
#         angle_rad = math.radians(angle_deg)
#         sin_val = math.sin(angle_rad)
#         cos_val = math.cos(angle_rad)

#         kalman_sin.predict()
#         kalman_sin.update(np.array([[sin_val]]))
#         filtered_sin = kalman_sin.x[0,0]

#         kalman_cos.predict()
#         kalman_cos.update(np.array([[cos_val]]))
#         filtered_cos = kalman_cos.x[0,0]

#         filtered_angle_rad = math.atan2(filtered_sin, filtered_cos)
#         filtered_angle_deg = math.degrees(filtered_angle_rad) % 360
#         return round(filtered_angle_deg, 3)

#     def update(self, data):
#         filtered = {}
#         for ch in [
#             "accelerationX", "accelerationY", "accelerationZ",
#             "angular_velocity_x", "angular_velocity_y", "angular_velocity_z",
#             "magnetic_field_x_uT", "magnetic_field_y_uT", "magnetic_field_z_uT"
#         ]:
#             kf = self.filters[ch]
#             meas = data.get(ch, 0.0)
#             kf.predict()
#             kf.update(np.array([[meas]]))
#             filtered[ch] = round(kf.x[0,0], 4)

#         filtered["roll_madgwick"] = self._filter_angle(data.get("roll_madgwick", 0.0), self.filters['roll_sin'], self.filters['roll_cos'])
#         filtered["pitch_madgwick"] = self._filter_angle(data.get("pitch_madgwick", 0.0), self.filters['pitch_sin'], self.filters['pitch_cos'])
#         filtered["yaw_madgwick"] = self._filter_angle(data.get("yaw_madgwick", 0.0), self.filters['yaw_sin'], self.filters['yaw_cos'])

#         for key in ["roll", "pitch", "yaw", "quaternion_q0", "quaternion_q1", "quaternion_q2", "quaternion_q3", "temperature"]:
#             filtered[key] = data.get(key)

#         return filtered


import statistics
import math


class SimpleKalman1D:
    def __init__(self, process_variance=0.01, measurement_variance=0.1, estimate=0.0):
        self.q = process_variance
        self.r = measurement_variance
        self.x = estimate
        self.p = 1.0

    def update(self, measurement):
        # Predict
        self.p += self.q
        # Update
        k = self.p / (self.p + self.r)
        self.x += k * (measurement - self.x)
        self.p *= (1 - k)
        return self.x


class SensorKalman:
    def __init__(self):
        # Acceleration filters
        self.acc_x = SimpleKalman1D()
        self.acc_y = SimpleKalman1D()
        self.acc_z = SimpleKalman1D()
        # Gyro filters
        self.gyro_x = SimpleKalman1D()
        self.gyro_y = SimpleKalman1D()
        self.gyro_z = SimpleKalman1D()
        # Mag filters
        self.mag_x = SimpleKalman1D()
        self.mag_y = SimpleKalman1D()
        self.mag_z = SimpleKalman1D()
        #angles: sin & cos separate
        self.roll_sin = SimpleKalman1D()
        self.roll_cos = SimpleKalman1D()
        self.pitch_sin = SimpleKalman1D()
        self.pitch_cos = SimpleKalman1D()
        self.yaw_sin = SimpleKalman1D()
        self.yaw_cos = SimpleKalman1D()

        # self.last_yaw = 0.0

    def _filter_angle(self, angle_deg, kalman_sin, kalman_cos):
        angle_rad = math.radians(angle_deg)
        sin_val = math.sin(angle_rad)
        cos_val = math.cos(angle_rad)

        filtered_sin = kalman_sin.update(sin_val)
        filtered_cos = kalman_cos.update(cos_val)

        filtered_angle_rad = math.atan2(filtered_sin, filtered_cos)
        filtered_angle_deg = math.degrees(filtered_angle_rad) % 360
        return round(filtered_angle_deg, 3)

    def update(self, data):
        filtered = {}
        filtered["accelerationX"] = round(self.acc_x.update(data.get("accelerationX", 0.0)), 4)
        filtered["accelerationY"] = round(self.acc_y.update(data.get("accelerationY", 0.0)), 4)
        filtered["accelerationZ"] = round(self.acc_z.update(data.get("accelerationZ", 0.0)), 4)
        filtered["angular_velocity_x"] = round(self.gyro_x.update(data.get("angular_velocity_x", 0.0)), 4)
        filtered["angular_velocity_y"] = round(self.gyro_y.update(data.get("angular_velocity_y", 0.0)), 4)
        filtered["angular_velocity_z"] = round(self.gyro_z.update(data.get("angular_velocity_z", 0.0)), 4)
        filtered["magnetic_field_x_uT"] = round(self.mag_x.update(data.get("magnetic_field_x_uT", 0.0)), 4)
        filtered["magnetic_field_y_uT"] = round(self.mag_y.update(data.get("magnetic_field_y_uT", 0.0)), 4)
        filtered["magnetic_field_z_uT"] = round(self.mag_z.update(data.get("magnetic_field_z_uT", 0.0)), 4)

        filtered["roll_madgwick"] = self._filter_angle(data.get("roll_madgwick", 0.0), self.roll_sin, self.roll_cos)
        filtered["pitch_madgwick"] = self._filter_angle(data.get("pitch_madgwick", 0.0), self.pitch_sin, self.pitch_cos)
        filtered["yaw_madgwick"] = self._filter_angle(data.get("yaw_madgwick", 0.0), self.yaw_sin, self.yaw_cos)
        # filtered["yaw_madgwick_v2"] = round(statistics.mean([self._filter_angle(data.get("yaw_madgwick", 0.0), self.yaw_sin, self.yaw_cos), self.last_yaw]) , 4)
        # self.last_yaw = self._filter_angle(data.get("yaw_madgwick", 0.0), self.yaw_sin, self.yaw_cos)

        for key in ["roll", "pitch", "yaw", "quaternion_q0", "quaternion_q1", "quaternion_q2", "quaternion_q3", "temperature"]:
            filtered[key] = data.get(key)

        return filtered
