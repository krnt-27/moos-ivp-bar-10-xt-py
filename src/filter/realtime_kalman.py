from filterpy.kalman import KalmanFilter
import numpy as np
import math
import time
from datetime import datetime
from src.utils.zupt import detect_zupt, detect_xy_movement
from src.utils.preprocess import *
from ahrs.filters import Madgwick
import math
import traceback

data_init = {
    "timestamp": "",
    "latitude": 0.0,
    "longitude": 0.0,
    "filtered_latitude": 0.0,
    "filtered_longitude": 0.0,
    "speed": 0.0,
    "heading": 0.0,
    "acceleration_x": 0.0,
    "acceleration_y": 0.0,
    "acceleration_z": 0.0,
    "angular_velocity_x": 0.0,
    "angular_velocity_y": 0.0,
    "angular_velocity_z": 0.0,
    "magnetic_field_x_uT": 0.0,
    "magnetic_field_y_uT": 0.0,
    "magnetic_field_z_uT": 0.0,
    "roll": 0.0,
    "pitch": 0.0,
    "yaw": 0.0,
    "quaternion_q0": 1.0,
    "quaternion_q1": 0.0,
    "quaternion_q2": 0.0,
    "quaternion_q3": 0.0,
    "temperature": 0.0,
    "x": 0.0,
    "y": 0.0,
    "counter": 0,
}


class EKFRealtime:
    def __init__(self, acc_threshold=0.025, gyro_threshold=0.5, use_angles=True, frequency=10.0):
        # Set use_angles to False to use quaternions
        self.lat0 = None
        self.lon0 = None
        self.last_time = time.time()
        self.acc_threshold = acc_threshold
        self.velocity_threshold = 0.8
        self.gyro_threshold = gyro_threshold
        self.use_angles = use_angles  # Set use_angles to False to use quaternions
        self.frequency = frequency
        # NOTE: Rotational transform order, for extrinsic use lowercase, intrinsic use uppercase
        self.rotation_order = "xyz"
        # distance 160m
        self.manual_correction = 3.75  # 361
        # self.manual_correction = 5.0  # 340
        # self.manual_correction = 6.0  # 320
        # self.manual_correction = 8.0  #

        self.radius_imu = 0.12
        self.kf = self.kalman_filter()
        self.distance = 0.0
        self.time_start = 0.0
        self.madgwick = Madgwick(frequency=frequency, auto=False)
        self.prev_quaternion = None

    def kalman_filter(self):
        kf = KalmanFilter(dim_x=6, dim_z=4)  # x, y, vx, vy, ax_bias, ay_bias
        dt = 1.0 / self.frequency

        kf.x = np.zeros(6)

        # Transition matrix
        kf.F = np.eye(6)
        kf.F[0, 2] = dt
        kf.F[1, 3] = dt
        kf.F[2, 4] = dt
        kf.F[3, 5] = dt

        # Control matrix (acceleration input)
        kf.B = np.zeros((6, 2))
        kf.B[2, 0] = dt
        kf.B[3, 1] = dt

        kf.H = np.array(
            [
                [1, 0, 0, 0, 0, 0],  # position_x
                [0, 1, 0, 0, 0, 0],  # position_y
                [0, 0, 1, 0, 0, 0],  # velocity_x
                [0, 0, 0, 1, 0, 0],  # velocity_y
            ]
        )

        kf.R = np.diag([25.0, 25.0, 5.0, 5.0])  # measurement noise
        kf.P = np.diag([10.0, 10.0, 5.0, 5.0, 0.01, 0.01])  # initial uncertainty
        kf.Q = np.diag(
            [
                0.01,  # pos x
                0.01,  # pos y
                0.1,  # vel x
                0.1,  # vel y
                0.001,  # acc_bias x
                0.001,  # acc_bias y
            ]
        )  # process noise
        return kf

    def run_kalman_rt(self, shared_data, dt=None):
        kf = self.kf
        if dt is None:
            dt = 1.0 / self.frequency

        delta_time = round((time.time() - self.time_start), 3)
        self.time_start = time.time()

        # print(delta_time)

        # Transition Matrix
        kf.F[0, 2] = dt
        kf.F[1, 3] = dt
        kf.F[2, 4] = dt
        kf.F[3, 5] = dt

        # Control Matrix
        kf.B[2, 0] = dt
        kf.B[3, 1] = dt
        try:
            accelerations = np.array(
                [
                    float(shared_data.get("acceleration_x", 0)),
                    float(shared_data.get("acceleration_y", 0)),
                    float(shared_data.get("acceleration_z", 0)),
                ]
            )

            
            if data_init.get("counter") == 0 :
                data_init["filtered_latitude"] = shared_data["latitude"]
                data_init["filtered_longitude"] = shared_data["longitude"]
                data_init["counter"] += 1

            state = "MOVING"
            gyros = np.array(
                [
                    float(shared_data.get("angular_velocity_x", 0)),
                    float(shared_data.get("angular_velocity_y", 0)),
                    float(shared_data.get("angular_velocity_z", 0)),
                ]
            )

            state, acc_union, vel_union = detect_xy_movement(accelerations, gyros, 0.025, 0.8)
            if state == "STATIC":
                accelerations[0] = 0.0
                accelerations[1] = 0.0
            else:
                if data_init.get("counter") > 2:
                    self.distance = round(self.distance + acc_union * 9.81 * self.radius_imu, 2)
                else:
                    self.distance = 0.0
                    data_init["counter"] += 1

            inv_yaw = round((-1 * shared_data.get("yaw") + 0.0) % 360, 2)

            print(f"roll: {shared_data.get("roll")}, pitch: {shared_data.get("pitch")}, yaw: {shared_data.get("yaw")}")
            # print(f"init_heading: {shared_data.get("init_heading")}, yaw: {shared_data.get("yaw")}, accent_yaw: {inv_yaw}, veocity_union: {vel_union},distance: {self.distance } , state: {state}")

            angles = np.array(
                [
                    float(shared_data.get("roll", 0)),
                    float(shared_data.get("pitch", 0)),
                    float(shared_data.get("init_heading")),
                    # float(shared_data.get('init_heading', 0))
                    # float(yaw)
                    # float(shared_data.get("yaw", 0)),
                    # float(shared_data.get("init_heading"), float(shared_data.get("yaw")))
                    # float(shared_data.get("heading") + inv_yaw )/2.0
                ]
            )
            quaternions = np.array(
                [
                    float(shared_data.get("quaternion_q0", 0)),
                    float(shared_data.get("quaternion_q1", 0)),
                    float(shared_data.get("quaternion_q2", 0)),
                    float(shared_data.get("quaternion_q3", 0)),
                ]
            )
            # print(f"Raw Quat: {quaternions}")
            if self.prev_quaternion is None:
                if quaternions is not None:
                    self.prev_quaternion = quaternions
                else:
                    self.prev_quaternion = np.array([1.0, 0.0, 0.0, 0.0])


            if state == "MOVING":
                data_init["latitude"] = shared_data["latitude"]
                data_init["longitude"] = shared_data["longitude"]
                data_init["filtered_latitude"] = shared_data["filtered_latitude"]
                data_init["filtered_longitude"] = shared_data["filtered_longitude"]
                data_init["speed"] = shared_data["speed"]
                data_init["heading"] = shared_data["heading"]
                data_init["acceleration_x"] = shared_data["acceleration_x"]
                data_init["acceleration_y"] = shared_data["acceleration_y"]
                data_init["acceleration_z"] = shared_data["acceleration_z"]
                data_init["angular_velocity_x"] = shared_data["angular_velocity_x"]
                data_init["angular_velocity_y"] = shared_data["angular_velocity_y"]
                data_init["angular_velocity_z"] = shared_data["angular_velocity_z"]
                data_init["magnetic_field_x_uT"] = shared_data["magnetic_field_x_uT"]
                data_init["magnetic_field_y_uT"] = shared_data["magnetic_field_y_uT"]
                data_init["magnetic_field_z_uT"] = shared_data["magnetic_field_z_uT"]
                data_init["roll"] = shared_data["roll"]
                data_init["pitch"] = shared_data["pitch"]
                data_init["yaw"] = shared_data["yaw"]
                data_init["quaternion_q0"] = shared_data["quaternion_q0"]
                data_init["quaternion_q1"] = shared_data["quaternion_q1"]
                data_init["quaternion_q2"] = shared_data["quaternion_q2"]
                data_init["quaternion_q3"] = shared_data["quaternion_q3"]
                data_init["temperature"] = shared_data["temperature"]
                data_init["timestamp"] = shared_data["timestamp"]
                data_init["counter"] = data_init["counter"] + 1

            # if data_init["counter"] > 0:
                # quaternions = self.madgwick.updateMARG(
                #     q=self.prev_quaternion, mag=mags / 1000.0, acc=(accelerations * 9.81), gyr=gyros
                # )
                # print(f"Madgwick Quat: {quaternions}")
            # if self.use_angles:
            #     rot = rotational_transform(euler_angles=angles, euler_order=self.rotation_order)
            # else:
            # NOTE: Quaternion needs to be in scalar-first order (e.g. q0 as first value in array)
            rot = rotational_transform(quaternions=quaternions)
            # print(f"Madgwick Euler: {rot.as_euler('XYZ', degrees=True)}")
            accelerations_world = rot.apply(accelerations) * 9.81
            ax, ay, az = (
                accelerations_world[0],
                accelerations_world[1],
                (accelerations_world[2] - 9.81),
            )
        except Exception as e:
            ax, ay, az = 0.0, 0.0, 0.0
            print(traceback.format_exc())

        u_corrected = np.array([ax - kf.x[4], ay - kf.x[5]])
        kf.predict(u=u_corrected)

        # Update Phase
        z = np.array([np.nan, np.nan, np.nan, np.nan])  # Measurement vector: [x, y, vx, vy]
        try:
            if "latitude" in shared_data and "longitude" in shared_data:
                lat = shared_data.get("latitude", "nan")
                lon = shared_data.get("longitude", "nan")
                if lat is not None and lon is not None:
                    lat = float(lat)
                    lon = float(lon)
                    if self.lat0 is None or self.lon0 is None:
                        self.lat0, self.lon0 = lat, lon
                    x, y = latlon_to_xy(lat, lon, self.lat0, self.lon0)
                    data_init["x"], data_init["y"] = x, y

                    z[0], z[1] = x, y
        except Exception as e:
            pass
            # print(f"Position unavailable: {e}")

        try:
            if "speed" in shared_data and "heading" in shared_data:
                speed = shared_data.get("speed", "nan")
                heading = shared_data.get("heading", "nan")
                if speed is not None and heading is not None:
                    speed = float(speed)
                    heading = float(heading)
                    vx, vy = polar_to_cartesian(speed, heading)
                    z[2], z[3] = vx, vy
        except Exception as e:
            pass
            # print(f"Velocity unavailable: {e}")

        if detect_zupt(
            shared_data, acc_threshold=self.acc_threshold, gyro_threshold=self.gyro_threshold
        ):
            # print("ZUPT triggered: Stationary detected.")
            z[2], z[3] = 0.0, 0.0
            kf.R[2, 2] = 1e-5
            kf.R[3, 3] = 1e-5
            kf.x[2:4] = 0.0

        valid_indices = np.where(~np.isnan(z))[0]

        if len(valid_indices) > 0:
            kf.dim_z = len(valid_indices)
            z_valid = z[valid_indices].reshape(-1, 1)
            H_valid = kf.H[valid_indices, :]
            R_valid = kf.R[np.ix_(valid_indices, valid_indices)]
            # print(f"[DEBUG] valid indices: {valid_indices}")
            # print(f"[DEBUG] z_valid shape: {z_valid.shape}")
            # print(f"[DEBUG] H_valid shape: {H_valid.shape}")
            # print(f"[DEBUG] R_valid shape: {R_valid.shape}")
            try:
                kf.update(z=z_valid, H=H_valid, R=R_valid)
            except Exception as e:
                pass
                # print(f"Kalman update failed: {e}")

        # Convert filtered position back to lat/lon
        pos = kf.x[:2] / self.manual_correction
        vel = kf.x[2:4]

        if self.lat0 is not None and self.lon0 is not None:
            lat_est, lon_est = xy_to_latlon(pos[0], pos[1], self.lat0, self.lon0)
            self.lat0, self.lon0 = lat_est, lon_est
        else:
            lat_est, lon_est = None, None

        if state == "STATIC" :
            lat_est, lon_est = data_init.get("filtered_latitude"), data_init.get("filtered_longitude")

        shared_data["timestamp"] = datetime.now().strftime("%d-%m-%Y %H:%M:%S")
        shared_data.update(
            {
                "latitude" : lat_est,
                "longitude" : lon_est,
                "filtered_latitude": lat_est,
                "filtered_longitude": lon_est,
                "filtered_position_x": round(float(pos[0]), 4),
                "filtered_position_y": round(float(pos[1]), 4),
                "filtered_velocity_x": round(float(vel[0]), 4),
                "filtered_velocity_y": round(float(vel[1]), 4),
                "bias_accl_x": round(float(kf.x[4]), 4),
                "bias_accl_y": round(float(kf.x[5]), 4),
            }
        )

        return shared_data
