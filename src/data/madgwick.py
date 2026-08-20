import numpy as np
from ahrs.filters import Madgwick
from scipy.spatial.transform import Rotation as R
import json

file = open("data_store.csv", "r")
if file :
    last_data = json.loads(file.read())
    prev_quaternion = np.array([last_data.get("quaternion_q0"), last_data.get("quaternion_q1"), last_data.get("quaternion_q2"), last_data.get("quaternion_q3")])  # [w, x, y, z]
else:
    prev_quaternion = np.array([0.0, 0.0, 0.0, 0.0])  # [w, x, y, z]

madgwick_filter = Madgwick()

def compute_rpy_madgwick(
    acc_x, acc_y, acc_z,              # g
    gyro_x, gyro_y, gyro_z,           # degree/s
    mag_x, mag_y, mag_z               # µT
):
    global prev_quaternion

    gyro_rad = np.radians([gyro_x, gyro_y, gyro_z]) 

    # array input
    acc = np.array([acc_x, acc_y, acc_z])
    gyr = np.radians(np.array([gyro_x, gyro_y, gyro_z])) + 1e-4  
    mag = np.array([mag_x, mag_y, mag_z])

    quat = madgwick_filter.updateMARG(prev_quaternion, gyr, acc, mag)
    prev_quaternion = quat

    rotation = R.from_quat([quat[1], quat[2], quat[3], quat[0]])  # [x, y, z, w]
    roll, pitch, yaw = rotation.as_euler('xyz', degrees=True)

    return (
        round(roll % 360, 3),
        round(pitch % 360, 3),
        round(yaw % 360, 3),
        round(quat[0], 5),
        round(quat[1], 5),
        round(quat[2], 5),
        round(quat[3], 5),
    )
