import numpy as np 
import math

def detect_zupt(data, acc_threshold=0.005, gyro_threshold=0.5):
    """
    Determines if the system is stationary (ZUPT condition).
    Returns True if both acceleration and gyro magnitude are below thresholds.
    """
    try:
        acc = np.array([
            float(data.get("acceleration_x", 0)),
            float(data.get("acceleration_y", 0)),
            float(data.get("acceleration_z", 0)),
        ])
        gyro = np.array([
            float(data.get("angular_velocity_x", 0)),
            float(data.get("angular_velocity_y", 0)),
            float(data.get("angular_velocity_z", 0)),
        ])
    except Exception:
        return False

    acc_norm = np.linalg.norm(acc) - 1
    gyro_norm = np.linalg.norm(gyro)
    
    return acc_norm < acc_threshold and gyro_norm < gyro_threshold


def detect_xy_movement(acceleration, velocity, acc_threshold=0.25, vel_threshold=0.8):
    acc_union = round(math.sqrt(acceleration[0]**2 + acceleration[1]**2), 3)
    vel_union = round(math.sqrt(velocity[0]**2 + velocity[1]**2 + velocity[2]**2), 3)
    if acc_union < acc_threshold or vel_union < vel_threshold :
        return "STATIC", acc_union, vel_union
    else:
        return "MOVING", acc_union, vel_union


