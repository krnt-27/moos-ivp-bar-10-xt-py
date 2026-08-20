import pandas as pd
from scipy.spatial.transform import Rotation
import numpy as np
import math

EARTH_RADIUS = 6378137


@staticmethod
def normalize_quaternion(quat):
    quat = np.array(quat)
    norm = np.linalg.norm(quat)
    if norm < 1e-6:
        raise ValueError("Quaternion norm too small, possible corrupt data")
    return quat / norm


def latlon_to_xy(lat, lon, lat0, lon0):
    R = EARTH_RADIUS
    dlat = math.radians(lat - lat0)
    dlon = math.radians(lon - lon0)
    x = R * dlon * math.cos(math.radians(lat0))
    y = R * dlat
    return x, y


def polar_to_cartesian(speed, heading_deg):
    rad = math.radians(heading_deg)
    vx = speed * math.cos(rad)
    vy = speed * math.sin(rad)
    return vx, vy


def xy_to_latlon(x, y, lat0, lon0):
    R = EARTH_RADIUS
    dlat = y / R
    dlon = x / (R * math.cos(math.radians(lat0)))
    lat = round(lat0 + math.degrees(dlat), 7)
    lon = round(lon0 + math.degrees(dlon), 7)
    return lat, lon


def rotate_to_world(ax, ay, yaw_deg):
    yaw_rad = math.radians(yaw_deg)
    cos_yaw = math.cos(yaw_rad)
    sin_yaw = math.sin(yaw_rad)

    ax_world = ax * cos_yaw - ay * sin_yaw
    ay_world = ax * sin_yaw + ay * cos_yaw
    return ax_world, ay_world


def quaternion_to_yaw(q):
    """
    Converts quaternion [q0, q1, q2, q3] to yaw (heading in degrees).
    Assumes scalar-first order: [q0, q1, q2, q3].
    """
    q0, q1, q2, q3 = q
    siny_cosp = 2.0 * (q0 * q3 + q1 * q2)
    cosy_cosp = 1.0 - 2.0 * (q2 * q2 + q3 * q3)
    yaw_rad = np.arctan2(siny_cosp, cosy_cosp)
    yaw_deg = np.degrees(yaw_rad) % 360
    return yaw_deg


def rotational_transform(
    euler_angles: np.array = None,
    euler_order: str = "xyz", # NOTE: Standard global frame extrinsic transform
    quaternions: np.array = None,
    degrees=True,
):
    """
    This function requires quaternion in scalar-first order
    """
    if euler_angles is None and quaternions is None:
        raise ValueError("Must provide either euler_angles or quaternions")
    if euler_angles is not None:
        r = Rotation.from_euler(euler_order, euler_angles, degrees=degrees)
    if quaternions is not None:
        w, x, y, z = quaternions
        r = Rotation.from_quat([x, y, z, w])
    return r

def mounting_alignment(heading_correction = -90.0):
    return Rotation.from_euler("z", heading_correction, degrees=True) 

def normalize_heading(yaw):
    """Convert yaw from [-180, 180] to [0, 360)"""
    return (yaw + 360.0) % 360.0
