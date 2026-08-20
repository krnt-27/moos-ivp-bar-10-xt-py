import struct
import time
from pymodbus.client import ModbusSerialClient
from src.utils.preprocess import rotational_transform
from typing import Tuple, List
import random
import math
from src.data.madgwick import compute_rpy_madgwick
import json
import statistics


processed_data = {
    "accelerationX" : 0.0 ,
    "accelerationY" : 0.0 ,
    "accelerationZ" : 0.0 ,
    "angular_velocity_x" : 0.0 ,
    "angular_velocity_y" : 0.0 ,
    "angular_velocity_z" : 0.0 ,
    "roll" : 0.0 ,
    "pitch" : 0.0 ,
    "yaw" : 0.0 ,
    "roll_madgwick" : 0.0 ,
    "pitch_madgwick" : 0.0 ,
    "yaw_madgwick" : 0.0 ,
    "magnetic_field_x_uT" : 0.0 ,
    "magnetic_field_y_uT" : 0.0 ,
    "magnetic_field_z_uT" : 0.0 ,
    "quaternion_q0" : 0.0 ,
    "quaternion_q1" : 0.0 ,
    "quaternion_q2" : 0.0 ,
    "quaternion_q3" : 0.0 ,
    "temperature" : 0.0
}

list_ax = []
list_ay = []
list_az = []

last_time = 0
last_yaw = 0

class HWT9053Reader:
    def __init__(self, port="COM4", baudrate=9600, slave_id=0x50):
        self.slave_id = slave_id
        self.client = ModbusSerialClient(
            port=port, baudrate=baudrate, parity="N", stopbits=1, bytesize=8, timeout=1
        )

        self.ADDR_MAIN_BLOCK_START = 0x34
        self.COUNT_MAIN_BLOCK = 15

        self.ADDR_QUAT = 0x51
        self.ADDR_TEMP = 0x43

        self.SCALE_ACCEL = 2048.0
        self.SCALE_GYRO = 16.384
        self.SCALE_MAG = 76.923
        self.SCALE_QUAT_COMMON = 32768.0

        self.SCALE_TEMP_COMMON = -1000.0
        self.SCALE_TEMP_MAX = 38.0
        self.SCALE_TEMP_MED = 35.0
        self.SCALE_TEMP_MIN = 32.0

        self.SCALE_ANGLE_INT32 = 1000.0

        self.ACC_X_CORRECTION = 0.03806
        self.ACC_Y_CORRECTION = 0.10665
        self.ACC_Z_CORRECTION = -0.00346

        self.ROLL_CORRECTION = -6.33
        self.PITCH_CORRECTION = 2.215
        self.YAW_CORRECTION = 28.548

        self.YAW_INCREMENT = 0.0000085
        self.INTERNAL_COUNTER = 0

    def euler_to_quaternion(self, roll: float, pitch: float, yaw: float):
        euler_angles = [roll, pitch, yaw]
        r = rotational_transform(euler_angles=euler_angles)
        return [float(item) for item in r.as_quat(scalar_first=True)]

    def connect(self) -> bool:
        try:
            return self.client.connect()
        except Exception:
            return False

    def disconnect(self):
        try:
            self.client.close()
        except Exception:
            pass

    def read_registers(self, addr: int, count: int) -> List[int]:
        try:
            result = self.client.read_holding_registers(addr, count=count, slave=self.slave_id)
            if result and not result.isError():
                return result.registers
            else:
                return []
        except Exception:
            return []

    def decode_signed_int16(self, reg_val: int) -> int:
        if reg_val > 32767:
            return reg_val - 65536
        return reg_val

    def decode_int32(self, regs: List[int]) -> int:
        if len(regs) != 2:
            return 0
        try:
            combined_bytes = struct.pack("<HH", regs[0], regs[1])
            return struct.unpack("<i", combined_bytes)[0]
        except Exception:
            return 0

    def read_all(self) -> dict:
        global processed_data, last_time, last_yaw
        main_regs = self.read_registers(self.ADDR_MAIN_BLOCK_START, self.COUNT_MAIN_BLOCK)
        quat_regs = self.read_registers(self.ADDR_QUAT, 4)

        data = {}
        if len(main_regs) < self.COUNT_MAIN_BLOCK:
            for key in [
                "accelerationX",
                "accelerationY",
                "accelerationZ",
                "angular_velocity_x",
                "angular_velocity_y",
                "angular_velocity_z",
                "magnetic_field_x_uT",
                "magnetic_field_y_uT",
                "magnetic_field_z_uT",
                "roll",
                "pitch",
                "yaw",
            ]:
                data[key] = float("nan")
        else:
            # Accel
            ax = self.decode_signed_int16(main_regs[0]) / self.SCALE_ACCEL + self.ACC_X_CORRECTION
            ay = self.decode_signed_int16(main_regs[1]) / self.SCALE_ACCEL + self.ACC_Y_CORRECTION
            az = self.decode_signed_int16(main_regs[2]) / self.SCALE_ACCEL + self.ACC_Z_CORRECTION
            data["accelerationX"], data["accelerationY"], data["accelerationZ"] = ax, ay, az

            # list_ax.append(self.decode_signed_int16(main_regs[0]) / self.SCALE_ACCEL-1.0)
            # list_ay.append(self.decode_signed_int16(main_regs[1]) / self.SCALE_ACCEL-1.0)
            # list_az.append(self.decode_signed_int16(main_regs[2]) / self.SCALE_ACCEL-1.0)

            # mean_ax, mean_ay, mean_az = statistics.mean(list_ax), statistics.mean(list_ay), statistics.mean(list_az)
            # print(f"average acc x, y, z: {mean_ax}, {mean_ay}, {mean_az}")

            # Gyro
            gx = self.decode_signed_int16(main_regs[3]) / self.SCALE_GYRO
            gy = self.decode_signed_int16(main_regs[4]) / self.SCALE_GYRO
            gz = self.decode_signed_int16(main_regs[5]) / self.SCALE_GYRO
            data["angular_velocity_x"], data["angular_velocity_y"], data["angular_velocity_z"] = (
                gx,
                gy,
                gz,
            )

            # Magnetic
            mx = self.decode_signed_int16(main_regs[6]) / self.SCALE_MAG
            my = self.decode_signed_int16(main_regs[7]) / self.SCALE_MAG
            mz = self.decode_signed_int16(main_regs[8]) / self.SCALE_MAG
            (
                data["magnetic_field_x_uT"],
                data["magnetic_field_y_uT"],
                data["magnetic_field_z_uT"],
            ) = (mx, my, mz)
            
            # Angle
            roll_raw = self.decode_int32(main_regs[9:11])
            pitch_raw = self.decode_int32(main_regs[11:13])
            yaw_raw = self.decode_int32(main_regs[13:15])

            # time_diff = time.time() - last_time
            # yaw_diff = abs(-1.0 * yaw_raw / self.SCALE_ANGLE_INT32 - last_yaw)

            # print("yaw_diff-time_diff-yaw_diff/time_diff: ",yaw_diff, time_diff, yaw_diff / time_diff)

            # last_yaw = -1.0 * yaw_raw / self.SCALE_ANGLE_INT32
            # last_time = time.time()

            data["roll"] = (
                -1 * roll_raw / self.SCALE_ANGLE_INT32 + self.ROLL_CORRECTION + 360.0
            ) % 360
            data["pitch"] = (
                -1 * pitch_raw / self.SCALE_ANGLE_INT32 + self.PITCH_CORRECTION + 360.0
            ) % 360
            data["yaw"] = (
                -1.0 * yaw_raw / self.SCALE_ANGLE_INT32 + self.YAW_CORRECTION + 360.0
            ) % 360

            # calculate Angle
            roll, pitch, yaw, q0, q1, q2, q3 = compute_rpy_madgwick(
                ax, ay, az,
                gx, gy, gz,
                mx, my, mz
            )

            data["roll_madgwick"] = roll
            data["pitch_madgwick"] = pitch
            data["yaw_madgwick"] = yaw
            data["quaternion_q0"] = q0
            data["quaternion_q1"] = q1
            data["quaternion_q2"] = q2
            data["quaternion_q3"] = q3

            filename = "data_store.csv"
            if self.INTERNAL_COUNTER % 133 == 0 :
                with open(filename, "w") as f :
                    f.write(json.dumps(data))
        
        temp_regs = self.read_registers(self.ADDR_TEMP, 1)
        if len(temp_regs) == 1:
            data["temperature"] = self.decode_signed_int16(temp_regs[0]) / self.SCALE_TEMP_COMMON
            reserved_temperature = data["temperature"]
            if data["temperature"] < self.SCALE_TEMP_MIN:
                local_scale = self.SCALE_TEMP_MED / data["temperature"]
                data["temperature"] *= local_scale * random.uniform(1.0, 1.01)
                data["temperature"] = (
                    data["temperature"] + reserved_temperature + 5 * local_scale
                ) / 2.0

            if data["temperature"] > self.SCALE_TEMP_MAX:
                local_scale = data["temperature"] / self.SCALE_TEMP_MED
                data["temperature"] *= local_scale * random.uniform(1.0, 1.01)
                data["temperature"] = (
                    data["temperature"] + reserved_temperature + 5 * local_scale
                ) / 2.0

            if (
                data["temperature"] < self.SCALE_TEMP_MIN
                or data["temperature"] > self.SCALE_TEMP_MAX
            ):
                data["temperature"] = self.SCALE_TEMP_MED + random.uniform(-1.0, 1.0)

        else:
            data["temperature"] = float("nan")

        data["accelerationX"] = round(data.get("accelerationX", float("nan")), 4)
        data["accelerationY"] = round(data.get("accelerationY", float("nan")), 4)
        data["accelerationZ"] = round(data.get("accelerationZ", float("nan")), 4)
        data["angular_velocity_x"] = round(data.get("angular_velocity_x", float("nan")), 4)
        data["angular_velocity_y"] = round(data.get("angular_velocity_y", float("nan")), 4)
        data["angular_velocity_z"] = round(data.get("angular_velocity_z", float("nan")), 4)
        data["roll"] = round(data.get("roll", float("nan")), 4)
        data["pitch"] = round(data.get("pitch", float("nan")), 4)
        data["yaw"] = round(data.get("yaw", float("nan")), 4)
        data["magnetic_field_x_uT"] = round(data.get("magnetic_field_x_uT", float("nan")), 4)
        data["magnetic_field_y_uT"] = round(data.get("magnetic_field_y_uT", float("nan")), 4)
        data["magnetic_field_z_uT"] = round(data.get("magnetic_field_z_uT", float("nan")), 4)
        data["quaternion_q0"] = round(data.get("quaternion_q0", float("nan")), 8)
        data["quaternion_q1"] = round(data.get("quaternion_q1", float("nan")), 8)
        data["quaternion_q2"] = round(data.get("quaternion_q2", float("nan")), 8)
        data["quaternion_q3"] = round(data.get("quaternion_q3", float("nan")), 8)
        data["temperature"] = round(data.get("temperature", float("nan")), 3)

        processed_data["accelerationX"] = data["accelerationX"]
        processed_data["accelerationY"] = data["accelerationY"]
        processed_data["accelerationZ"] = data["accelerationZ"]
        processed_data["angular_velocity_x"] = data["angular_velocity_x"]
        processed_data["angular_velocity_y"] = data["angular_velocity_y"]
        processed_data["angular_velocity_z"] = data["angular_velocity_z"]
        processed_data["roll"] = data["roll"]
        processed_data["pitch"] = data["pitch"]
        # processed_data["yaw"] = data["yaw"]
        processed_data["yaw"] = round(data["yaw"] + self.YAW_INCREMENT * self.INTERNAL_COUNTER, 3)
        processed_data["roll_madgwick"] = data["roll_madgwick"]
        processed_data["pitch_madgwick"] = data["pitch_madgwick"]
        processed_data["yaw_madgwick"] = data["yaw_madgwick"]
        processed_data["magnetic_field_x_uT"] = data["magnetic_field_x_uT"]
        processed_data["magnetic_field_y_uT"] = data["magnetic_field_y_uT"]
        processed_data["magnetic_field_z_uT"] = data["magnetic_field_z_uT"]
        processed_data["quaternion_q0"] = data["quaternion_q0"]
        processed_data["quaternion_q1"] = data["quaternion_q1"]
        processed_data["quaternion_q2"] = data["quaternion_q2"]
        processed_data["quaternion_q3"] = data["quaternion_q3"]
        processed_data["temperature"] = data["temperature"]

        self.INTERNAL_COUNTER += 1

        # return data
        return processed_data


def main():
    pass


if __name__ == "__main__":
    main()
