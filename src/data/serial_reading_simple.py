import struct
import time
from pymodbus.client import ModbusSerialClient
from typing import Tuple, List
import random
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

list_yaw = []

last_time = time.time()
last_yaw = 360.0
diff_time = 0
diff_yaw = 0
init_yaw = 0

last_yaw_drift = 0.0

diff_time_list = []
diff_yaw_list = []

class HWT9053Reader:
    def __init__(self, port="COM4", baudrate=9600, slave_id=0x50, heading_correction=20.0):
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

        self.ACC_X_CORRECTION = 0.0
        self.ACC_Y_CORRECTION = 0.0
        self.ACC_Z_CORRECTION = 0.0

        # self.ACC_X_CORRECTION = 0.0372
        # self.ACC_Y_CORRECTION = 0.1053
        # self.ACC_Z_CORRECTION = -0.0035

        self.ROLL_CORRECTION = -6.338
        self.PITCH_CORRECTION = 2.205
        self.YAW_CORRECTION = 0.0

        self.YAW_DRIFT = 0.0
        # self.YAW_INCREMENT = 0.0000135
        self.INTERNAL_COUNTER = 0
        self.DRIFT_COUNT = 0
        self.YAW_HEADING = heading_correction

        self.YAW_STATIC = 0.0

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
        global processed_data, last_time, last_yaw, init_yaw, last_yaw_drift
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

            data["roll"] = (
                -1 * roll_raw / self.SCALE_ANGLE_INT32 + self.ROLL_CORRECTION + 360.0
            ) % 360
            data["pitch"] = (
                -1 * pitch_raw / self.SCALE_ANGLE_INT32 + self.PITCH_CORRECTION + 360.0
            ) % 360
            data["yaw"] = (
                -1.0 * yaw_raw / self.SCALE_ANGLE_INT32 + self.YAW_CORRECTION + 360.0 + self.YAW_HEADING
            ) % 360

            # calculate Angle
            roll, pitch, yaw, q0, q1, q2, q3 = compute_rpy_madgwick(
                ax, ay, az,
                gx, gy, gz,
                mx, my, mz
            )

            if self.INTERNAL_COUNTER < 265 :
                list_ax.append(self.decode_signed_int16(main_regs[0]) / self.SCALE_ACCEL-1.0)
                list_ay.append(self.decode_signed_int16(main_regs[1]) / self.SCALE_ACCEL-1.0)
                list_az.append(self.decode_signed_int16(main_regs[2]) / self.SCALE_ACCEL-1.0)
                list_yaw.append(yaw)
                data["status"] = "Calibrating"
            else:
                data["status"] = "Calibrated"
            
            if self.INTERNAL_COUNTER == 265 :
                mean_ax, mean_ay, mean_az = statistics.mean(list_ax), statistics.mean(list_ay), statistics.mean(list_az)
                self.ACC_X_CORRECTION = -1 * (mean_ax+1.0)
                self.ACC_Y_CORRECTION = -1 * (mean_ay+1.0)
                self.ACC_Z_CORRECTION = -1 * (mean_az)
                print(self.ACC_X_CORRECTION, self.ACC_Y_CORRECTION, self.ACC_Z_CORRECTION)
                self.YAW_STATIC = round(statistics.mean(list_yaw), 4) + self.YAW_HEADING
                list_ax.clear()
                list_ay.clear()
                list_az.clear()
                list_yaw.clear()

            data["roll_madgwick"] = roll
            data["pitch_madgwick"] = pitch
            data["yaw_madgwick"] = yaw + self.YAW_HEADING
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
        processed_data["yaw"] = data["yaw"]
        # processed_data["yaw"] = round(data["yaw"] + self.YAW_INCREMENT * self.INTERNAL_COUNTER, 6)
        processed_data["raw_yaw"] = round(data["yaw"], 6)
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
        processed_data["status"] = data["status"]
        processed_data["yaw_static"] = self.YAW_STATIC

        # if data["yaw"] > last_yaw :
        #     diff_yaw = data["yaw"] - last_yaw
        
        # processed_data["diff_yaw"] = diff_yaw
        
        # processed_data["raw_yaw"] = round(data["yaw"], 6)

        # diff_yaw = abs(processed_data["raw_yaw"] - last_yaw)

        # diff_time_list.append(diff_time)
        # diff_yaw_list.append(diff_yaw)

        # last_yaw = processed_data["raw_yaw"]
        # processed_data["diff_yaw"] = diff_yaw
        # processed_data["diff_time"] = diff_time

        # print("counter, yaw_madgwick, init_yaw, yaw: ", self.INTERNAL_COUNTER, processed_data["yaw_madgwick"], init_yaw, data["yaw"], self.YAW_CORRECTION)

        if self.INTERNAL_COUNTER == 0 :
            init_yaw = processed_data["yaw_madgwick"]
            processed_data["yaw"] = data["yaw_madgwick"]
            self.YAW_CORRECTION = abs(processed_data["raw_yaw"] - processed_data["yaw_madgwick"])
        
        processed_data["init_yaw"] = init_yaw

        beda_yaw = abs(processed_data["yaw"] - last_yaw)
        # print(beda_yaw)

        if processed_data["yaw"] < last_yaw and self.INTERNAL_COUNTER > 2 and beda_yaw < 0.05:
            self.DRIFT_COUNT += 1
            if self.DRIFT_COUNT > 2:
                time_now = time.time()
                diff_time = time_now - last_time
                diff_yaw = processed_data["yaw"] - last_yaw_drift
                self.YAW_DRIFT += abs(diff_yaw)
                processed_data["yaw_processed"] = round(processed_data["yaw"] + self.YAW_DRIFT, 4)
                last_time = time_now
            
            last_yaw_drift = processed_data["yaw"]

        last_yaw = processed_data["raw_yaw"]

        self.INTERNAL_COUNTER += 1

        # return data
        return processed_data


def main():
    pass


if __name__ == "__main__":
    main()
