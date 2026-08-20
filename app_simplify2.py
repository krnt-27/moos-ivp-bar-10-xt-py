# import socket
import time
import json
from datetime import datetime
import pytz
import paho.mqtt.client as mqtt
from src.data.serial_reading_simple2 import HWT9053Reader
from src.data.inertial_navigation import InertialNavigation
from src.utils.moos_client import moos_client
from src.filter.kalman_simple import SensorKalman
import statistics

# Konfigurasi
# port_number = "/dev/ttyINS2"
port_number = "/dev/ttyUSB2"
tcp_host = "10.5.50.3"
tcp_port = 5101
mqtt_broker = "103.204.15.126"
mqtt_port = 1883
mqtt_topic = "AUV-NAVIGATION/INS"
reconnect_interval = 5
use_mqtt = False

if use_mqtt:
    mqtt_client = mqtt.Client()
last_time = time.time()

nav = InertialNavigation()
sensor_filter = SensorKalman()

def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("Connected to MQTT Broker")
    else:
        print(f"Failed to connect to MQTT Broker: {rc}")

if use_mqtt:
    mqtt_client.on_connect = on_connect
    mqtt_client.connect(mqtt_broker, mqtt_port, 60)
    mqtt_client.loop_start()

# tcp_client = None
# def connect_tcp():
#     global tcp_client
#     try:
#         if tcp_client:
#             tcp_client.close()
#         tcp_client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
#         tcp_client.connect((tcp_host, tcp_port))
#         print(f"Connected to TCP server at {tcp_host}:{tcp_port}")
#     except socket.error as e:
#         print(f"TCP connection failed: {e}")
#         tcp_client = None

# def send_to_tcp(message):
#     global tcp_client
#     if not tcp_client:
#         connect_tcp()
#         if not tcp_client:
#             time.sleep(reconnect_interval)
#             return
#     try:
#         tcp_client.sendall(json.dumps(message).encode('utf-8') + b"\n")
#     except socket.error as e:
#         print(f"TCP send error: {e}")
#         connect_tcp()
#         time.sleep(reconnect_interval)

last_raw = 0.0
list_raw = [0.0, 0.0 , 0.0]

def read_serial():
    global last_time, last_raw, list_raw
    reader = HWT9053Reader(port=port_number, baudrate=9600, slave_id=0x50, heading_correction=20.0)
    if reader.connect():
        print("Sensor connected.")
        try:
            while True:
                data = reader.read_all()
                # time_diff = time.time() - last_time

                sensor_subset = {
                    "accelerationX": data.get("accelerationX", 0.0),
                    "accelerationY": data.get("accelerationY", 0.0),
                    "accelerationZ": data.get("accelerationZ", 0.0),
                    "angular_velocity_x": data.get("angular_velocity_x", 0.0),
                    "angular_velocity_y": data.get("angular_velocity_y", 0.0),
                    "angular_velocity_z": data.get("angular_velocity_z", 0.0),
                    "magnetic_field_x_uT": data.get("magnetic_field_x_uT", 0.0),
                    "magnetic_field_y_uT": data.get("magnetic_field_y_uT", 0.0),
                    "magnetic_field_z_uT": data.get("magnetic_field_z_uT", 0.0),
                    "roll_madgwick": data.get("roll_madgwick", 0.0),
                    "pitch_madgwick": data.get("pitch_madgwick", 0.0),
                    "yaw_madgwick": data.get("yaw_madgwick", 0.0),
                    "temperature": data.get("temperature", 0.0),
                }

                kalman_input = {
                    "accelerationX": sensor_subset["accelerationX"],
                    "accelerationY": sensor_subset["accelerationY"],
                    "accelerationZ": sensor_subset["accelerationZ"],
                    "angular_velocity_x": sensor_subset["angular_velocity_x"],
                    "angular_velocity_y": sensor_subset["angular_velocity_y"],
                    "angular_velocity_z": sensor_subset["angular_velocity_z"],
                    "magnetic_field_x_uT": sensor_subset["magnetic_field_x_uT"],
                    "magnetic_field_y_uT": sensor_subset["magnetic_field_y_uT"],
                    "magnetic_field_z_uT": sensor_subset["magnetic_field_z_uT"],
                    "roll_madgwick": sensor_subset["roll_madgwick"],
                    "pitch_madgwick": sensor_subset["pitch_madgwick"],
                    "yaw_madgwick": sensor_subset["yaw_madgwick"],
                    "temperature": sensor_subset["temperature"],
                }

                filtered_sensor = sensor_filter.update(kalman_input)

                del list_raw[0]
                list_raw.append(filtered_sensor["yaw_madgwick"])
                yaw_mean = statistics.mean(list_raw)

                filtered_data = {
                    **data,  # semua data 
                    "accelerationX": filtered_sensor["accelerationX"],
                    "accelerationY": filtered_sensor["accelerationY"],
                    "accelerationZ": filtered_sensor["accelerationZ"],
                    "angular_velocity_x": filtered_sensor["angular_velocity_x"],
                    "angular_velocity_y": filtered_sensor["angular_velocity_y"],
                    "angular_velocity_z": filtered_sensor["angular_velocity_z"],
                    "magnetic_field_x_uT": filtered_sensor["magnetic_field_x_uT"],
                    "magnetic_field_y_uT": filtered_sensor["magnetic_field_y_uT"],
                    "magnetic_field_z_uT": filtered_sensor["magnetic_field_z_uT"],
                    "roll_madgwick": filtered_sensor["roll_madgwick"],
                    "pitch_madgwick": filtered_sensor["pitch_madgwick"],
                    "yaw_madgwick": filtered_sensor["yaw_madgwick"],
                    "yaw_madgwick_raw": data["yaw_madgwick"],
                    "yaw_madgwick_mean": round(yaw_mean, 4),
                    "temperature" : filtered_sensor["temperature"],
                }

                latitude, longitude, depth = nav.update_position(filtered_data)
                filtered_data["latitude"] = latitude
                filtered_data["longitude"] = longitude
                filtered_data["depth"] = depth
                filtered_data["timestamp"] = datetime.now(pytz.timezone('Asia/Jakarta')).strftime("%d-%m-%Y %H:%M:%S")

                # message_str = json.dumps(data)
                # message = {"source": "INS_DATA", "NMEA": message_str}
                
                # print(f"jeda waktu: {time_diff}")
                last_time = time.time()
                last_raw = filtered_data["yaw_madgwick_raw"]

                filtered_data.pop("raw_yaw")

                # send_to_tcp(message)
                if use_mqtt:
                    mqtt_client.publish(mqtt_topic, json.dumps(filtered_data))
                moos_client.publish("INS_DATA2", json.dumps(filtered_data))

                # time.sleep(update_freq)
        except KeyboardInterrupt:
            print("Terminated by user.")
        finally:
            reader.disconnect()
            if use_mqtt:
                mqtt_client.loop_stop()
    else:
        print("Failed to connect to sensor.")

if __name__ == "__main__":
    read_serial()
