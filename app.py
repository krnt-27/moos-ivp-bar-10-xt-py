from src.data.serial_reading import HWT9053Reader
from src.data.mock_reader import MockReader
from src.utils.locations import load_locations
import time
from datetime import datetime
import json
import argparse
from src.filter.realtime_kalman import EKFRealtime
import paho.mqtt.client as mqtt
from src.utils.process_ins import generate_map

# from src.utils.moos_client import moos_client
import socket



tcp_host = "10.5.50.3"
tcp_port = 5101
reconnect_interval = 5

tcp_client = None

def connect_tcp():
    global tcp_client
    try:
        if tcp_client:
            tcp_client.close()
        tcp_client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        tcp_client.connect((tcp_host, tcp_port))
        print(f"Connected to TCP server at {tcp_host}:{tcp_port}")
    except socket.error as e:
        print(f"Failed to connect to TCP server: {e}")
        tcp_client = None

def send_to_tcp(message):
    global tcp_client
    if not tcp_client:
        connect_tcp()
        if not tcp_client:
            time.sleep(reconnect_interval)
            return

    try:
        tcp_client.sendall(json.dumps(message).encode('utf-8') + b"\n")
        # print(f"Sent to TCP server: {message}")
    except socket.error as e:
        print(f"Error sending to TCP server: {e}")
        connect_tcp()
        time.sleep(reconnect_interval)


port_number = "/dev/ttyINS"
mqtt_broker = "10.5.51.2"
mqtt_port = 1883
mqtt_topic = "AUV-NAVIGATION/INS"
mock_data = "mock_data.jsonl"
test_locs_path = "test_locations.yaml"

mqtt_client = mqtt.Client()
default_freq = 10.0

# NOTE: Init test location here
location_key = "galangan"
test_location = load_locations(test_locs_path, location_key)


def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("Connected to MQTT Broker successfully!")
    else:
        print(f"Failed to connect to MQTT Broker with code {rc}")


def on_publish(client, userdata, mid):
    pass


heading_correction = 90.0

def initialize_kalman_filter_and_sensor(sensor_freq=default_freq):
    kf = EKFRealtime(frequency=sensor_freq, use_angles=False)
    init_location = [test_location["lat"], test_location["lon"]]
    init_heading = test_location["heading"]
    init_heading = (-1 * init_heading + heading_correction) % 360
    init_speed = 1.0
    data_store = {
        "timestamp": datetime.now().strftime("%d-%m-%Y %H:%M:%S"),
        "latitude": init_location[0],
        "longitude": init_location[1],
        "speed": init_speed,
        "heading": init_heading,
        "init_heading": init_heading,
        "accelerationX": 0.0,
        "accelerationY": 0.0,
        "accelerationZ": 0.0,
        "angular_velocity_x": 0.0,
        "angular_velocity_y": 0.0,
        "angular_velocity_z": 0.0,
        "magnetic_field_x_uT": 0.0,
        "magnetic_field_y_uT": 0.0,
        "magnetic_field_z_uT": 0.0,
        "roll": 0.0,
        "pitch": 0.0,
        "yaw": 0.0,
        "quaternion_q0": 0.0,
        "quaternion_q1": 0.0,
        "quaternion_q2": 0.0,
        "quaternion_q3": 0.0,
        "temperature": 0.0,
    }

    reader = HWT9053Reader(port=port_number, baudrate=9600, slave_id=0x50)

    return kf, data_store, reader


def main(
    write_json=False,
    mock_reader=False,
    mock_data=None,
    sensor_freq=default_freq,
    update_freq=0.5,
    mqtt_publish=True,
):
    kf, data_store, reader = initialize_kalman_filter_and_sensor(sensor_freq=sensor_freq)
    if mock_reader and mock_data is not None:
        reader = MockReader(path=mock_data, loop=False)
    start = datetime.now().strftime("%d%m%Y%H%M")
    mqtt_publish = True
    if mqtt_publish:
        mqtt_client.on_connect = on_connect
        mqtt_client.on_publish = on_publish
        mqtt_client.connect(mqtt_broker, mqtt_port, 60)

        mqtt_client.loop_start()

    if reader.connect():
        print("Connected to sensor.")
        last_time = time.time()
        try:
            while True:
                data = reader.read_all()
                data_store.update(data)
                now = time.time()
                dt = (now - last_time) - update_freq
                last_time = now
                updated_data = kf.run_kalman_rt(data_store)

                message_str = json.dumps(updated_data)

                message_to_send = {"source": "INS_DATA", "NMEA": message_str}
                send_to_tcp(message_to_send)

                # moos_client.publish("INS_DATA", json.dumps(updated_data))

                # # print("Updated", updated_data, "\n")
                # if write_json:
                #     with open(
                #         f"./results/kalman_output_{start}_{heading_correction}.jsonl", "a"
                #     ) as f:
                #         json.dump(updated_data, f)
                #         f.write("\n")
                #     generate_map(
                #         f"kalman_output_{start}_{heading_correction}",
                #         f"results/map_{start}_{heading_correction}.html",
                #     )
                if mqtt_publish:
                    mqtt_client.publish(mqtt_topic, json.dumps(updated_data))
                

                data_store.update(
                    {
                        "latitude": None,
                        "longitude": None,
                        "speed": None,
                        "heading": None,
                    }
                )
                time.sleep(update_freq)
        except KeyboardInterrupt:
            print("Terminating the pipeline.")
        finally:
            # generate_map(f'kalman_output_{start}', f'results/map_{start}.html')
            reader.disconnect()
            if mqtt_publish:
                mqtt_client.loop_stop()
    else:
        print("Failed to connect to the sensor.")


def test(mode):
    kf, data_store, reader = initialize_kalman_filter_and_sensor()
    print(f"Running test mode: {mode.upper()}")

    if reader.connect():
        print("Connected to sensor.")
        try:
            while True:
                data = reader.read_all()
                data_store.update(data)
                # Simulate missing measurements
                if mode == "gps":
                    test_data = {"latitude": None, "longitude": None}
                elif mode == "velocity":
                    test_data = {"velocity": None, "heading": None}
                elif mode == "none":
                    test_data = {
                        "latitude": None,
                        "longitude": None,
                        "velocity": None,
                        "heading": None,
                    }
                # else "all": include all data
                data_store.update(test_data)
                updated_data = kf.run_kalman_rt(data_store)

                time.sleep(0.5)

        except KeyboardInterrupt:
            print("Test stopped.")


if __name__ == "__main__":
    write_json = True
    run_freq = default_freq
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--test",
        choices=["gps", "velocity", "none", "all"],
        help="Run test mode with specific missing data",
    )
    parser.add_argument(
        "--mock",
        help="Run with mock or collected data",
    )
    parser.add_argument("--freq", help="Set Kalman filter frequency")
    parser.add_argument("--publish", help="Publish MQTT", default=False)
    parser.add_argument("--no-write", help="Don't write to JSONL file", action="store_false")
    args = parser.parse_args()

    if args.no_write:
        write_json = args.no_write

    if args.freq:
        run_freq = float(args.freq)

    if args.test:
        test(args.test)

    if args.mock:
        main(
            write_json=write_json,
            mock_reader=True,
            mock_data=args.mock,
            sensor_freq=run_freq,
        )
    else:
        main(write_json=write_json, sensor_freq=run_freq, mqtt_publish=args.publish)
