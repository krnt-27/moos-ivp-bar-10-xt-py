import json
import time


class MockReader:
    def __init__(self, path, frequency_hz=98.0, loop=True):
        self.excludes = {
            "timestamp",
            # "latitude",
            # "longitude",
            "speed",
            "heading",
            "filtered_latitude",
            "filtered_longitude",
            "filtered_position_x",
            "filtered_position_y",
            "filtered_velocity_x",
            "filtered_velocity_y",
            "bias_accl_x",
            "bias_accl_y",
        }
        self.file = open(path, "r")
        self.index = 0
        self.interval = 1.0 / frequency_hz  # seconds per sample
        self.loop = loop
        self.done = False

    def connect(self):
        return True

    def disconnect(self):
        return True

    def read_all(self):
        if self.done:
            return None

        line = self.file.readline()
        if not line:
            if self.loop:
                self.file.seek(0)
                line = self.file.readline()
            else:
                self.done = True
                return None

        data = json.loads(line)
        return {k: v for k, v in data.items() if k not in self.excludes}
