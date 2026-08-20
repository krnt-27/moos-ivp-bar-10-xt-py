import pymoos
import json
import random


MOOS_SERVER_IP = "localhost"
MOOS_SERVER_PORT = 9000

MOOS_VARIABLES_VEHICLE = {
    "NAV_LAT",
    "NAV_LONG",
    "NAV_HEADING",
}

data_store_vehicle = {moos_variable: {} for moos_variable in MOOS_VARIABLES_VEHICLE}


class MOOSClient:
  def __init__(self, server_ip, server_port):
    self.comms = pymoos.comms()
    self.server_ip = server_ip
    self.server_port = server_port
    self.comms.set_on_connect_callback(self.on_connect)
    self.comms.set_on_mail_callback(self.on_new_mail)
    self.uniq_name = "INS_SERIAL_" + str(random.randint(1000,2000))
    self.comms.run(server_ip, server_port, self.uniq_name)

  def on_connect(self):
    print("[INFO] Connected to MOOSDB...")

    for moos_variable in MOOS_VARIABLES_VEHICLE:
      self.comms.register(moos_variable, 0)

    return True

  def on_new_mail(self):
    messages = self.comms.fetch()
    for msg in messages:
      moos_variable = msg.key()
      if msg.is_string():
          moos_data = msg.string()
      elif msg.is_double():
          moos_data = msg.double()
      elif msg.is_binary():
          moos_data = msg.binary_data()
      else:
          moos_data = msg.string()

      data_store_vehicle[moos_variable] = moos_data

      if moos_variable in MOOS_VARIABLES_VEHICLE :
        data_store_vehicle[moos_variable] = moos_data

    return True

  def publish(self, moos_variable, value):
    if type(value) == str :
      success = self.comms.notify(moos_variable, value)
    else:
      success = self.comms.notify(moos_variable, json.dumps(value))

    # if success:
    #     print(f"[INFO] Successfully published {moos_variable}: {value}")
    # else:
    #     print(f"[ERROR] Failed to publish {moos_variable}")

moos_client = MOOSClient(MOOS_SERVER_IP, MOOS_SERVER_PORT)
