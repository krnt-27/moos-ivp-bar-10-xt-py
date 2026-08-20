from pymodbus.client import ModbusSerialClient


client = ModbusSerialClient(
    port='COM4', 
    baudrate=4800,
    stopbits=1,
    bytesize=8,
    parity='N',
    timeout=1
)

client.connect()

