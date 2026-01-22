import serial.tools.list_ports

for p in serial.tools.list_ports.comports():
    print(p.device, p.vid, p.pid, p.serial_number, p.manufacturer)
