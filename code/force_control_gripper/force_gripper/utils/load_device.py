import os
import yaml
import serial.tools.list_ports


def find_port_by_name(name: str) -> str:
    """
    Automatically locate force_gripper/config/devices.yaml,
    read device info, and return the system serial port (e.g., 'COM3' or '/dev/ttyUSB0')
    for the given device name ('right', 'left', 'gripper', 'teleoperator').
    """
    # Locate config/devices.yaml relative to this file
    current_dir = os.path.dirname(os.path.abspath(__file__))
    config_path = os.path.join(current_dir, "..", "config", "devices.yaml")
    config_path = os.path.normpath(config_path)

    if not os.path.exists(config_path):
        raise FileNotFoundError(f"Config file not found: {config_path}")

    # Load YAML
    with open(config_path, "r") as f:
        cfg = yaml.safe_load(f)

    devices_cfg = cfg.get("devices", {})
    if name not in devices_cfg:
        raise ValueError(f"Device '{name}' not found in config file: {config_path}")

    target = devices_cfg[name]

    # Search available serial ports
    for p in serial.tools.list_ports.comports():
        if p.serial_number == target["serial"] or target["serial"] is None:
            if (p.vid == target.get("vid") and p.pid == target.get("pid")) or (
                target.get("vid") is None or target.get("pid") is None
            ):
                return p.device  # e.g. 'COM3' or '/dev/ttyUSB0'
        

    raise RuntimeError(f"Device '{name}' (serial={target['serial']}) not found.")


if __name__ == "__main__":
    try:
        for dev in ["right", "left", "gripper", "teleoperator"]:
            port = find_port_by_name(dev)
            print(f"{dev}: {port}")
    except Exception as e:
        print("Error:", e)
