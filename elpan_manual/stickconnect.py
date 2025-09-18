import os
import time
import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from evdev import InputDevice, ecodes

DEVICE_PATH = "/dev/input/js_procontroller"

# Mapping kode tombol
button_map = {
    ecodes.BTN_SOUTH: "X",
    ecodes.BTN_EAST: "O",
    ecodes.BTN_NORTH: "△",
    ecodes.BTN_WEST: "□",
    ecodes.BTN_TL: "L1",
    ecodes.BTN_TR: "R1",
    ecodes.BTN_THUMBL: "L3",
    ecodes.BTN_THUMBR: "R3",
    ecodes.BTN_TL2: "L2",
    ecodes.BTN_TR2: "R2",
}

# Mapping kode axis
axis_map = {
    ecodes.ABS_X: "LX",
    ecodes.ABS_Y: "LY",
    ecodes.ABS_RX: "RX",
    ecodes.ABS_RY: "RY",
    ecodes.ABS_HAT0X: "↔ Arrow",
    ecodes.ABS_HAT0Y: "↕ Arrow",
}

class StickPublisher(Node):
    def __init__(self):
        super().__init__("stick_publisher")
        self.publisher_ = self.create_publisher(String, "/stickpub", 10)
        self.buttons = {v: 0 for v in button_map.values()}
        self.axes = {v: 0 for v in axis_map.values()}
        self.get_logger().info("🎮 Stick Publisher JSON Started")
        self.main_loop()

    def reset_all(self):
        for k in self.axes:
            self.axes[k] = 0
        for k in self.buttons:
            self.buttons[k] = 0

    def normalize_input(self, name, value):
        if "Arrow" in name:
            return -1 if value == -1 else 1 if value == 1 else 0
        elif name in ("LY", "RY"):
            return -value
        return value

    def format_status_json(self):
        tombol = {
            "Silang": self.buttons.get("X", 0),
            "Bulat": self.buttons.get("O", 0),
            "Kotak": self.buttons.get("□", 0),
            "Segitiga": self.buttons.get("△", 0),
            "L1": self.buttons.get("L1", 0),
            "R1": self.buttons.get("R1", 0),
            "L2": self.buttons.get("L2", 0),
            "R2": self.buttons.get("R2", 0),
            "L3": self.buttons.get("L3", 0),
            "R3": self.buttons.get("R3", 0),
            "Kanan": 1 if self.axes.get("↔ Arrow", 0) == 1 else 0,
            "Kiri": 1 if self.axes.get("↔ Arrow", 0) == -1 else 0,
            "Atas": 1 if self.axes.get("↕ Arrow", 0) == -1 else 0,
            "Bawah": 1 if self.axes.get("↕ Arrow", 0) == 1 else 0
        }

        analog = {
            "LX": self.axes.get("LX", 0),
            "LY": self.axes.get("LY", 0),
            "RX": self.axes.get("RX", 0),
            "RY": self.axes.get("RY", 0)
        }

        return json.dumps({
            "tombol": tombol,
            "analog": analog,
        })

    def process_event(self, event):
        if event.type == ecodes.EV_KEY:
            if event.code in button_map:
                name = button_map[event.code]
                self.buttons[name] = 1 if event.value else 0
        elif event.type == ecodes.EV_ABS:
            if event.code in axis_map:
                name = axis_map[event.code]
                self.axes[name] = self.normalize_input(name, event.value)

    def main_loop(self):
        while rclpy.ok():
            try:
                dev = InputDevice(DEVICE_PATH)
                self.get_logger().info(f"✅ Terhubung: {dev.name}")

                for event in dev.read_loop():
                    self.process_event(event)

                    msg = String()
                    msg.data = self.format_status_json()
                    self.publisher_.publish(msg)
                    # print(msg.data)
                    
            except (OSError, FileNotFoundError):
                self.get_logger().warn("⚠️ Gamepad disconnected!")
                self.reset_all()
                msg = String()
                msg.data = self.format_status_json()
                self.publisher_.publish(msg)
                # time.sleep(1)


def main(args=None):
    rclpy.init(args=args)
    node = StickPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
