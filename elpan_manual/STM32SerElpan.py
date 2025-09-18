import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import threading
import time
import json


class STM32DuplexNode(Node):
    def __init__(self):
        super().__init__('stm32_duplex_node')

        self.port = '/dev/STM32_1'
        self.baud = 921600
        self.ser = None

        self.rawenc1 = 0
        self.rawenc2 = 0
        self.rawenc3 = 0
        self.lim1 = 0
        self.lim2 = 0
        self.lim3 = 0


        self.lock = threading.Lock()
        self.last_cmd = ""
        self.running = True

        self.publisher_ = self.create_publisher(String, 'stm32f4serial', 10)
        self.create_subscription(String, 'motor_cmd', self.motor_cmd_callback, 1)

        self.connect_serial()

        self.read_thread = threading.Thread(target=self.read_serial_loop, daemon=True)
        self.write_thread = threading.Thread(target=self.write_serial_loop, daemon=True)
        self.read_thread.start()
        self.write_thread.start()

    def connect_serial(self):
        while self.ser is None:
            try:
                self.ser = serial.Serial(self.port, self.baud)
                self.get_logger().info(f"✅ Serial terbuka di {self.port}")
            except Exception as e:
                self.get_logger().warn(f"❌ Gagal membuka serial: {e}. Coba lagi...")
                time.sleep(1)

    def motor_cmd_callback(self, msg):
        with self.lock:
            self.last_cmd = msg.data

    def read_serial_loop(self):
        while self.running:
            try:
                if self.ser and self.ser.in_waiting > 0:
                    line = self.ser.readline().decode("utf-8").strip()
                    if line:
                        # format: enc1 | enc2 | enc3 | lim1 | lim2 | lim3
                        parts = line.split("|")
                        if len(parts) == 6:
                            try:
                                self.rawenc1 = int(parts[0].strip())
                                self.rawenc2 = int(parts[1].strip())
                                self.rawenc3 = int(parts[2].strip())
                                self.lim1 = int(parts[3].strip())
                                self.lim2 = int(parts[4].strip())
                                self.lim3 = int(parts[5].strip())

                                data = {
                                    "enc1": self.rawenc1,
                                    "enc2": self.rawenc2,
                                    "enc3": self.rawenc3,
                                    "lim1": self.lim1,
                                    "lim2": self.lim2,
                                    "lim3": self.lim3
                                }
                                msg = String()
                                msg.data = json.dumps(data)   # convert dict ke JSON string
                                self.publisher_.publish(msg)

                            except ValueError:
                                self.get_logger().warn(f"⚠️ Data tidak valid: {line}")
                        # else: 
                            # self.get_logger().warn(f"⚠️ Format salah: {line}")
            except Exception as e:
                self.get_logger().error(f"❌ Error serial read: {e}")
                self.reconnect_serial()

    def write_serial_loop(self):
        while self.running:
            if self.ser and self.ser.is_open:
                try:
                    with self.lock:
                        if self.last_cmd:
                            self.ser.write((self.last_cmd + '\n').encode())
                            # self.get_logger().info(f"📤 Kirim: {self.last_cmd}")
                    time.sleep(0.01)
                except Exception as e:
                    self.get_logger().error(f"❌ Write error: {e}")
                    self.reconnect_serial()

    def reconnect_serial(self):
        self.get_logger().warn("🔁 Reconnecting STM32 serial...")
        try:
            if self.ser:
                self.ser.close()
        except:
            pass
        self.ser = None
        self.connect_serial()

    def destroy_node(self):
        self.running = False
        if self.ser and self.ser.is_open:
            self.ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = STM32DuplexNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("⛔ Dihentikan oleh pengguna.")
    finally:
        node.destroy_node()
        node.read_thread.join()
        node.write_thread.join()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
