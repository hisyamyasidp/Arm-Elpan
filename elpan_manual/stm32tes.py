import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import threading
import time

class STM32DuplexNode(Node):
    def __init__(self):
        super().__init__('stm32_duplex_node')

        self.port = '/dev/STM32_1'
        self.baud = 921600
        self.ser = None
        self.running = True

        self.enc1 = self.enc2 = self.enc3 = 0
        self.rawenc1 = self.rawenc2 = self.rawenc3 = 0
        self.enc1_offset = self.enc2_offset = self.enc3_offset = 0   
        self.lim1 = self.lim2 = self.lim3 = 0
        
        self.connect_serial()

        self.timer = self.create_timer(0.5, self.write_serial)
        self.timer_loop = self.create_timer(0.01, self.loop)  # 100 Hz


        # Buat thread baca serial
        self.read_thread = threading.Thread(target=self.read_serial, daemon=True)
        self.read_thread.start()

    # ----------------- RESET -----------------
    def resetenc1(self):
        self.enc1_offset = self.rawenc1
        # self.enc1 = self.rawenc1 - self.enc1_offset
    def resetenc2(self):
        self.enc2_offset = self.rawenc2
        # self.enc2 = self.rawenc2 - self.enc2_offset
    def resetenc3(self):
        self.enc3_offset = self.rawenc3
        # self.enc3 = self.rawenc3 - self.enc3_offset

    def runreset(self):
        if self.lim1 == 1:
            self.resetenc1()
        if self.lim2 == 1:
            self.resetenc2()
        if self.lim3 == 1:
            self.resetenc3()

    def loop(self):
        self.runreset()

        self.enc1 = self.rawenc1 - self.enc1_offset
        self.enc2 = self.rawenc2 - self.enc2_offset
        self.enc3 = self.rawenc3 - self.enc3_offset

        

    # ----------------- SERIAL -----------------
    def connect_serial(self):
        while self.ser is None:
            try:
                self.ser = serial.Serial(self.port, self.baud, timeout=0.1)
                self.get_logger().info(f"✅ Serial terbuka di {self.port}")
            except Exception as e:
                self.get_logger().warn(f"❌ Gagal membuka serial: {e}. Coba lagi...")
                time.sleep(1)

    def reconnect_serial(self):
        self.get_logger().warn("🔁 Reconnecting STM32 serial...")
        try:
            if self.ser:
                self.ser.close()
        except:
            pass
        self.ser = None
        self.connect_serial()

    def write_serial(self):
        """Contoh: ngirim data tetap ke STM32"""
        try:
            if self.ser and self.ser.is_open:
                msg = "HELLO_STM32|123|456|789\n"
                self.ser.write(msg.encode("utf-8"))
                self.get_logger().info(f"TX: {msg.strip()}")
        except Exception as e:
            self.get_logger().error(f"❌ Error serial write: {e}")
            self.reconnect_serial()

    def read_serial(self):
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

                                self.get_logger().info(
                                    f"RX enc=({self.rawenc1}, {self.rawenc2}, {self.rawenc3}) "
                                    f"lim=({self.lim1}, {self.lim2}, {self.lim3})"
                                    f"RX enc=({self.enc1}, {self.enc2}, {self.enc3}) "
                                )
                            except ValueError:
                                self.get_logger().warn(f"⚠️ Data tidak valid: {line}")
                        else:
                            self.get_logger().warn(f"⚠️ Format salah: {line}")
            except Exception as e:
                self.get_logger().error(f"❌ Error serial read: {e}")
                self.reconnect_serial()

    # ----------------- CLEANUP -----------------
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
        rclpy.shutdown()


if __name__ == '__main__':
    main()
