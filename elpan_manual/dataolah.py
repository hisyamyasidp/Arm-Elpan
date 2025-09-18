import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Int32MultiArray
import json
import math

class PIDController:
    def __init__(self, Kp, Ki, Kd, T,
                 tau=0.02,
                 limMin=-1024, limMax=1024,
                 limMinInt=-300, limMaxInt=300):
        
        # Gain
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

        # Timing
        self.T = T          # Sampling time
        self.tau = tau      # Derivative filter time constant

        # Output limits
        self.limMin = limMin
        self.limMax = limMax

        # Integrator limits
        self.limMinInt = limMinInt
        self.limMaxInt = limMaxInt

        # Internal states
        self.integrator = 0.0
        self.prevError = 0.0
        self.differentiator = 0.0
        self.prevMeasurement = 0.0
        self.out = 0.0

    def update(self, setpoint, measurement):
        error = setpoint - measurement

        # Proportional term
        proportional = self.Kp * error

        # Integral term with anti-windup
        self.integrator += 0.5 * self.Ki * self.T * (error + self.prevError)
        self.integrator = max(self.limMinInt, min(self.integrator, self.limMaxInt))

        # Derivative term (band-limited differentiator)
        self.differentiator = -(2.0 * self.Kd * (measurement - self.prevMeasurement)
                                + (2.0 * self.tau - self.T) * self.differentiator) \
                              / (2.0 * self.tau + self.T)

        # PID output with limits
        self.out = proportional + self.integrator + self.differentiator
        self.out = max(self.limMin, min(self.out, self.limMax))

        # Update state
        self.prevError = error
        self.prevMeasurement = measurement

        return self.out

class DataSendNode(Node):
    def __init__(self):
        super().__init__('datasend_node')

        self.create_subscription(Int32MultiArray, 'yolocam', self.yolo_callback, 1)
        self.create_subscription(String, 'stm32f4serial', self.stm32_callback, 1)
        self.motor_publisher = self.create_publisher(String, 'motor_cmd', 1)
        self.enc_publisher = self.create_publisher(String, 'encdata', 1)

        # Simpan hasil koordinat YOLO
        self.rel_x = None
        self.rel_y = None

        self.enc1 = self.enc2 = self.enc3 = 0
        self.rawenc1 = self.rawenc2 = self.rawenc3 = 0
        self.enc1_offset = self.enc2_offset = self.enc3_offset = 0
        
        self.lim1 = self.lim2 = self.lim3 = 0

        self.speed1 = 0
        self.speed2 = 0
        self.speed3 = 0

        self.homing_done = False
        self.homing_step = 1   # 1=homing motor1, 2=motor2, 3=motor3, 4=selesai

        self.create_timer(0.001, self.loop) 

        self.pid_x = PIDController(
            Kp=2,  #2
            Ki=0.05,  #0.2
            Kd=0.05, #0.001
            T=0.01,
            tau=0.05,
            limMin=-200,
            limMax=200,
            limMinInt=-100,
            limMaxInt=100
        ) 
        self.pid_y = PIDController(
            Kp=4,
            Ki=0.04,
            Kd=0.000,
            T=0.01,
            tau=0.05,
            limMin=-200,
            limMax=200,
            limMinInt=-100,
            limMaxInt=100
        )
    def map_value(self, value, from_min, from_max, to_min, to_max):
        return (value - from_min) * (to_max - to_min) / (from_max - from_min) + to_min

    def constrain(self, value, min_val, max_val):
        return max(min_val, min(value, max_val))
    
    def yolo_callback(self, msg):
        if len(msg.data) >= 2:
            self.rel_x = msg.data[0]
            self.rel_y = msg.data[1]
            # self.get_logger().info(f"YOLO masuk: {msg.data}")

            # self.rel_x = 50
            # self.rel_y = 50

    def stm32_callback(self, msg):
        try:
            data = json.loads(msg.data)
            self.rawenc1 = data["enc1"]
            self.rawenc2 = data["enc2"]
            self.rawenc3 = data["enc3"]
            self.lim1 = data["lim1"]
            self.lim2 = data["lim2"]
            self.lim3 = data["lim3"]
        except Exception as e:
            self.get_logger().warn(f"⚠️ JSON error: {e}")

    def homing(self):
        # Motor 1
        if self.homing_step == 1:
            if self.lim3 == 0:
                self.speed1 = -150
            else:
                self.resetenc1()
                self.speed1 = 0
                self.homing_step = 2
                self.get_logger().info("✅ Motor1 homing selesai")

        # Motor 2
        elif self.homing_step == 2:
            if self.lim1 == 0:
                self.speed2 = 150
            else:
                self.resetenc2()
                self.speed2 = 0
                self.homing_step = 3
                self.get_logger().info("✅ Motor2 homing selesai")

        # Motor 3
        elif self.homing_step == 3:
            if self.lim2 == 0:
                self.speed3 = -100
            else:
                self.resetenc3()
                self.speed3 = 0
                self.homing_step = 4
                self.get_logger().info("✅ Motor3 homing selesai")

        # Semua motor sudah homing
        elif self.homing_step == 4:
            self.homing_done = True
            self.homing_step = 5
            self.get_logger().info("🎉 Semua motor homing selesai")

    def resetenc1(self):
        self.enc1_offset = self.rawenc1
    def resetenc2(self):
        self.enc2_offset = self.rawenc2
    def resetenc3(self):
        self.enc3_offset = self.rawenc3

    def runreset(self):
        if self.lim1 == 1:
            # self.speed2 =-150
            # if self.lim1 == 0:
            self.resetenc1()
                
        if self.lim2 == 1:
            self.resetenc2()
        if self.lim3 == 1:
            self.resetenc3()

    def encoderlimit(self):
        if self.homing_done:
            # Batasi speed berdasarkan encoder
            # Motor 1
            if self.enc3 <= 0 and self.speed1 < 0:
                self.speed1 = 0  # Stop kalau mau mundur tapi udah mentok bawah
            elif self.enc3 >= 1700 and self.speed1 > 0:
                self.speed1 = 0  # Stop kalau mau maju tapi udah mentok atas

            # Motor 2
            if self.enc1 <= 0 and self.speed2 > 0:
                self.speed2 = 0
            elif self.enc1 >= 1300 and self.speed2 < 0:
                self.speed2 = 0

            # motor 3
            # posisi idlenya 1000
            if self.enc2 <= 0 and self.speed3 < 0:
                self.speed3 = 0
            elif self.enc2 >= 1600 and self.speed3 > 0:
                self.speed3 = 0

    def autoyolo(self, rel_x, rel_y):
    # Deadband biar gak ngejar sampai error 0 persis
        if abs(rel_x) < 5:   # tolerance X
            rel_x = 0
        if abs(rel_y) < 5:   # tolerance Y
            rel_y = 0

        self.speed1 = self.pid_x.update(0, rel_x)
        self.speed2 = self.pid_y.update(0, rel_y)
    
    def loop(self):

        self.runreset()

        self.enc1 = self.rawenc1 - self.enc1_offset
        self.enc2 = self.rawenc2 - self.enc2_offset
        self.enc3 = self.rawenc3 - self.enc3_offset

        
        if not self.homing_done:
            self.homing()
        else:
            if self.rel_x is not None and self.rel_y is not None:
                self.autoyolo(self.rel_x, self.rel_y)
                
                # self.get_logger().info(f"{self.rel_y}")
            else:
                # YOLO hilang → motor stop
                self.speed1 = 0
                self.speed2 = 0
                self.speed3 = 0

        
        self.encoderlimit()

        # self.get_logger().info(f"{self.speed2}")
        self.speed1 = int(self.speed1)
        self.speed2 = int(self.speed2)
        self.speed3 = int(self.speed3)

        # self.get_logger().info(f"{self.rel_x, self.rel_y}")
        motor_msg = String()
        motor_msg.data = f"*{self.speed1:05d} {self.speed2:05d} {self.speed3:05d}#"
     
        self.motor_publisher.publish(motor_msg)

        enc_msg = String()
        enc_msg.data = f"{self.enc1} {self.enc2} {self.enc3} {self.homing_step}"
        self.enc_publisher.publish(enc_msg)

def main(args=None):
    rclpy.init(args=args)
    node = DataSendNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info(" 🚩 Dihentikan oleh pengguna.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()