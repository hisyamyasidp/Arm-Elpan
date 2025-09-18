import cv2
from ultralytics import YOLO
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
# from sensor_msgs.msg import Image
# from cv_bridge import CvBridge


class YOLOPublisher(Node):
    def __init__(self):
        super().__init__('yolo_publisher')
        self.publisher_ = self.create_publisher(Int32MultiArray, 'yolocam', 1)
        # self.imagepub = self.create_publisher(Image, '/image', 1)
        # self.bridge = CvBridge()

        # Load YOLO model
        model_path = "/home/hisyam/Desktop/elpan_ws/src/elpan_manual/elpan_manual/botolvn.pt"
        self.model = YOLO(model_path)

        # Retry buka kamera sampai berhasil
        self.cap = None
        while True:
            self.cap = cv2.VideoCapture("/dev/cam_elpan")
            if self.cap.isOpened():
                print("✅ Kamera berhasil dibuka")
                break
            else:
                self.cap.release()
                print("Kamera berhasil nuutup")

    def run(self):
        try:
            while rclpy.ok():
                ret, frame = self.cap.read()
                if not ret:
                    print("Error: Cannot read frame")
                    continue

                # YOLO inference
                results = self.model(frame, conf=0.5, stream=True)

                for result in results:
                    annotated_frame = result.plot()
                    break

                # Ambil ukuran frame
                h, w = annotated_frame.shape[:2]
                center_x, center_y = w // 2, h // 2

                # Gambar garis vertikal & horizontal
                # cv2.line(annotated_frame, (center_x, 0), (center_x, h), (0, 0, 255), 2)
                # cv2.line(annotated_frame, (0, center_y), (w, center_y), (0, 0, 255), 2)
                cv2.circle(annotated_frame, (center_x, center_y), 5, (0, 0, 255), -1)

                rel_x, rel_y = None, None

                # Loop semua box
                for box in result.boxes.xyxy:
                    x1, y1, x2, y2 = box
                    cx = int((x1 + x2) / 2)
                    cy = int((y1 + y2) / 2)

                    # Hitung koordinat relatif (0,0 di titik merah)
                    rel_x = cx - center_x
                    rel_y = -(cy - center_y)

                    # Titik biru di tengah box
                    cv2.circle(annotated_frame, (cx, cy), 5, (255, 0, 0), -1)
                    coord_text = f"({rel_x}, {rel_y})"
                    cv2.putText(annotated_frame, coord_text, (cx + 10, cy - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
                    break

                # Kalau ada data → publish
                if rel_x is not None and rel_y is not None:
                    msg = Int32MultiArray()
                    msg.data = [rel_x, rel_y]
                    self.publisher_.publish(msg)
                    # self.get_logger().info(f"📤 Publish koordinat: {msg.data}")
                else:
                    msg = Int32MultiArray()
                    msg.data = [0, 0]   # default kalau ga ada objek
                    self.publisher_.publish(msg)

                # img_msg = self.bridge.cv2_to_imgmsg(annotated_frame, encoding="bgr8")
                # self.imagepub.publish(img_msg)

                # Tampilkan hasil
                cv2.imshow('YOLO Detection', annotated_frame)
                cv2.waitKey(30)

        except KeyboardInterrupt:
            self.get_logger().info("🛑 Dihentikan oleh user (Ctrl+C)")
        finally:
            self.cap.release()
            cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)
    node = YOLOPublisher()
    node.run()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
