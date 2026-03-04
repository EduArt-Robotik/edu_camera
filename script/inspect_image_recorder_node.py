# --- ROS2 Node Main ---
import rclpy
import image_transport

from rclpy.node import Node
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger
from nav_msgs.msg import Odometry

class InspectImageRecorderNode(Node):
  def __del__(self):
    if hasattr(self, 'cap_left') and self.cap_left is not None:
      self.cap_left.release()
    if hasattr(self, 'cap_right') and self.cap_right is not None:
      self.cap_right.release()

  def __init__(self):
    super().__init__('inspect_image_recorder_node')

    # TODO: parameter for camera index and output directory
    self.camera_left_index = 42
    self.camera_right_index = 43
    self.output_dir = "/home/user/Documents/inspect_images/"
    self.output_dir_left = os.path.join(self.output_dir, 'left')
    self.output_dir_right = os.path.join(self.output_dir, 'right')
    self.counter = 1
    os.makedirs(self.output_dir_left, exist_ok=True)
    os.makedirs(self.output_dir_right, exist_ok=True)

    self.recording = False
    self.interval_send_image = 2.0  # seconds
  
    # open camera and start recording
    self.cap_left = cv2.VideoCapture(self.camera_left_index)
    if not self.cap_left.isOpened():
      self.get_logger().error("error opening camera.")
      exit(1)

    self.cap_right = cv2.VideoCapture(self.camera_right_index)
    if not self.cap_right.isOpened():
      self.get_logger().error("error opening camera.")
      exit(1)

    # creating services and topics
    self.srv_start_stop = self.create_service(Trigger, 'start_stop_recording', self.start_stop_recording)

    self.image_transport_left = image_transport.ImageTransport(self)
    self.image_transport_right = image_transport.ImageTransport(self)
    self.pub_image_left = self.image_transport_left.advertise('inspect/image_left', 2)
    self.pub_image_right = self.image_transport_right.advertise('inspect/image_right', 2)
    self.last_publish_time = self.get_clock().now()
    
    self.sub_odometry = self.create_subscription(Odometry, 'odometry', self.callback_odometry, 2)

  def capture_images(self):
    ret_left, frame_left = self.cap_left.read()
    ret_right, frame_right = self.cap_right.read()

    if not ret_left or not ret_right:
      self.get_logger().error("error reading camera image.")
      return

    # publish images
    timestamp = time.strftime("%Y.%m.%d_%H:%M")
    filename_left = os.path.join(self.output_dir_left, f"{timestamp}_inspect_left.jpg")
    filename_right = os.path.join(self.output_dir_right, f"{timestamp}_inspect_right.jpg")
    cv2.imwrite(filename_left, frame_left)
    cv2.imwrite(filename_right, frame_right)
    self.get_logger().info(f"image saved: {filename_left}")
    self.get_logger().info(f"image saved: {filename_right}")

    # publish images to topics (for visualization in rqt_image_view)
    if (self.get_clock().now() - self.last_publish_time).nanoseconds > self.interval_send_image * 1e9:
      msg_left = Image()
      msg_right = Image()
      self.pub_image_left.publish(msg_left)
      self.pub_image_right.publish(msg_right)
      self.last_publish_time = self.get_clock().now()

  def start_stop_recording(self, request, response):
    self.recording = not self.recording
    response.success = True
    response.message = "recording started" if self.recording else "recording stopped"
    return response
    
  def callback_odometry(self, msg):
    if (msg.pose.pose.position.x < self.distance_threshold):
      return

    if (not self.recording):
      return
      
    # recoding is active and distance threshold is reached, capture images
    self.get_logger().info("distance threshold reached --> triggering image capture")
    self.distance_threshold += 0.2 # trigger every 20cm
    self.capture_images()


def main(args=None):
  rclpy.init(args=args)
  node = InspectImageRecorderNode()
  try:
    rclpy.spin(node)
  except KeyboardInterrupt:
    pass
  finally:
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
  main()


# # Kamera-Index (0 = Standardkamera)
# camera_index = 43
# output_dir = "/tmp/"
# os.makedirs(output_dir, exist_ok=True)

# cap = cv2.VideoCapture(camera_index)
# if not cap.isOpened():
#     print("Kamera konnte nicht geöffnet werden.")
#     exit(1)

# try:
#     count = 0
#     while True:
#         ret, frame = cap.read()
#         if not ret:
#             print("Fehler beim Lesen des Kamerabildes.")
#             break
#         cv2.imshow('Kamera', frame)
#         if cv2.waitKey(1) & 0xFF == ord('q'):
#             break
#         filename = os.path.join(output_dir, f"snapshot_{count:04d}.jpg")
#         cv2.imwrite(filename, frame)
#         print(f"Bild gespeichert: {filename}")
#         count += 1
#         time.sleep(1)
# except KeyboardInterrupt:
#     print("Aufnahme beendet.")
# finally:
#     cap.release()
