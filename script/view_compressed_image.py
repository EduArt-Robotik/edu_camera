#!/usr/bin/env python3
"""
ROS2 Python script to display compressed image transport topics.
Subscribes to a compressed image topic and displays it using OpenCV.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np
import sys


class CompressedImageViewer(Node):
    """Node that subscribes to a compressed image topic and displays it."""

    def __init__(self, topic_name: str, window_name: str = "Compressed Image Viewer"):
        """
        Initialize the compressed image viewer node.
        
        Args:
            topic_name: Name of the compressed image topic to subscribe to
            window_name: Name of the OpenCV window
        """
        super().__init__('compressed_image_viewer')
        
        self.window_name = window_name
        self.get_logger().info(f'Subscribing to compressed image topic: {topic_name}')
        
        # Subscribe to compressed image topic
        self.subscription = self.create_subscription(
            CompressedImage,
            topic_name,
            self.image_callback,
            10
        )
        
        # Create OpenCV window
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        
        self.get_logger().info('Compressed image viewer ready. Press "q" or ESC to quit.')

    def image_callback(self, msg: CompressedImage):
        """
        Callback function for compressed image messages.
        
        Args:
            msg: CompressedImage message
        """
        try:
            # Decode compressed image data to OpenCV format
            np_arr = np.frombuffer(msg.data, np.uint8)
            image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            
            if image is None:
                self.get_logger().warn('Failed to decode compressed image')
                return
            
            # Display the image
            cv2.imshow(self.window_name, image)
            
            # Process keyboard input (1ms wait)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q') or key == 27:  # 'q' or ESC
                self.get_logger().info('Quit requested by user')
                rclpy.shutdown()
                
        except Exception as e:
            self.get_logger().error(f'Error processing compressed image: {e}')

    def destroy_node(self):
        """Cleanup when node is destroyed."""
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    """Main function to run the compressed image viewer."""
    rclpy.init(args=args)
    
    # Parse command line arguments
    if len(sys.argv) < 2:
        print("Usage: view_compressed_image.py <compressed_image_topic>")
        print("Example: view_compressed_image.py /camera/image/compressed")
        sys.exit(1)
    
    topic_name = sys.argv[1]
    window_name = f"Compressed Image: {topic_name}"
    
    # Create and run the node
    node = CompressedImageViewer(topic_name, window_name)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt detected')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
