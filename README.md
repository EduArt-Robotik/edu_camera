# edu_camera
Reading data from cameras and stream it on network.

## Dependencies

### System Dependencies
- **GStreamer 1.0** - Multimedia framework for video streaming
  ```bash
  sudo apt-get install -y \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    libgstreamer-plugins-bad1.0-dev \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-plugins-ugly \
    gstreamer1.0-libav \
    gstreamer1.0-tools \
    gstreamer1.0-x
  ```

### ROS 2 Dependencies
- rclcpp
- sensor_msgs
- cv_bridge
- image_transport
- camera_info_manager
- OpenCV

## Features
- Video streaming via GStreamer with UDP protocol
- Adaptive bitrate encoding (H.264)
- Quality settings (bitrate, resolution, fps)
- Support for VideoGstreamOutput (sender) and VideoGstreamInput (receiver)

## Debug Stream

```bash
gst-launch-1.0 -v udpsrc port=5000 caps="application/x-rtp,media=video,clock-rate=90000,encoding-name=H264,payload=96"   ! rtph264depay ! avdec_h264 ! videoconvert ! autovideosink
```