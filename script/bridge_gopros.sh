#!/bin/bash
# This script is used to bridge the GoPro cameras as v4l2 device
# Its without generalization and only for the InInspekt prototype

# First bring up v4l2loopback
GOPRO_LEFT_INTERFACE='inspect_left'
GOPRO_RIGHT_INTERFACE='inspect_right'

modprobe v4l2loopback video_nr=42,43 card_label=${GOPRO_LEFT_INTERFACE},${GOPRO_RIGHT_INTERFACE}

# Then start the gopro stream and pipe it to ffmpeg to convert it to v4l2 format
# The gopro stream is in h264 format and ffmpeg will convert it to yuv format and pipe it to the v4l2loopback device
RESOLUTION='1080p'
FPS='5'
GOPRO_LEFT_IP='172.28.124.51'
GOPRO_LEFT_PORT='5001'
GOPRO_RIGHT_IP='172.26.179.51'
GOPRO_RIGHT_PORT='5002'

## Starting gopro stream via http request
## GoPro will send a h264 udp stream to host
START_PATH="/gp/gpWebcam/START"
STOP_PATH="/gp/gpWebcam/STOP"
FOV_PATH="/gp/gpWebcam/SETTINGS?fov="

curl -s "http://${GOPRO_LEFT_IP}${START_PATH}?res=${RESOLUTION}&fov=linear&port=${GOPRO_LEFT_PORT}&fps=${FPS}"
curl -s "http://${GOPRO_RIGHT_IP}${START_PATH}?res=${RESOLUTION}&fov=linear&port=${GOPRO_RIGHT_PORT}&fps=${FPS}"

## Using ffmepg to forward and convert the h264 video stream to a yuvu stream on video device
ffmpeg -nostdin -threads 1 -i 'udp://@0.0.0.0:5001?overrun_nonfatal=1&fifo_size=50000000' -f:v mpegts -fflags nobuffer -vf format=yuv420p -f v4l2 /dev/video42 &
ffmpeg -nostdin -threads 1 -i 'udp://@0.0.0.0:5002?overrun_nonfatal=1&fifo_size=50000000' -f:v mpegts -fflags nobuffer -vf format=yuv420p -f v4l2 /dev/video43
