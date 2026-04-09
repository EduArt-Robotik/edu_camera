START_PATH="/gp/gpWebcam/START"
STOP_PATH="/gp/gpWebcam/STOP"
FOV_PATH="/gp/gpWebcam/SETTINGS?fov="
curl -s "${GOPRO_INTERFACE_IP}${START_PATH}?res=1080p&fov=linear&port=5001"
curl -s "172.28.124.51${START_PATH}?res=1080p&fov=linear&port=5001"
curl -s "172.26.179.51${START_PATH}?res=1080p&fov=linear&port=5002"
ffmpeg -nostdin -threads 1 -i 'udp://@0.0.0.0:5002?overrun_nonfatal=1&fifo_size=50000000' -f:v mpegts -fflags nobuffer -vf format=yuv420p -f v4l2 /dev/video43

