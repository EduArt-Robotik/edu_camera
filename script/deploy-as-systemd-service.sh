#!/bin/sh
if [ "$(whoami)" != "root" ]; then
  echo "This script requires root privileges!"
  exit 1
fi

bridge_file_path=$(pwd)
systemd_service_file="bridge_gopros.service"
tag="<bridge_file_path>"

echo "Deploying GoPro bridge as systemd service"

# First installing systemd service on system.
echo "Installing systemd service on system"
cp $systemd_service_file /etc/systemd/system/

# Modify systemd service so it runs the correct docker compose file.
echo "Add current file path \"$bridge_file_path\" to systemd service."
sed -i "s|$tag|$bridge_file_path|g" /etc/systemd/system/$systemd_service_file

# Starting up service.
systemctl daemon-reload
systemctl start $systemd_service_file
systemctl enable $systemd_service_file
echo "Systemd service was installed and enabled, so it will start automatically at boot up."
